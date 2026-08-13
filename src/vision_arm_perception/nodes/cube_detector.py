#!/usr/bin/env python3
"""
Colour-blob cube detector: RGB + aligned depth -> Detection3DArray + TF.

ponytail: HSV colour segmentation, not YOLO. The scene is three saturated
cubes on a matte table; a 20-line threshold solves it exactly and runs in
~1 ms on the CPU-only dev box. The YOLO11n-ONNX path in BUILD_PLAN Phase 5
buys nothing until the objects stop being primary-coloured blocks.

ponytail: no ByteTrack either. Colour IS the stable ID here - one cube per
colour, so `obj_red` names the same physical object every frame, which is all
the tracker was for. Add supervision + ByteTrack when two objects can share a
class.
"""

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from image_geometry import PinholeCameraModel
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_point
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

import cv2

# Drawn on the debug image, BGR.
DRAW_COLOURS = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 0, 0)}


def detect_blobs(bgr, bands, min_area):
    """
    Find the largest blob per colour.

    `bands` maps a label to a flat list of HSV bounds, 6 numbers per band
    (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi). Red needs two bands because its hue
    wraps around 0/180 in OpenCV's 0-179 hue scale.

    Returns [(label, u, v, x, y, w, h)] with (u, v) the blob centroid.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out = []
    for label, flat in bands.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for i in range(0, len(flat), 6):
            lo = np.array(flat[i:i + 3], np.uint8)
            hi = np.array(flat[i + 3:i + 6], np.uint8)
            mask |= cv2.inRange(hsv, lo, hi)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        best = max(contours, key=cv2.contourArea)
        if cv2.contourArea(best) < min_area:
            continue
        x, y, w, h = cv2.boundingRect(best)
        # A blob touching the border is cut off, so its centroid is not the object's
        # centre and the depth there may belong to whatever is behind it. Measured:
        # a fully visible cube reports within 1.6 mm, the same cube clipped at the
        # frame edge reports 10.2 mm out - past the phase's 1 cm criterion. Better to
        # report nothing and let the arm look again.
        if x == 0 or y == 0 or x + w >= mask.shape[1] or y + h >= mask.shape[0]:
            continue
        m = cv2.moments(best)
        u = int(m["m10"] / m["m00"])
        v = int(m["m01"] / m["m00"])
        out.append((label, u, v, x, y, w, h))
    return out


def patch_depth(depth, u, v, half):
    """Median of the finite depths in a (2*half+1)^2 patch, or None."""
    h, w = depth.shape
    patch = depth[max(v - half, 0):min(v + half + 1, h),
                  max(u - half, 0):min(u + half + 1, w)]
    finite = patch[np.isfinite(patch) & (patch > 0.0)]
    return float(np.median(finite)) if finite.size else None


class CubeDetector(Node):
    def __init__(self):
        super().__init__("cube_detector")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("min_area", 60)
        self.declare_parameter("depth_patch", 2)
        self.declare_parameter("cube_size", 0.03)
        self.declare_parameter("hsv.red", [0, 120, 70, 10, 255, 255,
                                           170, 120, 70, 179, 255, 255])
        self.declare_parameter("hsv.green", [40, 120, 70, 80, 255, 255])
        self.declare_parameter("hsv.blue", [100, 120, 70, 130, 255, 255])

        self.bands = {c: list(self.get_parameter(f"hsv.{c}").value)
                      for c in ("red", "green", "blue")}
        self.target_frame = self.get_parameter("target_frame").value
        self.min_area = self.get_parameter("min_area").value
        self.half = self.get_parameter("depth_patch").value
        self.cube_size = self.get_parameter("cube_size").value

        self.bridge = CvBridge()
        self.model = None
        # The world model: colour -> (position in target_frame, when it was seen).
        # ponytail: a dict, not a tracker. No single look pose frames all three
        # cubes (84 deg FOV, cubes spread 0.24 m across the bench), so the arm sweeps
        # and the poses accumulate here. Positions are already in base_link and
        # the cubes are static, so a pose stays true after it leaves the frame -
        # which is exactly what the eye-in-hand look -> decide -> commit sequence
        # needs, since the jaws blind the camera on the way down. Nothing
        # expires; add a timeout when something else can move the cubes.
        self.world = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.detections_pub = self.create_publisher(
            Detection3DArray, "detections", 10)
        self.debug_pub = self.create_publisher(Image, "detections/debug", 1)
        self.create_subscription(
            CameraInfo, "camera/camera_info", self.on_info, 10)
        sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, "camera/image"),
             Subscriber(self, Image, "camera/depth_image")],
            queue_size=5, slop=0.1)
        sync.registerCallback(self.on_frame)
        self.sync = sync  # keep alive

    def on_info(self, msg):
        if self.model is None:
            self.model = PinholeCameraModel()
            self.model.fromCameraInfo(msg)

    def on_frame(self, image_msg, depth_msg):
        if self.model is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        for label, u, v, x, y, w, h in detect_blobs(
                bgr, self.bands, self.min_area):
            colour = DRAW_COLOURS[label]
            cv2.rectangle(bgr, (x, y), (x + w, y + h), colour, 1)
            z = patch_depth(depth, u, v, self.half)
            if z is None:
                continue
            point = self.to_target(image_msg.header, u, v, z)
            if point is None:
                continue
            self.world[label] = (point, image_msg.header.stamp)
            cv2.putText(bgr, f"{label} {z:.2f}m", (x, max(y - 3, 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, colour, 1)

        self.publish_world(image_msg.header.stamp)
        debug = self.bridge.cv2_to_imgmsg(bgr, "bgr8")
        debug.header = image_msg.header
        self.debug_pub.publish(debug)

    def publish_world(self, stamp):
        """
        Publish every remembered object, whether or not it is in frame now.

        Each Detection3D keeps the stamp it was actually seen at, so a consumer
        can tell a fresh look from a remembered one. The array header and the TF
        frames carry the current stamp - the cubes are static, so asserting
        "it is there now" is true, and TF consumers reject old transforms.
        """
        array = Detection3DArray()
        array.header.stamp = stamp
        array.header.frame_id = self.target_frame
        now = Header(stamp=stamp, frame_id=self.target_frame)
        for label, (point, seen_at) in self.world.items():
            seen = Header(stamp=seen_at, frame_id=self.target_frame)
            array.detections.append(self.make_detection(seen, label, point))
            self.send_object_tf(now, label, point)
        self.detections_pub.publish(array)

    def to_target(self, header, u, v, z):
        """Deproject pixel + depth, then transform into the target frame."""
        ray = self.model.projectPixelTo3dRay(self.model.rectifyPoint((u, v)))
        # Depth lands on the cube's visible *surface*; the centre is half a cube
        # further along the same ray. Without this the report is biased ~15 mm
        # toward the camera, outside the 1 cm accuracy the phase asks for.
        z += self.cube_size / 2.0
        stamped = PointStamped()
        stamped.header = header
        stamped.point.x = ray[0] * z / ray[2]
        stamped.point.y = ray[1] * z / ray[2]
        stamped.point.z = z
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"TF unavailable: {exc}", throttle_duration_sec=5.0)
            return None
        return do_transform_point(stamped, tf).point

    def make_detection(self, header, label, point):
        detection = Detection3D()
        detection.header = header
        detection.id = f"obj_{label}"
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label
        hypothesis.hypothesis.score = 1.0
        hypothesis.pose.pose.position = point
        hypothesis.pose.pose.orientation.w = 1.0
        detection.results.append(hypothesis)
        detection.bbox = BoundingBox3D()
        detection.bbox.center = hypothesis.pose.pose
        detection.bbox.size.x = self.cube_size
        detection.bbox.size.y = self.cube_size
        detection.bbox.size.z = self.cube_size
        return detection

    def send_object_tf(self, header, label, point):
        tf = TransformStamped()
        tf.header = header
        tf.child_frame_id = f"obj_{label}"
        tf.transform.translation.x = point.x
        tf.transform.translation.y = point.y
        tf.transform.translation.z = point.z
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
