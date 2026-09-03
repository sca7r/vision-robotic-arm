"""Publish a grasp correction whenever the detector reports a cube.

This is the deployment half. It keeps torch out of vision_arm_tasks entirely:
the task server only ever sees three numbers on /grasp_residual, and clips them,
so it neither knows nor cares whether a network or a human produced them. The
same topic is what the training environment drives, which means training and
deployment exercise one code path rather than two.

    ros2 run vision_arm_rl policy_node --ros-args -p model:=models/residual.zip
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection3DArray

from .bench import RESIDUAL_LIMIT
from .residual_env import ResidualGraspEnv


class PolicyNode(Node):
    def __init__(self):
        super().__init__("grasp_residual_policy")
        self.declare_parameter("model", "")
        self.declare_parameter("colour", "red")
        path = self.get_parameter("model").value
        if not path:
            raise RuntimeError("set the model parameter to a checkpoint path")

        from stable_baselines3 import SAC
        self.policy = SAC.load(path)
        self.colour = self.get_parameter("colour").value
        self.pub = self.create_publisher(Float64MultiArray, "/grasp_residual", 10)
        self.create_subscription(Detection3DArray, "/detections", self._on_detections, 10)
        self.get_logger().info(f"publishing grasp residuals from {path}")

    def _on_detections(self, msg):
        for d in msg.detections:
            if d.results[0].hypothesis.class_id != self.colour:
                continue
            p = d.results[0].pose.pose.position
            obs = ResidualGraspEnv.normalise((p.x, p.y, p.z))
            action, _ = self.policy.predict(obs, deterministic=True)
            residual = np.clip(action, -1.0, 1.0) * RESIDUAL_LIMIT
            self.pub.publish(Float64MultiArray(data=[float(v) for v in residual]))


def main():
    rclpy.init()
    node = PolicyNode()
    rclpy.spin(node)


if __name__ == "__main__":
    sys.exit(main())
