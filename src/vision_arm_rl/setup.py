from setuptools import find_packages, setup

package_name = "vision_arm_rl"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="harsh",
    maintainer_email="harsh23patil96@gmail.com",
    description="Learned residual correction to the grasp pose",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "policy_node = vision_arm_rl.policy_node:main",
        ],
    },
)
