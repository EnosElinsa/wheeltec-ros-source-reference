# Copyright (c) 2026 WHEELTEC ROS Textbook Contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from setuptools import find_packages, setup


package_name = "py_pubsub"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="WHEELTEC ROS Textbook Contributors",
    maintainer_email="maintainers@wheeltec-ros-textbook.invalid",
    description="Python publisher and subscriber exercises for the WHEELTEC ROS textbook.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "talker = py_pubsub.publisher:main",
            "listener = py_pubsub.subscriber:main",
        ],
    },
)
