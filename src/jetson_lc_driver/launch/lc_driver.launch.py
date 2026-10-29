from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("jetson_lc_driver"), "config", "lc_driver.yaml")
    return LaunchDescription([
        Node(package="jetson_lc_driver", executable="lc_driver_node",
             name="lc_driver", output="screen", parameters=[config])
    ])
