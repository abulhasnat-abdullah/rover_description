import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    rover_description = get_package_share_directory("rover_description")
    gz_bridge_config_path = os.path.join(
        rover_description, "config", "gz_stereolabs_zed_remappings.yaml"
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gz_bridge_config_path}],
        output="screen",
    )

    return LaunchDescription([gz_bridge])