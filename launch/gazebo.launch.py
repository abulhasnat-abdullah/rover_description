import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Changed package name from "rover_description" to "rover_description"
    rover_description = get_package_share_directory("rover_description")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(rover_description, "urdf", "rover_description.urdf.xacro"), # Changed to .urdf.xacro for XACRO processing
        description="Absolute path to robot urdf file"
    )
    
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(rover_description).parent.resolve())]
    )
    
    # Process XACRO file and get robot description
    def get_robot_description():
        urdf_file_path = os.path.join(rover_description, "urdf", "rover_description.urdf.xacro")
        robot_description_content = Command(['xacro ', urdf_file_path])
        return robot_description_content
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": get_robot_description(), # Process XACRO content
            "use_sim_time": True
        }]
    )

    joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    name="joint_state_publisher",
    parameters=[{"use_sim_time": True}]
)
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4", " -r", " empty.sdf"])
        ]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "rover" # Changed robot name from "rover" to "rover"
        ],
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ]
    )
    
    # ZED Camera Gazebo Bridge (integrated from zed.launch.py)
    # Using direct arguments instead of config file
    zed_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="zed_gz_bridge",
        arguments=[
            "/zed/zed_node/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/zed/zed_node/rgb/image_rect_color@sensor_msgs/msg/Image[gz.msgs.Image",
            "/zed/zed_node/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/zed/zed_node/depth@sensor_msgs/msg/Image[gz.msgs.Image",
            "/zed/zed_node/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
    )
    
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        joint_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        zed_gz_bridge,
    ])