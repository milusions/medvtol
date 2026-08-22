from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("medvtol_description"), "urdf", "medvtol.urdf.xacro")
    )
    autopod_description_dir = get_package_share_directory("medvtol_description")
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    gazebo_resource_path = SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=[str(Path(autopod_description_dir).parent.resolve())])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            
            os.path.join(
               get_package_share_directory("ros_gz_sim"),
                "launch"
            ), "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", " -v 4 -r empty.sdf")
        ]
    )
    gz_spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    output="screen",
    arguments=[
        "-world", "empty",
        "-topic", "robot_description",
        "-name", "medvtol",
        "-x", "0.0",
        "-y", "0.0",
        "-z", "1.5"  # Sets the spawning height to 1.5 meters
    ]
    )
    
    gz_ros_imu_topic = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
      parameters=[{
            "config_file": os.path.join(
                autopod_description_dir,
                "config",
                "bridge.yaml"
            )
        }]
    )

    falcon_state_estimation = Node(
        package="falcon_state_estimation",
        executable="attitude_filter",
        output="screen"
    )
    
    falcon_inner_loop = Node(
        package="falcon_inner_loop",
        executable="inner_loop",
        output="screen"
    )
    
    pid_setpoint_parser = Node(
        package="falcon_inner_loop",
        executable="pid_setpoint_parser",
        output="screen"
    )
    
    
    delayed_inner_loop_spawn = TimerAction(
        actions=[falcon_inner_loop],
        period=5.0
    )
    airlink_websocket_server = Node(
        package="airlink",
        executable="airlink_websocket_server")
    
    airlink_static_server = Node(
        package="airlink",
        executable="airlink_static_server")
    return LaunchDescription(
        [model_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros_imu_topic,
    
        airlink_websocket_server,
        airlink_static_server,
        falcon_state_estimation,
        pid_setpoint_parser,
        delayed_inner_loop_spawn,
      
      
        ]
        );
