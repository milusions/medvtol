from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    companion_controller = Node(
        package="medvtol_companion",
        executable="companion_controller",
       
    )
    
    mission_manager = Node(
        package="medvtol_companion",
        executable="mission_manager",
       
    )
    
    control_panel_ws = Node(
        package="medvtol_control_panel",
        executable="control_panel_ws",
       
    )
    
    control_panel_static = Node(
        package="medvtol_control_panel",
        executable="control_panel_static",
       
    )
   
    return LaunchDescription(
        [
        companion_controller,
        mission_manager,
control_panel_ws,
control_panel_static
        ]
        );
