import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()
    
    # Nodes to launch
    drive_core = Node(
        package="locomotion_core",
        executable="movebase_kinematics",
    )

    gps_core = Node(
        package="rsl_gps",
        executable="run_gps",
    )
    cmd_roboteq = Node(
        package="locomotion_core",
        executable="cmd_roboteq",
    )
    navigation_core = Node(
        package="navigation_core",
            executable="run_navigation",
            output="screen",
            emulate_tty=True)
    # include another launch file
    launch_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('teleop_twist_joy'),
                'launch/teleop-launch.py'))
    )
    

    # ld.add_action(launch_joy)
    
    ld.add_action(drive_core)
    ld.add_action(navigation_core)
    ld.add_action(cmd_roboteq)
    ld.add_action(gps_core)


    return ld


