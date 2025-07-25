# piper_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription  # Correct import method
from launch_ros.actions import Node  # Remains unchanged
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Get the path to the piper_description package
    piper_description_path = os.path.join(
        get_package_share_directory('piper_description'),
        'launch',
        'piper_with_gripper',
        'display_xacro.launch.py'
    )

    # Define launch parameters
    left_can_port_arg = DeclareLaunchArgument(
        'left_can_port',
        default_value='can_left',
        description='CAN port for the robot arm'
    )
    right_can_port_arg = DeclareLaunchArgument(
        'right_can_port',
        default_value='can_right',
        description='CAN port for the robot arm'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Enable robot arm automatically'
    )

    # Include display_xacro.launch.py
    display_xacro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_description_path)
    )

    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        'rviz_ctrl_flag',
        default_value='true',
        description='Start rviz flag.'
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Gripper existence flag'
    )
    
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='2',
        description='gripper'
    )

    # Define the robot arm node
    left_piper_ctrl_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='left_piper_ctrl_single_node',
        output='screen',
        namespace='left_arm',
        parameters=[
            {'can_port': LaunchConfiguration('left_can_port')},
            {'auto_enable': LaunchConfiguration('auto_enable')},
            {'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple')},
            {'gripper_exist': LaunchConfiguration('gripper_exist')}
        ],
    )

    right_piper_ctrl_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='right_piper_ctrl_single_node',
        output='screen',
        namespace='right_arm',
        parameters=[
            {'can_port': LaunchConfiguration('right_can_port')},
            {'auto_enable': LaunchConfiguration('auto_enable')},
            {'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple')},
            {'gripper_exist': LaunchConfiguration('gripper_exist')}
        ],
    )

    # Return the LaunchDescription object containing all the above elements
    return LaunchDescription([
        left_can_port_arg,
        right_can_port_arg,
        auto_enable_arg,
        display_xacro_launch,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        left_piper_ctrl_node,
        right_piper_ctrl_node
    ])
