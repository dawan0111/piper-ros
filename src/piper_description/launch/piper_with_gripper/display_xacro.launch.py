from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('piper_description')
    default_model_path = urdf_tutorial_path / 'urdf/piper_description.xacro'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/piper_ctrl.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    left_robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    right_robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model'), " robot_namespace:=right_arm spawn_world_link:=true"]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': left_robot_description}],
        namespace='left_arm',
        # remappings=[
        #     ('/joint_states', 'left_arm/joint_states'),
        #     ('/robot_description', 'left_arm/robot_description'),
        # ]
    )

    robot_state_publisher_node2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node2',
        parameters=[{'robot_description': right_robot_description}],
        namespace='right_arm',
        # remappings=[
        #     ('/joint_states', 'right_arm/joint_states'),
        #     ('/robot_description', 'right_arm/robot_description'),
        # ]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    node1 = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "world", "left_arm_base_link"])

    node2 = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "world", "right_arm_base_link"])
    
    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        # joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        robot_state_publisher_node2,
        rviz_node,

        node1,
        node2
    ])