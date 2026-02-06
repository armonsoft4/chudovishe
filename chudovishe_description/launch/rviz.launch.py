import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time=LaunchConfiguration('use_sim_time')
    pkg_path=os.path.join(get_package_share_directory ('chudovishe_description'))
    urdf_file=os.path.join(pkg_path, 'urdf', 'chudovishe_base.urdf.xacro')
    robot_description_config=xacro.process_file(urdf_file).toxml()

    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    robot_controllers = PathJoinSubstitution(
    [
        FindPackageShare("chudovishe_description"), "config", "controller_manager.yaml",
    ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, robot_controllers],
        output="both",
    )
    
    node_robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["chudovishe_base_controller", "-c", "/controller_manager"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'),
            control_node,
            node_robot_state_publisher,
            joint_state_broadcaster_spawner,
            robot_controller_spawner

            ])
            

