import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    use_sim_time=LaunchConfiguration('use_sim_time')
    pkg_path=os.path.join(get_package_share_directory ('chudovishe_description'))
    urdf_file=os.path.join(pkg_path, 'urdf', 'chudovishe_base.urdf')
    robot_description_confog=xacro.process_file(urdf_file).toxml()

    params = {'robot_description': robot_description_confog, 'use_sim_time': use_sim_time}
    node_joint_state_publisher=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    
    node_robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'),
            node_robot_state_publisher,
            node_joint_state_publisher
            ])

