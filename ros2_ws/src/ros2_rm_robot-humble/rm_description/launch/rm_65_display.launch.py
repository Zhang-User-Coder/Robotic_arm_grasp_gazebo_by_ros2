import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

import xacro

def generate_launch_description():

    realman_xacro_file = os.path.join(get_package_share_directory('rm_description'), 'urdf',
                                        'rm_65.urdf.xacro')
    
    #为launch声明参数
    realman_xacro_file_declarement = DeclareLaunchArgument(
        name='model',default_value=str(realman_xacro_file),
        description='URDF的绝对路径')
    
    #获取文件内容生成新的参数
    robot_description = ParameterValue(
        Command(
            ['xacro ',LaunchConfiguration('model')]),
    value_type=str)

    return LaunchDescription([
            realman_xacro_file_declarement,
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                respawn=True,
                parameters=[{'robot_description': robot_description}],
                output='screen'
            ),
            
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                respawn=True,
                parameters=[{'robot_description': robot_description}],
                output='screen'
            )
        ])
