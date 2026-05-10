import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

import xacro

from launch.actions import TimerAction,AppendEnvironmentVariable

from launch.launch_description_sources import PythonLaunchDescriptionSource

os.environ['GZ_SIM_RESOURCE_PATH'] = '/usr/share/gazebo-11:/usr/share/gazebo_models'
config_path = os.path.join(
    get_package_share_directory("rm_65_config"),
    "config",
    "ros2_controllers.yaml"
)
print(f"Config path:{config_path}")

def generate_launch_description():
    package_name = 'rm_gazebo'

    robot_name_in_model = 'rm_65_description'

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'config/gazebo_65_description.urdf.xacro')

    
    print("---", urdf_model_path)

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    print("urdf", doc.toxml())

    world = os.path.join(get_package_share_directory('rm_gazebo'),'world','sim_env.world')
    
    # 启动gazebo
    gazebo =  ExecuteProcess(
        cmd=['gazebo', '--verbose',world,'-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容
    # 并且会订阅 /joint_states 话题，获取关节的数据，然后发布tf和tf_static话题.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )
    
    #controller_manager = Node(
    #    package="controller_manager",
    #    executable="ros2_control_node",
    #    parameters=[
    #        params,
    #        os.path.join(get_package_share_directory("rm_65_config"),"config","ros2_controllers.yaml")
    #    ],
    #    output="screen",
    #)

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', f'{robot_name_in_model}',], 
                        output='screen')

    # gazebo在加载urdf时，根据urdf的设定，会启动一个joint_states节点?
    # 关节状态发布器
    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster","--controller-manager","/controller_manager"],
        output="screen",
    )

    # 路径执行控制器，也就是那个action？
    # 这个rm_group_controller需要根据urdf文件里面引用的ros2_controllers.yaml里面的名字确定
    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rm_group_controller","-c","/controller_manager"],
        output='screen'
    )
    
    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller","-c","/controller_manager"],
        output='screen'
    )
    
    # 用下面这两个估计是想控制好各个节点的启动顺序
    # 监听 spawn_entity_cmd，当其退出（完全启动）时，启动load_joint_state_controller
    #close_evt1 =  RegisterEventHandler( 
    #       event_handler=OnProcessExit(
    #            target_action=spawn_entity,
    #            on_exit=[controller_manager],
    #        )
    #)
    # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller
    # moveit是怎么和gazebo这里提供的action连接起来的
    #close_evt2 = RegisterEventHandler(
    #        event_handler=OnProcessExit(
    #            target_action=controller_manager,
    #            on_exit=[load_joint_state_controller],
    #        )
    #)
    
    #close_evt3 = RegisterEventHandler(
    #        event_handler=OnProcessExit(
    #            target_action=load_joint_state_controller,
    #            on_exit=[load_joint_trajectory_controller,load_gripper_controller],
    #        )
    #)
    #delayed_controller_manager = TimerAction(
    #    period=2.0,
    #    actions=[controller_manager]
    #)
    
    #delayed_joint_state_broadcaster = TimerAction(
    #    period=3.0,
    #    actions=[load_joint_state_controller],
    #)
    
    #delayed_joint_trajectory_controller = TimerAction(
    #    period=5.0,
    #    actions=[load_joint_trajectory_controller],
    #)
    
    #delayed_gripper_controller = TimerAction(
    #    period=5.0,
    #    actions=[load_gripper_controller],
    #)
    
    ld = LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_controller,
        load_joint_trajectory_controller,
        load_gripper_controller,
    ])

    return ld
