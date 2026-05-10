import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: 
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: 
        return None

def generate_launch_description():
    MOVEIT_CONFIG_PKG = "rm_65_config" 
    URDF_PKG = "rm_gazebo" 
    URDF_FILE = "config/gazebo_65_description.urdf.xacro" 
    SRDF_FILE = "config/rm_65_description.srdf"
    
    # URDF
    xacro_file = os.path.join(get_package_share_directory(URDF_PKG), URDF_FILE)
    doc = xacro.process_file(xacro_file)
    robot_description_content = doc.toxml()
    robot_description = {'robot_description': robot_description_content}

    # SRDF
    robot_description_semantic_config = load_file(MOVEIT_CONFIG_PKG, SRDF_FILE)
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics
    kinematics_yaml = load_yaml(MOVEIT_CONFIG_PKG, 'config/kinematics.yaml')

    # Controllers
    moveit_controllers = load_yaml(MOVEIT_CONFIG_PKG, 'config/moveit_controllers.yaml')
    
    # 创建一个总的字典，用于存放所有要传给 MoveGroup 的一般参数
    planning_limits = {}

    # A. 加载并合并关节限制
    joint_limits_yaml = load_yaml(MOVEIT_CONFIG_PKG, 'config/joint_limits.yaml')
    if joint_limits_yaml:
        planning_limits.update(joint_limits_yaml)
    
    # B. 加载并合并笛卡尔限制 (Pilz 必须项)
    cartesian_limits_yaml = load_yaml(MOVEIT_CONFIG_PKG, 'config/pilz_cartesian_limits.yaml')
    if cartesian_limits_yaml:
        planning_limits.update(cartesian_limits_yaml)
    else:
        print("未找到pilz_cartesian_limits.yaml")
    
    # C. 打包成Moveit需要的参数
    robot_description_planning = {'robot_description_planning': planning_limits}
    general_parameters = {'use_sim_time': True}

    # ========================================================================
    # 配置多规划器 (OMPL + Pilz)
    # ========================================================================
    
    # 1. OMPL 配置
    ompl_config = load_yaml(MOVEIT_CONFIG_PKG, 'config/ompl_planning.yaml')
    ompl_planning_pipeline = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        'start_state_max_bounds_error': 0.1,
    }
    if ompl_config:
        ompl_planning_pipeline.update(ompl_config)

    # 2. Pilz 配置
    pilz_planning_pipeline = {
        'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
        'request_adapters': """default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        'default_planner_config': 'PTP',
        'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService'
    }

    # 3. 组合 MoveGroup Capabilities 参数
    move_group_capabilities = {
        "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_pipeline,
        "pilz_industrial_motion_planner": pilz_planning_pipeline,
    }

    # ================= 构造节点 =================

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            robot_description_planning,
            general_parameters,
            move_group_capabilities,
            moveit_controllers,
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'moveit_simple_controller_manager': moveit_controllers}
        ],
    )

    # RViz 节点
    rviz_config_file = os.path.join(get_package_share_directory(MOVEIT_CONFIG_PKG), 'config', 'moveit.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            robot_description_planning,
            general_parameters,
        ],
    )

    # 静态 TF 发布
    virtual_joint_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='virtual_joint_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )

    return LaunchDescription([
        virtual_joint_broadcaster,
        run_move_group_node,
        rviz_node
    ])