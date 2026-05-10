import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
import time
import copy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

class AutoMoveGrasp(Node):
    def __init__(self):
        super().__init__('auto_move_grasp_node')
        
        self.cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        
        # --- 配置参数 ---
        self.planning_group = 'rm_group'
        self.base_frame = 'base_link'
        self.end_effector_link = 'robotiq_85_base_link'
        self.joint_names = ['robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint']
        
        # 抓取参数修正 (根据之前的经验微调)
        # Link6 到 夹爪指尖中心的物理距离约为 0.16m
        self.GRIPPER_OFFSET = 0.16 
        
        # --- 状态机标志 ---
        self.is_busy = False
        self.parking_slots = []
        self.init_parking_lots()
        
        #订阅目标坐标
        self.sub_target = self.create_subscription(
            PoseStamped, '/grasp_target', self.target_callback, 10, callback_group=self.cb_group)
            
        self.arm_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.cb_group)
        
        self.get_logger().info(">>> 全自动抓取系统就绪! 等待YOLO目标...")
        
        # 初始动作：先回到观察点
        threading.Thread(target=self.go_to_home).start()

    def init_parking_lots(self):
        """初始化放置区域 (2x3 网格)"""
        start_x = 0.0
        start_y = 0.45
        z_height = 0.05 # 放置平面的高度
        gap = 0.10      # 间距
        for row in range(2):
            for col in range(3):
                slot = {
                    'x': start_x + (row * gap),
                    'y': start_y + (col * gap),
                    'z': z_height,
                    'occupied': False,
                    'id': f"Slot-{row}-{col}"
                }
                self.parking_slots.append(slot)
                
    def get_next_empty_slot(self):
        for slot in self.parking_slots:
            if not slot['occupied']:
                return slot
        return None

    def target_callback(self, msg):
        """YOLO 目标回调函数"""
        # 如果正在忙，直接忽略新目标
        if self._lock.acquire(blocking=False):
            try:
                if self.is_busy:
                    return
                
                # 检查是否有空闲放置位
                target_slot = self.get_next_empty_slot()
                if target_slot is None:
                    self.get_logger().warn(">>> 放置区已满，任务停止！")
                    return

                self.is_busy = True
                self.get_logger().info(f">>> 收到新目标: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})")
                
                # 开启新线程执行任务，避免阻塞回调
                threading.Thread(target=self.execute_mission_thread, args=(msg, target_slot)).start()
                
            finally:
                self._lock.release()

    def execute_mission_thread(self, target_pose, target_slot):
        """ 完整的抓取-放置 状态机流程 """
        try:
            #确保夹爪张开
            self.get_logger().info("1. 张开夹爪")
            self.control_gripper(position=0.0)
            
            #预备抓取点 (物体上方 12cm)
            pre_grasp_pose = copy.deepcopy(target_pose)
            pre_grasp_pose.pose.position.z += self.GRIPPER_OFFSET + 0.12 
            
            #实际抓取点 (物体位置)
            grasp_pose = copy.deepcopy(target_pose)
            grasp_pose.pose.position.z += self.GRIPPER_OFFSET - 0.01
            

            #YOLO节点发来的通常已经是这个姿态，这里强制覆盖一下以防万一
            default_orientation = target_pose.pose.orientation # 使用YOLO发来的姿态
            pre_grasp_pose.pose.orientation = default_orientation
            grasp_pose.pose.orientation = default_orientation

            #快速移动到预备点
            self.get_logger().info("2. OMPL: 移动到预备点...")
            if not self.move_arm_ompl(pre_grasp_pose): raise Exception("移动到预备点失败")
            
            #直线下降抓取
            self.get_logger().info("3. Pilz: 直线下降...")
            if not self.move_arm_pilz_lin(grasp_pose): raise Exception("下探抓取失败")
            
            #闭合夹爪 
            self.get_logger().info("4. 闭合夹爪...")
            self.control_gripper(position=0.42)
            time.sleep(1.0) # 等待抓稳
            
            #Pilz LIN: 直线提起 (回到预备高度)
            self.get_logger().info("5. Pilz: 直线提起...")
            if not self.move_arm_pilz_lin(pre_grasp_pose): raise Exception("提起失败")
            
            #计算放置点
            drop_pre_pose = PoseStamped()
            drop_pre_pose.header.frame_id = self.base_frame
            drop_pre_pose.pose.orientation = default_orientation
            drop_pre_pose.pose.position.x = target_slot['x']
            drop_pre_pose.pose.position.y = target_slot['y']
            #放置预备高度：放置面 + 夹爪长 + 余量(12cm)
            drop_pre_pose.pose.position.z = target_slot['z'] + self.GRIPPER_OFFSET + 0.12
            
            drop_pose = copy.deepcopy(drop_pre_pose)
            #放置实际高度：放置面 + 夹爪长 + 微小余量(避免撞击)
            drop_pose.pose.position.z = target_slot['z'] + self.GRIPPER_OFFSET + 0.06
            
            self.get_logger().info(f"6. OMPL: 搬运至 {target_slot['id']} 上方")
            if not self.move_arm_ompl(drop_pre_pose): raise Exception("搬运移动失败")
            
            #Pilz LIN: 直线下放
            self.get_logger().info("7. Pilz: 缓慢下放")
            if not self.move_arm_pilz_lin(drop_pose): raise Exception("下放失败")
            
            #张开夹爪
            self.get_logger().info("8. 松开夹爪")
            self.control_gripper(position=0.0)
            time.sleep(0.5)
            
            #Pilz LIN: 直线撤离
            self.get_logger().info("9. Pilz: 撤离")
            self.move_arm_pilz_lin(drop_pre_pose)
            
            # 标记占位
            target_slot['occupied'] = True
            self.get_logger().info(f">>> {target_slot['id']} 放置完成")
            
            #OMPL: 回到观察点 (Home)
            self.go_to_home()
            
        except Exception as e:
            self.get_logger().error(f"!!! 任务执行中断: {e}")
            #出错后尝试复位，防止卡在半空
            self.control_gripper(position=0.0)
            self.go_to_home()
            
        finally:
            self.is_busy = False
            self.get_logger().info(">>> 系统空闲，等待下一个目标...")

    def go_to_home(self):
        """回到观察点"""
        self.get_logger().info("移动到观察点(Home)...")
        home_pose = PoseStamped()
        home_pose.header.frame_id = self.base_frame
        home_pose.pose.position.x = 0.25
        home_pose.pose.position.y = 0.0
        home_pose.pose.position.z = 0.35
        home_pose.pose.orientation.x = 1.0
        home_pose.pose.orientation.y = 0.0
        home_pose.pose.orientation.z = 0.0
        home_pose.pose.orientation.w = 0.0
        self.move_arm_ompl(home_pose)
    
    def move_arm_ompl(self, target_pose):
        """ OMPL: 适用于大范围移动 (Pre-Grasp, Drop, Home) """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.planner_id = "RRTstarkConfigDefault"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        self.add_constraints(goal_msg, target_pose, tolerance=0.01)
        return self.send_goal(goal_msg)

    def move_arm_pilz_lin(self, target_pose):
        """ Pilz LIN: 适用于直线操作 (Approach, Retreat) """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
        goal_msg.request.planner_id = "LIN"
        goal_msg.request.allowed_planning_time = 2.0
        goal_msg.request.max_velocity_scaling_factor = 0.2 # 直线操作慢一点更稳
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        self.add_constraints(goal_msg, target_pose, tolerance=0.01)
        return self.send_goal(goal_msg)

    def add_constraints(self, goal_msg, target_pose, tolerance=0.01):
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_link
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.SPHERE
        shape.dimensions = [tolerance]
        pos_constraint.constraint_region.primitives.append(shape)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0
        
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.base_frame
        ori_constraint.link_name = self.end_effector_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = tolerance
        ori_constraint.absolute_y_axis_tolerance = tolerance
        ori_constraint.absolute_z_axis_tolerance = tolerance
        ori_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)

    def send_goal(self, goal_msg):
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("MoveGroup Action Server 不在线")
            return False

        send_future = self.arm_client.send_goal_async(goal_msg)
        try:
            goal_handle = send_future.result(timeout=5.0)
        except Exception as e:
            self.get_logger().error(f"Goal发送失败: {e}")
            return False
        if goal_handle is None:
            self.get_logger().error(f"Action Server返回了None的goal_handle: {e}")
            return False
        
        if not goal_handle.accepted:
            self.get_logger().error(f"规划器 ({goal_msg.request.planner_id}) 拒绝了路径")
            return False
        
        result_future = goal_handle.get_result_async()
        try:
            result = result_future.result()
        except Exception as e:
            self.get_logger().error(f"执行过程异常: {e}")
            return False
            
        if result.result.error_code.val == 1:
            return True
        else:
            self.get_logger().error(f"执行失败, ErrorCode: {result.result.error_code.val}")
            return False
    
    def control_gripper(self, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(seconds=1.0).to_msg()
        goal.trajectory.points.append(point)
        
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            return
            
        send_future = self.gripper_client.send_goal_async(goal)
        try:
            goal_handle = send_future.result()
            if not goal_handle.accepted: return
            goal_handle.get_result_async().result()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AutoMoveGrasp()
    
    # 使用多线程执行器，确保 Action Client 和 Subscription 回调互不干扰
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
