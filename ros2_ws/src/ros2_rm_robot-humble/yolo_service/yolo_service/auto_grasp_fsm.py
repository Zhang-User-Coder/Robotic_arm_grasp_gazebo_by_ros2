#----- cpu越强的，使用这套代码搬运的成功率越高 -----
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient   #调用动作服务器
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup  #Moveit移动动作
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from rclpy.executors import MultiThreadedExecutor   #多线程执行器
from rclpy.callback_groups import ReentrantCallbackGroup  #可重入回调组，运行多个回调同时执行，与MultiThreadExecutor搭配使用
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory  #夹爪轨迹控制动作
from trajectory_msgs.msg import JointTrajectoryPoint  #轨迹点
from rclpy.duration import Duration  #持续时间
import time
import threading
import copy
import math

class GraspStateMachine(Node):
    def __init__(self):
        super().__init__('grasp_fsm_node')
        
        self.cb_group = ReentrantCallbackGroup()
        self.success = False
        
        self.allow_detection = False
        self.filter_target_slot = None
        
        self.planning_group = 'rm_group'
        self.base_frame = 'base_link'
        self.end_effector_link = 'robotiq_85_base_link' # 机械臂末端法兰
        
        # 夹爪的长度偏移量 (Link6 到 指尖抓取点 的距离)
        self.GRIPPER_LENGTH = 0.137 
        
        self.parking_slots = []
        self.init_parking_lots()
        
        # 状态机控制变量
        self.latest_target_pose = None       #存储最新的目标
        self.new_target_received = False     #标志位
        self.stop_event = threading.Event()  #用于优雅退出
        
        self.sub_target = self.create_subscription(
            PoseStamped, '/grasp_target', self.target_callback, 10, callback_group=self.cb_group)
        self.arm_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.get_logger().info(">>>放置区准备就绪！空闲位置:6")
        
        # 启动独立的业务逻辑线程
        self.process_thread = threading.Thread(target=self.processing_loop)
        self.process_thread.start()
        
        #公用观察位置
        self.observe_pose_back = PoseStamped()
        self.observe_pose_back.header.frame_id = self.base_frame
        
    #初始化停车位
    def init_parking_lots(self):
        start_x = -0.2
        start_y = -0.2
        z_height = 0.05
        gap = 0.08
        for row in range(2):
            for col in range(3):
                slot = {
                    'x':start_x + (row*gap),
                    'y':start_y - (col*gap),
                    'z':z_height,
                    'occupied':False,
                    'id':f"Slot-{row}-{col}"
                }
                self.parking_slots.append(slot)
                self.get_logger().info(f"初始化放置位：{slot['id']}于{slot['x']:.3f},{slot['y']:.3f}")
                
    def get_next_empty_slot(self):
        for slot in self.parking_slots:
            if not slot['occupied']:
                return slot
        return None
    
    def target_callback(self, msg):
        if not self.allow_detection:
            return
        
        #防止放置点与预期车位距离过远，或者判定抓取失败
        if self.filter_target_slot is not None:
            dx = msg.pose.position.x - self.filter_target_slot['x']
            dy = msg.pose.position.y - self.filter_target_slot['y']
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist > 0.07:
                return
            else:
                self.latest_target_pose = None
        
        self.latest_target_pose = msg
        self.new_target_received = True
        
     # ---  独立的业务逻辑循环 ---
    def processing_loop(self):
        self.get_logger().info("业务处理线程已启动，等待目标...")
        
        while rclpy.ok() and not self.stop_event.is_set():
            #----------观察模式----------
            self.latest_target_pose = None
            self.new_target_received = False
            
            #2.打开视觉开关
            self.allow_detection = True
            wait_start = time.time()
            found_target = False
            while time.time() - wait_start < 10.0:
                if self.new_target_received and self.latest_target_pose is not None:
                    found_target = True
                    break
                time.sleep(0.05)
            
            self.allow_detection = False
            
            stop_time = time.time()
            if not found_target:
                self.get_logger().info("当前视野无目标")
                time.sleep(2.0)
                if time.time() - stop_time > 4.5:
                    home_pose = self.observe_pose(self.observe_pose_back)
                    if not self.move_arm_rrtconnect(home_pose): raise Exception("移动失败")
                continue
                
            #----------执行模式----------
            # 获取一个快照，防止处理过程中数据突变
            time.sleep(1)
            target_pose = copy.deepcopy(self.latest_target_pose)
                
            self.new_target_received = False # 重置标志位
                
            dist = math.sqrt(target_pose.pose.position.x**2 + target_pose.pose.position.y**2)
                
            if dist > 0.45:
                self.get_logger().warn(f"距离太远({dist:.2f}),放弃抓取")
                continue
                
            # 检查有没有空位
            target_slot = self.get_next_empty_slot()
            if target_slot is None:
                self.get_logger().warn(">>>放置区域已满，等待清理...")
                time.sleep(2.0)
                continue

            self.get_logger().info(f"锁定目标，开始执行任务 -> {target_slot['id']}")
                
            # 执行任务 (这里是独立线程，可以随便阻塞等待 result)
            self.execute_mission(target_pose, target_slot)
                
            if not self.success:
                self.get_logger().warn("任务失败，稍作休息再接收新目标,先回观察点")
                home_pose = self.observe_pose(self.observe_pose_back)
                if not self.move_arm_rrtconnect(home_pose): raise Exception("移动失败") # 失败了赶紧回，别疯狂抽搐
                time.sleep(1)
            else:
                self.get_logger().info("任务完成，准备下一个")
    
    def execute_mission(self, target_pose, target_slot):
        try:
            self.get_logger().info(f"分配放置位置：{target_slot['id']}")
            if not self.arm_client.server_is_ready():
                self.get_logger().info("等待MoveGroup服务器...")
                self.arm_client.wait_for_server(timeout_sec=1.5)
            
            if not self.gripper_client.server_is_ready():
                self.get_logger().info("等待MoveGroup服务器...")
                self.gripper_client.wait_for_server(timeout_sec=1.5)
            
            #home_pose = self.observe_pose(target_pose)
            #if not self.move_arm_ompl(home_pose): raise Exception("复位失败")
            
            # 1. 张开夹爪
            self.control_gripper(position=0.0)
            
            # 目标Z (物体中心) + 夹爪长度
            pre_grasp_z_offset = self.GRIPPER_LENGTH + 0.12
            
            # 目标Z + 夹爪长度 (正好抓取)
            grasp_z_offset = self.GRIPPER_LENGTH
            
            # 2. 移动到预抓取点
            prep_grasp = self.offset_pose(target_pose, z_offset=pre_grasp_z_offset)
            self.get_logger().info(f"移动到预抓取位置{target_pose.pose.position.x:.3f} {target_pose.pose.position.y:.3f} {(target_pose.pose.position.z+pre_grasp_z_offset):.3f}")
            if not self.move_arm_rrtconnect(prep_grasp): raise Exception("移动失败")
            
            # 3. 下探抓取
            grasp_pose = self.offset_pose(target_pose, z_offset=grasp_z_offset)
            self.get_logger().info(f"下探抓取,下探位置为{(target_pose.pose.position.z+grasp_z_offset):.3f}")
            if not self.move_arm_pilz_lin(grasp_pose): raise Exception("下探失败")
        
            # 4. 闭合夹爪
            self.get_logger().info("闭合夹爪")
            self.control_gripper(position=0.42)
            time.sleep(1.0)
            
            # 5. 抬起
            self.get_logger().info("抬起...")
            if not self.move_arm_pilz_lin(prep_grasp): raise Exception("抬起失败")
            
            # 6. 搬运逻辑
            drop_pose = PoseStamped()
            drop_pose.header.frame_id = self.base_frame
            drop_pose.pose.position.x = target_slot['x']
            drop_pose.pose.position.y = target_slot['y']
            # 放置时也要加上夹爪长度，否则把方块按进土里
            drop_pose.pose.position.z = 0.3 
            
            drop_pose.pose.orientation.x = 1.0
            drop_pose.pose.orientation.y = 0.0
            drop_pose.pose.orientation.z = 0.0
            drop_pose.pose.orientation.w = 0.0
            
            self.get_logger().info(f"搬到{target_slot['id']}上方")
            if not self.move_arm_rrtstar(drop_pose): raise Exception("搬运失败")
            
            # 7. 下放
            final_drop = self.offset_pose(drop_pose, z_offset=-0.07)
            self.get_logger().info("缓慢下放")
            self.move_arm_pilz_lin(final_drop)
            
            # 8. 松开
            self.get_logger().info("松开夹爪")
            self.control_gripper(position=0.0)
            time.sleep(0.5)
            
            # 9. 上升离开         
            #if not self.move_arm_pilz_lin(drop_pose): raise Exception("离开失败")
            #target_slot["occupied"] = True
            #self.get_logger().info(f">>>{target_slot['id']}已被占用")
            
            #----- 这里是9.的另一套平替代码，用来检测是否成功搬运至车位,仅供参考 -----
            drop_pose.pose.position.x = 0.05
            drop_pose.pose.position.y = -0.28
            drop_pose.pose.orientation.x = 0.7071
            drop_pose.pose.orientation.y = 0.7071
            
            if not self.move_arm_rrtconnect(drop_pose): raise Exception("离开失败")
            self.get_logger().info("离开放置区")
            time.sleep(3)
            self.filter_target_slot = target_slot
            self.allow_detection = True
            self.new_target_received = False
            self.latest_target_pose = None
            found_target = False
            wait_start = time.time()
            while time.time() - wait_start < 3.0:
                if self.new_target_received and self.latest_target_pose is not None:
                    dx = self.latest_target_pose.pose.position.x - target_slot['x']
                    dy = self.latest_target_pose.pose.position.y - target_slot['y']
                    dist = math.sqrt(dx**2 + dy**2)
                    
                    self.get_logger().info(f"检测到物体,距离车位中心偏差:{dist}")
            
                    if dist < 0.07:
                        found_target = True
                        break
                    else:
                        self.new_target_received = False
                    
                time.sleep(0.05)
            
            self.allow_detection = False
            self.filter_target_slot = None
            
            if found_target:
                #标记该车位被占用
                target_slot["occupied"] = True
                self.get_logger().info(f">>>{target_slot['id']}已被占用")
            else:
                target_slot["occupied"] = False
                self.get_logger().warn("机械臂没夹住目标，重新再试")
            #---------------------------------------------------------------------------------
            
            # 10. 复位 (回到观察点)
            self.get_logger().info("复位中...")
            home_pose = self.observe_pose(drop_pose)
            if not self.move_arm_rrtconnect(home_pose): raise Exception("复位失败")
            self.success = True
            time.sleep(3)    #这行代码建议别删，用来度过视觉噪声大的时间段
            
        except Exception as e:
            self.success = False
            self.get_logger().error(f"任务异常: {e}")
        
    def offset_pose(self, origin_pose, z_offset):
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.base_frame
        new_pose.header.stamp = self.get_clock().now().to_msg()
        new_pose.pose.position.x = origin_pose.pose.position.x
        new_pose.pose.position.y = origin_pose.pose.position.y
        new_pose.pose.position.z = origin_pose.pose.position.z + z_offset
        new_pose.pose.orientation = origin_pose.pose.orientation
        return new_pose
    
    #观察点
    def observe_pose(self,origin_pose):
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.base_frame
        new_pose.pose.position.x = 0.15
        new_pose.pose.position.y = 0.0
        new_pose.pose.position.z = 0.4
        new_pose.pose.orientation.x = 0.7071
        new_pose.pose.orientation.y = -0.7071
        new_pose.pose.orientation.z = 0.0
        new_pose.pose.orientation.w = 0.0
        
        return new_pose
    
    #夹爪控制
    def control_gripper(self, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint']
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.velocities = [0.0, 0.0]
        if position == 0.0:
            point.time_from_start = Duration(seconds=1.0).to_msg()  #在1秒左右完成打开动作
        else:
            point.time_from_start = Duration(seconds=2.0).to_msg()  #在3秒左右完成闭合动作
        goal.trajectory.points.append(point)
        
        future = self.gripper_client.send_goal_async(goal)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 5.0:
                self.get_logger().warn("夹爪指令超时")
                return False
            time.sleep(0.05)
        
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("夹爪拒绝了请求")
                return False
            
            res_future = goal_handle.get_result_async()
            while not res_future.done():
                time.sleep(0.05)
            return True

        except Exception as e:
            self.get_logger().error(f"夹爪控制异常:{e}")
            return False
    
    #三种机械臂运行模式
    #搬运模式（这里考验cpu性能）
    def move_arm_rrtstar(self, target_pose):
        return self.send_move_goal(target_pose, planner="rrtstar")

    #直线模式（这里考验cpu性能）
    def move_arm_pilz_lin(self, target_pose):
        return self.send_move_goal(target_pose, planner="pilz")
    
    #未处于抓取状态时，快速运行模式
    def move_arm_rrtconnect(self, target_pose):
        return self.send_move_goal(target_pose, planner="rrtconnect")

    def send_move_goal(self, target_pose, planner):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 3.0  #设置成10.0，也可以完成整个流程，这里必须是浮点数
        
        if planner == "rrtstar":
            goal_msg.request.pipeline_id = "ompl"
            goal_msg.request.planner_id = "RRTstarkConfigDefault"
            goal_msg.request.max_velocity_scaling_factor = 0.5
            goal_msg.request.max_acceleration_scaling_factor = 0.5
            tol = 0.05  #姿态容差0.05弧度
        elif planner == "pilz":
            goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
            goal_msg.request.planner_id = "LIN"
            goal_msg.request.max_velocity_scaling_factor = 0.25 # 直线慢一点
            goal_msg.request.max_acceleration_scaling_factor = 0.25
            tol = 0.01
        elif planner == "rrtconnect":
            goal_msg.request.pipeline_id = "ompl"
            goal_msg.request.planner_id = "RRTConnectkConfigDefault"
            goal_msg.request.max_velocity_scaling_factor = 1.0
            goal_msg.request.max_acceleration_scaling_factor = 1.0
            tol = 0.02

        self.add_constraints(goal_msg, target_pose, tolerance=tol)
        
        # 发送目标
        send_future = self.arm_client.send_goal_async(goal_msg)
        
        # 等待 Server 接受
        start_time = time.time()
        while not send_future.done():
            if time.time() - start_time > 15.0:
                self.get_logger().error("Moveit 没反应(Send Goal Timeout)")
                return False
            time.sleep(0.01)
            
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"MoveIt 拒绝了请求: {planner}")
            return False
        
        # 等待执行结果
        res_future = goal_handle.get_result_async()
        start_time = time.time()
        while not res_future.done():
            if time.time() - start_time > 60.0:
                self.get_logger().error("Moveit 执行超时(Excution Timeout)")
                return False
            time.sleep(0.05)
            
        result = res_future.result()
        
        #检查最终状态
        if result.result.error_code.val == 1:
            return True
        else:
            self.get_logger().error(f"MoveIt 报错: Code {result.result.error_code.val}")
            return False
        
    def add_constraints(self, goal_msg, target_pose, tolerance=0.01):
        #位置约束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_link
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.SPHERE     #定义约束是什么类型
        shape.dimensions = [0.01] 
        pos_constraint.constraint_region.primitives.append(shape)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0
        
        #姿态约束
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
    
def main(args=None):
    rclpy.init(args=args)
    node = GraspStateMachine()
    executor = MultiThreadedExecutor(num_threads=4)  #主要处理三个回调:target_callback
    executor.add_node(node)                          #arm_client的send_goal_async返回的future的回调
    try:                                             #gripper_client的send_goal_async返回的future的回调
        executor.spin()                              #还处理arm_client和gripper_client的get_result_async返回的future的回调和其他的回调
    except:                                          #所以线程数是四个
        node.get_logger().info("正在退出...")
        node.stop_event.set()
        node.process_thread.join()
    finally:
        node.destroy_node()
        rclpy.shutdown()
