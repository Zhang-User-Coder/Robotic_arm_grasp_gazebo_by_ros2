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

class SimpleMover(Node):
    def __init__(self):
        super().__init__('move_grasp_node')
        
        self.cb_group = ReentrantCallbackGroup()
        
        self.planning_group = 'rm_group'
        self.base_frame = 'base_link'
        self.end_effector_link = 'robotiq_85_base_link'
        
        self.joint_names = ['robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint']
        
        # 唯一的运动客户端 (无论是 OMPL 还是 Pilz，都通过这个 Action 发送)
        self.arm_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        
        # 夹爪客户端
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.cb_group)
        
    def run_task(self):
        self.get_logger().info("等待动作服务器连接...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info("服务器已连接，开始执行...")
        
        # --- 1. 定义预备点 ---
        target_pose1 = PoseStamped()
        target_pose1.header.stamp = self.get_clock().now().to_msg()
        target_pose1.header.frame_id = self.base_frame
        
        target_pose1.pose.position.x = 0.3055
        target_pose1.pose.position.y = -0.254
        # 预备高度: 物体(0.025) + 夹爪长(0.16) + 余量(0.10) = 0.285
        target_pose1.pose.position.z = 0.025 + 0.26 
        
        target_pose1.pose.orientation.x = 0.0
        target_pose1.pose.orientation.y = 1.0
        target_pose1.pose.orientation.z = 0.0
        target_pose1.pose.orientation.w = 0.0
        
        self.get_logger().info(f"1. 移动到预备点 (z={target_pose1.pose.position.z:.3f})...")
        
        # 使用 OMPL (RRT) 进行长距离移动
        if self.move_arm_ompl(target_pose1):
            self.get_logger().info("到达预备点")
            time.sleep(0.5)
            
            # --- 2. 定义抓取点 ---
            target_pose2 = copy.deepcopy(target_pose1)
            target_pose2.pose.position.z = 0.025 + 0.14 
            
            self.get_logger().info(f"2. 直线下降 (z={target_pose2.pose.position.z:.3f})...")
            
            # 关键：使用 Pilz (LIN) 进行直线移动
            if self.move_arm_pilz_lin(target_pose2):
                self.get_logger().info("直线下降成功")
                time.sleep(0.5)
            
                # --- 3. 闭合夹爪 ---
                self.control_gripper(position=0.42)
                self.get_logger().info("夹爪已闭合")
                time.sleep(1.5)
            else:
                self.get_logger().error("第二步:Pilz 直线规划失败 (可能不可达或碰撞)")
            
            target_pose2.pose.position.z = 0.3
            if self.move_arm_pilz_lin(target_pose2):
                self.get_logger().info("直线上升成功")
                time.sleep(0.5)
            else:
                self.get_logger().error("第二步:Pilz 直线规划失败 (可能不可达)")
                
            
            target_pose3 = copy.deepcopy(target_pose1)
            target_pose3.pose.position.x = 0.2
            target_pose3.pose.position.y = 0.2
            if self.move_arm_ompl(target_pose3):
                self.get_logger().info("直线转移成功")
                time.sleep(0.5)
            else:
                self.get_logger().error("第三步:转移失败")
            
            target_pose3.pose.position.z = 0.225
            if self.move_arm_pilz_lin(target_pose3):
                self.get_logger().info("直线下降成功")
                time.sleep(0.5)
                self.control_gripper(position=0.0)
                self.get_logger().info("夹爪张开，任务完成")
            else:
                self.get_logger().error("第四步:Pilz 直线规划失败 (可能不可达)")
             
            
        else:
            self.get_logger().error("第一步:OMPL 规划失败")
            
    def move_arm_ompl(self, target_pose):
        """
        使用默认的 OMPL (RRTConnect) 进行规划
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        
        # 指定 pipeline 和 planner
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.planner_id = "RRTstarkConfigDefault"
        
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        self.add_constraints(goal_msg, target_pose, tolerance=0.01) # 第一步精度要高，方便第二步接力
        
        return self.send_goal(goal_msg)

    def move_arm_pilz_lin(self, target_pose):
        """
        使用 Pilz (LIN) 进行直线规划
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        
        # 指定 Pilz 管线 和 LIN 算法
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
        goal_msg.request.planner_id = "LIN"
        
        goal_msg.request.allowed_planning_time = 1.0
        goal_msg.request.max_velocity_scaling_factor = 0.1 
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Pilz 非常严格，不需要太松的容差,容差1cm
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
        send_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"规划器 ({goal_msg.request.planner_id}) 拒绝了请求")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            return True
        else:
            self.get_logger().error(f"执行失败, 错误码: {result.result.error_code.val}")
            return False
    
    def control_gripper(self, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(seconds=1.0).to_msg()
        goal.trajectory.points.append(point)
        
        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMover()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.run_task()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()