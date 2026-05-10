import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
import time

class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
        ) 
        
        self.joint_names = ['robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint']
        
        if self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("控制器连接成功!准备开始测试...")
            self.run_test_loop()
            
        else:
            self.get_logger().error("连接失败！请检查：\n1. Gazebo 是否启动？\n2. ros2_controllers.yaml 是否加载成功？\n3. 控制器是否处于 active 状态？")
        
    def send_command(self,position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        
        point.positions = [position,position]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(seconds=1.0).to_msg()
        goal.trajectory.points.append(point)
        
        self.get_logger().info(f"发送指令:位置={position}")
        
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self,future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("指令被控制器拒绝")
            return
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self,result_future)
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info("动作执行完毕")
        else:
            self.get_logger().warn(f"动作结束(代码:{result.result.error_code})")
            
    def run_test_loop(self):
        try:
            while rclpy.ok():
                self.get_logger().info("测试半张开(0.4)")
                self.send_command(0.4)
                time.sleep(1.0)
                
                self.get_logger().info("测试闭合(0.8)")
                self.send_command(0.8)
                time.sleep(1.0)
                
                self.get_logger().info("测试全张开(0)")
                self.send_command(0.0)
                time.sleep(1.0)
                
                self.get_logger().info("--------------------------------------")
                
        except Exception as e:
            self.get_logger().error(f"测试错误:{e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = GripperTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()  