import rclpy
from rclpy.node import Node
import time
from rm_ros_interfaces.msg import Movejp
from std_msgs.msg import Bool
import threading
import queue
import sys
import select

class MoveJPDemo(Node):
    def __init__(self):
        super().__init__("test_move_node")
        
        #创建发布者和订阅器
        self.test_publisher = self.create_publisher(Movejp,"/rm_driver/movej_p_cmd",10)
        self.test_subscriber = self.create_subscription(Bool,"/rm_driver/movej_p_result",self.movejp_callback,10)
        
        #创建命令的队列
        self.parameters_queue = queue.Queue()
        while self.test_publisher.get_subscription_count() == 0:
            time.sleep(0.1)
        self.get_logger().info("机械臂驱动已连接")
        self.is_moving = False
        self.command_num = 0
        self.current_command= None
        
        
    def movejp_callback(self,msg):
        """结果回调函数"""
        if msg.data:
            self.get_logger().info("运动成功")
        else:
            self.get_logger().info("运动失败")
        self.command_num -= 1
        self.get_logger().info(f"剩余指令数:{self.command_num}")
        self.is_moving = False
        if self.command_num > 0:
            self.execute_next_move()
        
    def execute_next_move(self):
        """执行下一个动作"""
        try:
            if not self.parameters_queue.empty():
                parameters = self.parameters_queue.get_nowait()
                self.movejp_demo(parameters)
        except queue.Empty:
            pass
        except Exception as e:
            self.get_logger().error(f"执行命令错误:{str(e)}")   
            
    def movejp_demo(self,parameters):
        """movejp运动规划函数"""
        movej_p_target_pose = Movejp()
        #设置位置
        movej_p_target_pose.pose.position.x = parameters[0]
        movej_p_target_pose.pose.position.y = parameters[1]
        movej_p_target_pose.pose.position.z = parameters[2]
        #设置方向(四元数)
        movej_p_target_pose.pose.orientation.x = parameters[3]
        movej_p_target_pose.pose.orientation.y = parameters[4]
        movej_p_target_pose.pose.orientation.z = parameters[5]
        movej_p_target_pose.pose.orientation.w = parameters[6]
        #设置其他参数
        movej_p_target_pose.speed = 20
        movej_p_target_pose.trajectory_connect = 0
        movej_p_target_pose.block = True
        #发布消息
        self.test_publisher.publish(movej_p_target_pose)
        self.get_logger().info('运动规划消息已发送')
                    
        self.is_moving = True
        self.current_command = parameters
                   
        
    
    #存取并使用指令
    def get_queue(self,parameters):
        self.parameters_queue.put(parameters)
        self.command_num += 1
        self.get_logger().info(f"{self.command_num}")
        if not self.is_moving:
            self.execute_next_move()
        else:
            self.get_logger().info("机械臂正在运动中，稍后处理指令")

def spin_thread(node):
    """在单独线程中持续处理ROS2回调"""
    rclpy.spin(node)

def main(args = None):
    rclpy.init(args = args)
    test_move = MoveJPDemo()
    
    #创建并启动ROS2回调处理线程，设置为守护线程，主程序退出时自动结束
    spin_thread_obj = threading.Thread(target=spin_thread, args=(test_move,),daemon=True)
    spin_thread_obj.start()
    
    try:
         while rclpy.ok():
            if select.select([sys.stdin],[],[],0.1)[0]:
                position_input_ = sys.stdin.readline().strip()
                position_input = position_input_.split(" ")
                #清空空字符串
                for i in position_input:
                    if i == "":
                        position_input.remove("")
                
                if position_input_ == "exit":
                    parameters = [0.0,0.0,0.85,0.0,0.0,0.0,1.0]
                    test_move.get_queue(parameters)
                    while test_move.command_num > 0:
                        time.sleep(0.1)
                    break
            
                if not all(s.replace(".","",1).replace('-','',1).isdigit() for s in position_input):
                    test_move.get_logger().info("请确保输入的参数全部为数字")
                    continue    
                if len(position_input) != 7:
                    test_move.get_logger().info("请输入七个数字")
                    continue
                parameters = [float(i) for i in position_input]
                test_move.get_queue(parameters)
                
    except KeyboardInterrupt:
        pass
    finally:
        test_move.parameters_queue.put(None)
        test_move.destroy_node()
        rclpy.shutdown()   