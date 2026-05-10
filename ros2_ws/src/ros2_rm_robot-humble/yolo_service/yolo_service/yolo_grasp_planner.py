import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped,PoseStamped
from ultralytics import YOLO

class YoloGraspPlanner(Node):
    def __init__(self):
        super().__init__('yolo_grasp_planner')
        
        self.bridge = CvBridge()
        
        #1.加载训练的模型
        self.model = YOLO("/home/zws/arm/ros2_ws/src/ros2_rm_robot-humble/yolo_service/yolo_models/train2/weights/last.pt")
        
        #2.订阅图像和相机信息
        self.sub_color = self.create_subscription(
            Image,'/camera_sensor/image_raw',self.color_callback,10
        )
        
        self.sub_depth = self.create_subscription(
            Image,'/camera_sensor/depth/image_raw',self.depth_callback,10
        )
        
        self.sub_info = self.create_subscription(
            CameraInfo,'/camera_sensor/camera_info',self.info_callback,10
        )
        
        # 发布目标坐标
        self.target_pub = self.create_publisher(PoseStamped,"/grasp_target",10)
        
        #3.TF变换监听器（用于把相机坐标转为机械臂基座坐标）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)
        
        #缓存数据
        self.latest_depth_img = None
        self.fx = None
        self.fy = None   #相机焦距参数
        self.cx = None
        self.cy = None   #相机光心坐标
        
        self.get_logger().info("YOLO Grasp Planner准备就绪!等待图像...")
        
    def info_callback(self,msg):
        #获取相机内参
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.get_logger().info(f"相机内参为:fx={self.fx:.2f},cx={self.cx:.2f}")
    
    def depth_callback(self,msg):
        try:
            #Gazebo的深度图通常是32FC1（米为单位）
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg,"32FC1")
        except Exception as e:
            pass
        
    def color_callback(self,msg):
        if self.latest_depth_img is None or self.fx is None:
            self.get_logger().info(f"等待数据: Depth={self.latest_depth_img is not None}, Info={self.fx is not None}")
            return
        
        try:
            #将ROS图像转换为OpenCV图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            
            #使用YOLO进行推理
            results = self.model(cv_image,verbose=False,device='cpu')
            
            #处理识别结果
            for result in results:
                #计数器
                num = 0
                #result.boxes包含了识别框的信息
                for box in result.boxes:
                    #获取类别id和置信度
                    class_id = int(box.cls[0])
                    conf = float(box.conf[0])
                
                    #获取边界框坐标
                    x1,y1,x2,y2 = map(int,box.xyxy[0])
                    
                    #过滤置信度低的
                    if conf < 0.25:
                        continue
                    
                    #获取类别名称
                    label = self.model.names[class_id]
                    
                    #在图上画框（调试用）
                    cv2.rectangle(cv_image,(x1,y1),(x2,y2),(255,0,0),2)
                    
                    #计算中心点和深度
                    center_x = int((x1+x2)/2)
                    center_y = int((y1+y2)/2)
                    
                    #边界保护，确保中心点坐标在深度图像范围内
                    if center_y >= self.latest_depth_img.shape[0] or center_x >=self.latest_depth_img.shape[1]:
                        continue
                    
                    #获取深度值（单位：m）
                    depth = self.latest_depth_img[center_y,center_x]
                    
                    #过滤无效深度（Gazebo里太远是inf，太近是nan）
                    if np.isnan(depth) or depth <= 0 or depth > 2.0:
                        continue
                    
                    #2D -> 3D（相机坐标系），基于针孔相机模型
                    X_c = (center_x - self.cx) * depth / self.fx
                    Y_c = (center_y - self.cy) * depth / self.fy
                    Z_c = depth
                    
                    #坐标转换（相机 -> 机械臂基座）
                    self.process_target(X_c,Y_c,Z_c,label)
            
            cv2.imshow("YOLO Grasp",cv_image)
            cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().info(f"处理出错:{e}")
    
    def process_target(self,x,y,z,label):
        #创建一个带坐标系的点
        point_camera = PointStamped()
        #必须和gazebo_65_description.urdf.xacro的<frameName>的名字一致
        point_camera.header.frame_id = "camera"
        point_camera.header.stamp = rclpy.time.Time().to_msg()
        point_camera.point.x = float(z)
        point_camera.point.y = float(-x)       
        point_camera.point.z = float(-y)
        
        try:
            #查询base_link 到 camera的变换
            #为避免阻塞，time_out为1.0s
            target_frame = "base_link"
            if self.tf_buffer.can_transform(target_frame,point_camera.header.frame_id,
            rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=1.0)):
                point_base = self.tf_buffer.transform(point_camera,target_frame)
                
                #过滤过大的高度
                z_val = point_base.point.z
                if z_val < 0.0 or z_val >0.06:
                    return
                    
                #过滤过近距离和过远距离的目标
                dist = np.sqrt(point_base.point.x**2 + point_base.point.y**2)
                if dist < 0.05 or dist > 0.65:
                    return
            
                #=======================
                #发送目标位置给抓取执行的节点
                target_pose = PoseStamped()
                target_pose.header.frame_id = "base_link"
                target_pose.header.stamp = self.get_clock().now().to_msg()
                
                #设置位置
                target_pose.pose.position = point_base.point
                
                #设置抓取姿态（垂直向下的四元数,绕x旋转180°）
                target_pose.pose.orientation.x = 0.7071
                target_pose.pose.orientation.y = -0.7071
                target_pose.pose.orientation.z = 0.0
                target_pose.pose.orientation.w = 0.0
                
                #将目标位置发布出去
                self.target_pub.publish(target_pose)
                
                #输出结果：这就是机械臂需要移动到的位置,相机坐标：目标相对摄像头光心的坐标，基座坐标：目标相对基座的坐标
                self.get_logger().info(
                    f"\n >>>发现目标[{label}]\n"
                    f"      相机坐标: x={x:.3f},y={y:.3f},z={z:.3f}\n"
                    f"      基座坐标: x={point_base.point.x:.3f},y={point_base.point.y:.3f},z={point_base.point.z:.3f}"
                )
            
            else:
                self.get_logger().warn(f"TF超时:无法从{point_camera.header.frame_id}转换到{target_frame}")
        
        except Exception as e:
            self.get_logger().error(f"TF变换异常:{e}")
                
def main(args=None):
    rclpy.init(args=args)  
    node = YoloGraspPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()       