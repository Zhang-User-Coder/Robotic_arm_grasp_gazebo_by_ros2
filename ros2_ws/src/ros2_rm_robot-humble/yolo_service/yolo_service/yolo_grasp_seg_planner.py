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
from scipy.spatial.transform import Rotation as R #用来计算旋转
import math

class YoloGraspPlanner(Node):
    def __init__(self):
        super().__init__('yolo_grasp_planner')
        
        self.bridge = CvBridge()
        
        #1.加载训练的模型(实例分割)
        self.model = YOLO("/home/zws/arm/ros2_ws/src/ros2_rm_robot-humble/yolo_service/yolo_models/train3/weights/best.pt")
        
        #2.订阅图像和相机信息
        self.sub_color = self.create_subscription(                          #订阅彩色图像话题
            Image,'/camera_sensor/image_raw',self.color_callback,10
        )
        
        self.sub_depth = self.create_subscription(                          #订阅深度图像话题
            Image,'/camera_sensor/depth/image_raw',self.depth_callback,10
        )
        
        self.sub_info = self.create_subscription(                           #订阅相机信息话题
            CameraInfo,'/camera_sensor/camera_info',self.info_callback,10
        )
        
        # 发布目标坐标
        self.target_pub = self.create_publisher(PoseStamped,"/grasp_target",10)
        
        #3.TF变换监听器（用于把相机坐标转为机械臂基座坐标）
        self.tf_buffer = tf2_ros.Buffer()                                  #缓存，存储TF变换关系
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)  #监听TF广播，把变换数据存入tf_buffer
        
        #缓存数据
        self.latest_depth_img = None
        self.fx = None
        self.fy = None   #相机焦距参数
        self.cx = None
        self.cy = None   #相机光心坐标
        
        self.get_logger().info("YOLO Grasp Seg Planner准备就绪!等待图像...")
        
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
        #检查深度图和相机内参是否准备就绪
        if self.latest_depth_img is None or self.fx is None:
            self.get_logger().info(f"等待数据: Depth={self.latest_depth_img is not None}, Info={self.fx is not None}")
            return
        
        try:
            #将ROS图像转换为OpenCV图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            
            #使用YOLO进行推理
            results = self.model(cv_image,verbose=False,device='cuda',retina_masks=True)
            
            #处理识别结果
            for result in results:
                #如果没有检测到mask，即没有检测到物体，直接跳过
                if result.masks is None:
                    continue
                
                #遍历每一个检测到的物体（xy属性包括了轮廓坐标）
                for i,contour in enumerate(result.masks.xy):
                    #1.获取轮廓并计算质心
                    if len(contour) < 3: continue   #轮廓点数小于3，直接跳过
                    
                    contour = contour.astype(np.int32)  #将轮廓坐标转化为整数
                    
                    #计算矩
                    M = cv2.moments(contour)
                    if M['m00'] == 0: continue    #m00是轮廓的面积
                    center_x = int(M['m10'] / M['m00'])  #m10和m01是x和y方向的一阶矩
                    center_y = int(M['m01'] / M['m00'])
                    
                    #2.边界保护，确保中心点坐标在深度图像范围内
                    if center_y >= self.latest_depth_img.shape[0] or center_x >=self.latest_depth_img.shape[1]:
                        continue
                    
                    #获取深度值（单位：m）
                    depth = self.latest_depth_img[center_y,center_x]
                    
                    #过滤无效深度（Gazebo里太远是inf，太近是nan）
                    if np.isnan(depth) or depth <= 0 or depth > 2.0:
                        continue
                    
                    #获取类别标签
                    if result.boxes is not None and len(result.boxes) > i:
                        class_id = int(result.boxes[i].cls[0])
                        label = self.model.names[class_id]
                    else:
                        label = "obj"
                    
                    #忽略错误识别出的夹爪
                    if class_id == 1:
                        continue
                    
                    #3.计算外接矩形，找最长真边
                    rect = cv2.minAreaRect(contour)
                    ##rect = ((cx,cy),(width,height),angle)
                    
                    box_points = cv2.boxPoints(rect)   #从矩形参数得到四个点的坐标
                    box_points = np.int0(box_points)   #np.int0取整
                    
                    #以下是另一套抓取角度的计算,没用到深度，抓取的姿态是失败的:
                    #获取角度和尺寸
                    #angle = rect[2]
                    #width,height = rect[1]
                    
                    #我希望夹爪长边对齐物体的左边
                    #if width < height:
                    #    angle = angle - 90
                        
                    #if angle < 0:
                    #    angle += 180
                    
                    #angle = angle % 180
                    
                    #转为弧度
                    #angle_rad = math.radians(angle)
                    
                    epsilon = 0.02*cv2.arcLength(contour,True)          #对轮廓进行多边形近似，arcLength计算轮廓周长，单位算像素，最大误差是2%，
                    approx = cv2.approxPolyDP(contour,epsilon,True)     #数值越小越逼近原始轮廓，得到被近似后的多边形顶点，eplsilon是距离阈值，
                    if len(approx) < 3: continue                        #contour--原始轮廓点集，找原始轮廓各线段距离最远的两个点，如果两点间距离
                                                                        #小于epsilon,该线段保留
                    max_len = 0
                    margin = 10   #预设像素点数量为10
                    img_h,img_w = cv_image.shape[:2]
                    best_pt_a = None
                    best_pt_b = None
                    for j in range(len(approx)):         #遍历多边形的每两个相邻顶点，也就是一条边
                        pt1 = approx[j][0]                     #pt1和pt2是当前这条边的两端点
                        pt2 = approx[(j+1) % len(approx)][0]
                        on_left = (pt1[0] <= margin and pt2[0] <= margin)                    #检查这些边是否在图像边缘，是的话，认为这条边由
                        on_right = (pt1[0] >= img_w - margin and pt2[0] >= img_w - margin)   #图像边缘造成，直接忽略跳过
                        on_top = (pt1[1] <= margin and pt2[1] <= margin)
                        on_bottom = (pt1[1] >= img_h - margin and pt2[1] >= img_h - margin)
                        
                        if on_left or on_right or on_top or on_bottom:
                            continue
                        
                        length = math.hypot(pt2[0] - pt1[0],pt2[1] - pt1[1])   #找到最长真边
                        if length > max_len:
                            max_len = length
                            best_pt_a = pt1
                            best_pt_b = pt2
                    
                    #4.2D -> 3D（相机坐标系），基于针孔相机模型
                    #通过端点和中心点找到这条边的安全点，避免边缘深度噪声
                    safe_a_x = int(best_pt_a[0]*0.9 + center_x*0.1)
                    safe_a_y = int(best_pt_a[1]*0.9 + center_y*0.1)
                    safe_b_x = int(best_pt_b[0]*0.9 + center_x*0.1)
                    safe_b_y = int(best_pt_b[1]*0.9 + center_y*0.1)
                    
                    #获取这两个安全点的深度值，过滤无效深度
                    z_a = self.latest_depth_img[safe_a_y,safe_a_x]
                    z_b = self.latest_depth_img[safe_b_y,safe_b_x]
                    if np.isnan(z_a) or z_a <= 0 or np.isnan(z_b) or z_b <= 0:
                        continue
                    
                    #利用针孔模型，获得相机坐标系下的安全点和中心点坐标
                    X_a = (safe_a_x - self.cx) * z_a /self.fx
                    Y_a = (safe_a_y - self.cy) * z_a /self.fy
                    Z_a = z_a
                    
                    X_b = (safe_b_x - self.cx) * z_b /self.fx
                    Y_b = (safe_b_y - self.cy) * z_b /self.fy
                    Z_b = z_b
                    
                    X_c = (center_x - self.cx) * depth / self.fx
                    Y_c = (center_y - self.cy) * depth / self.fy
                    Z_c = depth
                    
                    
                    #用紫色线画出最长真边
                    cv2.line(cv_image,tuple(best_pt_a),tuple(best_pt_b),(255,0,255),2)
                    
                    #5.绘制轮廓
                    #画轮廓
                    cv2.drawContours(cv_image,[contour],-1,(0,255,0),2)        #绘制列表中所有轮廓
                    #画外接矩形
                    cv2.drawContours(cv_image,[box_points],0,(0,0,255),1)      #0：只绘制列表中第一个轮廓
                    #画质心
                    cv2.circle(cv_image,(center_x,center_y),5,(255,0,0),-1)    #5是半径，5个像素大小
                    
                    #坐标转换（相机 -> 机械臂基座）
                    self.process_target(X_c,Y_c,Z_c,X_a,Y_a,Z_a,X_b,Y_b,Z_b,label)
            
            cv2.imshow("YOLO Grasp",cv_image)
            cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f"处理出错:{e}")
    
    def process_target(self,x_c,y_c,z_c,x_a,y_a,z_a,x_b,y_b,z_b,label):
        def get_base_point(cx,cy,cz):
            #创建一个带坐标系的点
            point_camera = PointStamped()
            #必须和gazebo_65_description.urdf.xacro的<frameName>的名字一致
            point_camera.header.frame_id = "camera"
            point_camera.header.stamp = rclpy.time.Time().to_msg()
            point_camera.point.x = float(cz)
            point_camera.point.y = float(-cx)       
            point_camera.point.z = float(-cy)
            if self.tf_buffer.can_transform(target_frame,"camera",
            rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.5)):
                return self.tf_buffer.transform(point_camera,target_frame).point
            return None
        
        try:
            #查询base_link 到 camera的变换
            #为避免阻塞，time_out为1.0s
            target_frame = "base_link"
            base_c = get_base_point(x_c,y_c,z_c)
            base_a = get_base_point(x_a,y_a,z_a)
            base_b = get_base_point(x_b,y_b,z_b)
            
            if not base_c or not base_a or not base_b:
                return
                
            #过滤过大的高度
            z_val = base_c.z
            if z_val < 0.0 or z_val > 0.06:
                return
                    
            #过滤过近距离和过远距离的目标
            dist = np.sqrt(base_c.x**2 + base_c.y**2)
            if dist < 0.05 or dist > 0.65:
                return
            
            dx = base_b.x - base_a.x
            dy = base_b.y - base_a.y
            angle_rad = math.atan2(dy,dx)    #获得该向量从x正轴逆时针旋转的角度
            
            if angle_rad > math.pi / 2:
                angle_rad -= math.pi
            elif angle_rad < -math.pi / 2:
                angle_rad += math.pi
            
            #=======================
            #发送目标位置给抓取执行的节点
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.header.stamp = self.get_clock().now().to_msg()
                
            #设置位置
            target_pose.pose.position = base_c
                
            #1.基础姿态：垂直向下
            r_base = R.from_quat([0.7071,-0.7071,0.0,0.0])
            #2.叠加识别的旋转（绕Z轴旋转angle_rad）
            r_rot = R.from_euler('z',angle_rad)
            #3.组合旋转
            r_final = r_rot * r_base
            q_final = r_final.as_quat()
                
            target_pose.pose.orientation.x = float(q_final[0])
            target_pose.pose.orientation.y = float(q_final[1])
            target_pose.pose.orientation.z = float(q_final[2])
            target_pose.pose.orientation.w = float(q_final[3])
                
            #将目标位置发布出去
            self.target_pub.publish(target_pose)
                
            #输出结果：这就是机械臂需要移动到的位置,相机坐标：目标相对摄像头光心的坐标，基座坐标：目标相对基座的坐标
            self.get_logger().info(
                f"\n >>>发现目标[{label}]\n"
                f"      基座坐标: x={base_c.x:.3f},y={base_c.y:.3f},z={base_c.z:.3f}"
                )
        
        except Exception as e:
            self.get_logger().error(f"TF变换异常:{e}")
                
def main(args=None):
    rclpy.init(args=args)  
    node = YoloGraspPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()       