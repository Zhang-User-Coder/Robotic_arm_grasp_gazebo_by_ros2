import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        #1.订阅Gazebo发布的RGB图像话题
        #注意：这里的话题必须和你rqt_image_view里看到的一样
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10)
        
        #2.初始化CV工具和YOLO模型
        self.bridge = CvBridge()
        
        self.model = YOLO("/home/zws/arm/ros2_ws/src/ros2_rm_robot-humble/yolo_service/yolo_models/train1/weights/last.pt")
        
        self.get_logger().info("YOLOv8检测节点已经准备就绪!")
    
    def image_callback(self,msg):
        try:
            #3.将ROS图像转换为OpenCV图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            
            #4.使用YOLO进行推理
            results = self.model(cv_image,verbose=False,device='cpu')
            
            #5.处理识别结果
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
                    
                    #获取类别名称
                    label = self.model.names[class_id]
                    
                    #在图上画框
                    cv2.rectangle(cv_image,(x1,y1),(x2,y2),(255,0,0),2)
                    cv2.putText(cv_image,f"{num}_{label}{conf:.2f}",(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                    #(x1,y1-10)对应要显示的文本的位置，cv2.FONT_HERSHEY_SIMPLEX是一个字体类型，0.5的意思是0.5倍原始字体大小，2是文本线条的粗细
                    
                    #抓取
                    #计算中心像素点坐标
                    center_x = int((x1+x2)/2)
                    center_y = int((y1+y2)/2)
                    
                    self.get_logger().info(f"{num}_{label}的中心坐标为{center_x}{center_y}")
                    num+=1
            
            #6.显示图像窗口
            cv2.imshow("YOLOv8 Inference",cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"图片处理错误:{str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()