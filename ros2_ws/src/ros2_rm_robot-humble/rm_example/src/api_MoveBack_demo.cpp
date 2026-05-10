#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/moveback.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/****************************************创建类************************************/ 
class MoveBackDemo: public rclcpp::Node
{
  public:
    MoveBackDemo();                                                                                 //构造函数
    void moveback_demo();                                                                           //movejp运动规划函数
    void MoveBackDemo_Callback(const std_msgs::msg::Bool & msg);                                    //结果回调函数
  
  private:
    rclcpp::Publisher<rm_ros_interfaces::msg::Moveback>::SharedPtr moveback_publisher_;               //声明发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr moveback_subscription_;                    //声明订阅器
};


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveBackDemo::MoveBackDemo_Callback(const std_msgs::msg::Bool & msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg.data)
    {
        RCLCPP_INFO (this->get_logger(),"*******MoveBack succeeded\n");
    } else {
        RCLCPP_INFO (this->get_logger(),"*******MoveBack Failed\n");
    }
}   
/***********************************************end**************************************************/

/*******************************************获取位姿函数****************************************/
void MoveBackDemo::moveback_demo()
{

    rm_ros_interfaces::msg::Moveback moveB_C_TargetPose;
    moveB_C_TargetPose.pose.position.x = 0.0;
    moveB_C_TargetPose.pose.position.y = 0.0;
    moveB_C_TargetPose.pose.position.z = 0.85;
    moveB_C_TargetPose.pose.orientation.x = 0.0;
    moveB_C_TargetPose.pose.orientation.y = 0.0;
    moveB_C_TargetPose.pose.orientation.z = 0.0;
    moveB_C_TargetPose.pose.orientation.w = 1.0;
    moveB_C_TargetPose.speed = 20;
    moveB_C_TargetPose.trajectory_connect = 0;
    moveB_C_TargetPose.block = true;
    this->moveback_publisher_->publish(moveB_C_TargetPose);
}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
MoveBackDemo::MoveBackDemo():rclcpp::Node("Moveback_demo_node")
{

  moveback_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/moveb_c_result", rclcpp::ParametersQoS(), std::bind(&MoveBackDemo::MoveBackDemo_Callback, this,_1));
  moveback_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Moveback>("/rm_driver/moveb_c_cmd", rclcpp::ParametersQoS());
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  moveback_demo();
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveBackDemo>());
  rclcpp::shutdown();
  return 0;
}
