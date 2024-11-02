#include "control_sim/controller_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ControllerNode::ControllerNode(
  const rclcpp::NodeOptions& options
): ControllerNode("",options){}

ControllerNode::ControllerNode(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("ControllerNode", name_space, options){
    this->declare_parameter("kp", 0.2);         // デフォルト値として0.1を設定
    this->declare_parameter("ki", 0.3);        // デフォルト値として0.01を設定
    this->declare_parameter("k2", 0.3);        // デフォルト値として0.01を設定
    this->declare_parameter("k3", 0.3);        // デフォルト値として0.01を設定
    this->declare_parameter("dt", 0.05);        // デフォルト値として0.05を設定
    this->declare_parameter("target_velocity", 5.0);        // デフォルト値として0.05を設定
    this->declare_parameter("wheel_base", 1.0);        // デフォルト値として0.05を設定

    this->get_parameter("kp",kp_);
    this->get_parameter("ki",ki_);
    this->get_parameter("k2",k2_);
    this->get_parameter("k3",k3_);
    this->get_parameter("dt",dt_);
    this->get_parameter("target_velocity",target_velocity_);
    this->get_parameter("wheel_base",wheel_base_);

    controller_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("vehicle_state", 1, std::bind(&ControllerNode::state_callback, this, std::placeholders::_1));

    controller_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("steering_angle", 1);
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("target_velocity", 1);
    reference_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("target_path", 1);

    using namespace std::literals::chrono_literals; // これが無いと、create_wall_timer等での100msとかの時間単位付きの変数を指定できない
    timer_ = this->create_wall_timer(20ms, std::bind(&ControllerNode::timer_callback, this));
}
void ControllerNode::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  yawrate_=msg->twist.twist.angular.z;
  velocity_=msg->twist.twist.linear.x;
  x_=msg->pose.pose.position.x;
  y_=msg->pose.pose.position.y;
  tf2::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);
  theta_=yaw;
  counter_=counter_+dt_;
  // 目標値計算：本来はプランナーモジュール側で計算するが、今回はデモ版なので制御内部で目標値も計算
  // 直進後に一定旋回半径を曲がる目標パスを計算
  if(counter_<5.0)
  {
    curvature_=0.0;
  }
  else if (counter_>15.0 && counter_<25.0)
  {
    curvature_=0.05;
  }
  else if (counter_>=25.0)
  {
    curvature_=0.0;
    counter_=25.0;
  }
  theta_r_ = theta_r_+curvature_*target_velocity_*dt_;// target heading
  xr_ = xr_+target_velocity_*cos(theta_r_)*dt_; // target path x
  yr_ = yr_+target_velocity_*sin(theta_r_)*dt_;// target path y
  // 誤差計算
  e2_=-sin(theta_r_)*(x_-xr_)+cos(theta_r_)*(y_-yr_);// lateral error
  e3_=theta_-theta_r_;// heading error
  
  // メッセージpublish処理
  geometry_msgs::msg::TwistStamped output_msg;
  output_msg.header.stamp = this->now();
  output_msg.header.frame_id = "base_link";
  output_msg.twist.linear.x = target_velocity_;
  velocity_publisher_->publish(output_msg); 

  nav_msgs::msg::Odometry reference_msg;
  reference_msg.header.stamp=this->now();
  reference_msg.pose.pose.position.x=xr_;
  reference_msg.pose.pose.position.y=yr_;
  tf2::Quaternion orientation_temp;
  orientation_temp.setRPY(0.0,0.0,theta_r_);

  reference_msg.pose.pose.orientation.x=orientation_temp.x();
  reference_msg.pose.pose.orientation.y=orientation_temp.y();
  reference_msg.pose.pose.orientation.z=orientation_temp.z();
  reference_msg.pose.pose.orientation.w=orientation_temp.w();
  reference_msg.twist.twist.linear.x=target_velocity_;
  reference_msg.twist.twist.angular.z=curvature_*target_velocity_;
  reference_publisher_->publish(reference_msg);
}
void ControllerNode::timer_callback() {
  // Path following制御による目標ヨーレート計算
  yawrate_r_=curvature_*(velocity_*cos(e3_)/(1-e2_*curvature_));
  yawrate_c_=yawrate_r_-k2_*e2_*velocity_*-k3_*sin(e3_);

  // ヨーレート制御
  ew_=yawrate_c_-yawrate_;
  ew_intg_=ew_intg_+ew_*dt_;
  // steering_angle_=atan2(wheel_base_*yawrate_c_,velocity_)+kp_*ew_+ki_*ew_intg_;    
  steering_angle_=kp_*ew_+ki_*ew_intg_;    

  // 操舵角入力計算
  ackermann_msgs::msg::AckermannDriveStamped output_msg;
  output_msg.header.stamp = this->now();
  output_msg.header.frame_id = "base_link";
  output_msg.drive.steering_angle = steering_angle_;
  controller_publisher_->publish(output_msg);    
}