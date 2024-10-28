#include "control_sim/controller_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

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
    this->declare_parameter("dt", 0.01);        // デフォルト値として0.05を設定

    this->get_parameter("kp",kp_);
    this->get_parameter("ki",ki_);
    this->get_parameter("k2",k2_);
    this->get_parameter("k3",k3_);
    this->get_parameter("dt",dt_);

    controller_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("vehicle_velocity", 1, std::bind(&ControllerNode::state_callback, this, std::placeholders::_1));
    reference_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("vehicle_velocity", 1, std::bind(&ControllerNode::reference_callback, this, std::placeholders::_1));

    controller_publisher_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("accel", 1);

    using namespace std::literals::chrono_literals; // これが無いと、create_wall_timer等での100msとかの時間単位付きの変数を指定できない
    timer_ = this->create_wall_timer(20ms, std::bind(&ControllerNode::timer_callback, this));
}
void ControllerNode::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  yawrate_=msg->twist.angular.z;
  velocity_=msg->twist.linar.x;
  x_=msg->pose.pose.linear.x;
  y_=msg->pose.pose.linear.y;
  tf2::Quaternion quaternion_temp;
  quaternion_temp.setRPY(0.0,0.0,theta_);
  counter_=counter_+dt_;
  // 目標値計算：本来はプランナーモジュール側で計算するが、今回はデモ版なので制御内部で目標値も計算
  if(counter<5.0)
  {
    curvature_=0.0;
  }
  else if (counter>15.0 && counter<25.0)
  {
    curvature_=0.05;
  }
  else if (counter>25.0)
  {
    curvature_=0.0;
    counter_=25.0;
  }
  // 誤差計算
  theta_r_ = theta_r_+curvature_*target_velocity_*dt_;
  xr_ = xr_+target_velocity_*cos(theta_r_);
  yr_ = yr_+target_velocity_*sin(theta_r_);
  e2_=-sin(theta_r_)*(x_-xr_)+cos(theta_r_)*(y_-yr_);// lateral error
  e3_=theta_-theta_r_;// heading error
}
void ControllerNode::timer_callback() {
  yawrate_r_=curvature_*(velocity_*cos(e3_)/(1-e2_*curvature_));
  yawrate_c_=w_r_-k2_*e2_*velocity_*-k3_*sin(e3_);
  ew_=yawrate_c_-yawrate_;
  ew_intg_=ew_intg_+ew_*dt_;
  steering_angle_=atan2(wheelbase_*yawrate_,velocity)+kp_*ew_+ki*ew_intg_;    

  ackermann_msgs::msg:AckermannDriveStamped output_msg;
  output_msg.header.stamp = this->now();
  output_msg.header.frame_id = "base_link";
  output_msg.drive.steering_angle = steering_angle_;
  controller_publisher_->publish(output_msg);    

  geometry_msgs::msg:TwistedStamped output_msg;
  output_msg.header.stamp = this->now();
  output_msg.header.frame_id = "base_link";
  output_msg.drive.steering_angle = steering_angle_;
  controller_publisher_->publish(output_msg); 
}