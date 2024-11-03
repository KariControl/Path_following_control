#include "vehicle_sim/vehicle_plant.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


VehiclePlant::VehiclePlant(
  const rclcpp::NodeOptions& options
): VehiclePlant("",options){}

VehiclePlant::VehiclePlant(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("VehiclePlant", name_space, options){
    this->declare_parameter("vehicle_speed", 0.0);     // 車速デフォルト設定
    this->declare_parameter("a_coe", 1.0);     // 時定数
    this->declare_parameter("b_coe", 0.8);     // DCゲイン
    this->declare_parameter("diff_time", 0.01);     // サンプル時間

    this->get_parameter("vehicle_speed",vehicle_speed_); // Paramからの車速読み込み
    this->get_parameter("a_coe",a_coe_); //時定数計算のための係数
    this->get_parameter("b_coe",b_coe_); //DCゲイン計算のための係数
    this->get_parameter("diff_time",diff_time_); //差分時間

    // Publisher
    vehicle_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("vehicle_state", 1);// yaw_rate output

    // Subscriber
    str_angle_subscriber_=this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("steering_angle", 1, std::bind(&VehiclePlant::lateral_callback, this, std::placeholders::_1));
    velocity_subscriber_=this->create_subscription<geometry_msgs::msg::TwistStamped>("target_velocity", 1, std::bind(&VehiclePlant::Velocity_callback, this, std::placeholders::_1));

    DC_gain_=b_coe_*vehicle_speed_;
    time_constant_=a_coe_*vehicle_speed_;
}
void VehiclePlant::Velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    geometry_msgs::msg::TwistStamped output_msg;// 車速メッセージ定義
    // 問題設定簡単化のため、実車速と目標車速が一致すると仮定
    vehicle_speed_ = msg->twist.linear.x;
    DC_gain_=b_coe_*vehicle_speed_;
    time_constant_=a_coe_*vehicle_speed_;
}
void VehiclePlant::lateral_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    static double x_1;
    double u=msg->drive.steering_angle;// 操舵角入力
    double k1;//ルンゲクッタ中間変数1
    double k2;//ルンゲクッタ中間変数2
    double k3;//ルンゲクッタ中間変数3
    double k4;//ルンゲクッタ中間変数4
    
    // 1次遅れ系に対する4次ルンゲクッタ
    k1 = (-(1.0 / time_constant_) * x_1 + (DC_gain_ / time_constant_) * u);
    k2 = (-(1.0 / time_constant_) * (x_1 + diff_time_ * k1 / 2.0) + (DC_gain_ / time_constant_) * u);
    k3 = (-(1.0 / time_constant_) * (x_1 + diff_time_ * k2 / 2.0) + (DC_gain_ / time_constant_) * u);
    k4 = (-(1.0 / time_constant_) * (x_1 + diff_time_ * k3) + (DC_gain_ / time_constant_) * u);

    yaw_=yaw_+x_1*diff_time_;
    x_=x_+vehicle_speed_*cos(yaw_)*diff_time_;
    y_=y_+vehicle_speed_*sin(yaw_)*diff_time_;

    nav_msgs::msg::Odometry pose_output_msg;// 位置メッセージ定義
    pose_output_msg.header.stamp = this->now();
    pose_output_msg.header.frame_id = "map";
    pose_output_msg.pose.pose.position.x=x_;
    pose_output_msg.pose.pose.position.y=y_;
    tf2::Quaternion orientation_temp;
    orientation_temp.setRPY(0.0,0.0,yaw_);
    pose_output_msg.pose.pose.orientation.x=orientation_temp.x();
    pose_output_msg.pose.pose.orientation.y=orientation_temp.y();
    pose_output_msg.pose.pose.orientation.z=orientation_temp.z();
    pose_output_msg.pose.pose.orientation.w=orientation_temp.w();
    pose_output_msg.twist.twist.linear.x = vehicle_speed_;
    pose_output_msg.twist.twist.angular.z = x_1;
    vehicle_publisher_->publish(pose_output_msg);

    x_1 = x_1 + diff_time_ * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0; // 1サンプル先の状態変数を更新
}