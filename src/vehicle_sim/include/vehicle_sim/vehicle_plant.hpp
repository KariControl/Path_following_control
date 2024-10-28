#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

class VehiclePlant : public rclcpp::Node
{
public:
  VehiclePlant(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  VehiclePlant(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );

private:
  void lateral_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  void Velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr yawrate_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr str_angle_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  double steering_angle_;
  double yaw_rate_;
  double vehicle_speed_;
  double time_constant_;//参照モデルの時定数
  double DC_gain_;//参照モデルの時定数
  double diff_time_;//差分時間
  double b_coe_;//係数b
  double a_coe_;//係数a

};