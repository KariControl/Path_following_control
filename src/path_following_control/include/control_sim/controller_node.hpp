#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  ControllerNode(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );

private:
  void timer_callback();
  void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr controller_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reference_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr controller_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  double steering_angle_;
  double velocity_;
  double kp_;
  double ki_;
  double k2_;
  double k3_;
  double dt_;
  double setpoint_;
  double integral_;
  double yawrate_;
  double x_;
  double y_;
  double theta_;
  double counter_;
  double curvature_;
  double theta_r_ ;
  double target_velocity_;
  double xr_;
  double yr_;
  double e2_;
  double e3_;
  double yawrate_r_;
  double yawrate_c_;
  double ew_;
  double ew_intg_;  
  double wheel_base_;
  double kff_;
};