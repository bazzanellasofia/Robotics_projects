#include <functional>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bunker_msgs/msg/bunker_status.hpp"           // ← snake_case
#include "nav_msgs/msg/odometry.hpp"                    // ← snake_case
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;


class Odometer : public rclcpp::Node
{
public:
  Odometer() : Node("odometer")
  {
    subscription_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>("/bunker_status", 10, std::bind(&Odometer::topic_callback, this, _1));
     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [this](const nav_msgs::msg::Odometry & msg) {
        if (!odom_initialized_) {
            // Prendi posizione iniziale dalla bag
            x_   = msg.pose.pose.position.x;
            y_   = msg.pose.pose.position.y;
            // Estrai yaw dal quaternione
            const double qz = msg.pose.pose.orientation.z;
            const double qw = msg.pose.pose.orientation.w;
            yaw_ = 2.0 * std::atan2(qz, qw);
            odom_initialized_ = true;

            RCLCPP_INFO(this->get_logger(),
                "Posizione iniziale: x=%.3f y=%.3f yaw=%.3f", x_, y_, yaw_);
        }
    });
    
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
    "reset",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
           std::shared_ptr<std_srvs::srv::Empty::Response>) {
        x_   = 0.0;
        y_   = 0.0;
        yaw_ = 0.0;
        initialized_     = false;
        odom_initialized_ = false;
        RCLCPP_INFO(this->get_logger(), "Odometria resettata");
    });
  }

private:
  
   static constexpr double WHEEL_RADIUS   = 0.085;  // [m] raggio della ruota r++ --> velocità lineare più alta r-- --> velocità lineare più bassa
   static constexpr double GEAR_REDUCTION = 7.5; 
   static constexpr double L = 0.710;  // [m] distanza tra le ruote L++ --> curva di meno L-- --> curva di più
  
   void topic_callback(const bunker_msgs::msg::BunkerStatus & msg)
  {
    if (!odom_initialized_) return;  // aspetta posizione iniziale dalla bag
    
    const auto now = this->now();
    
    if (!initialized_) {
    last_update_time_ = now;
    initialized_ = true;
    return;  // ← non pubblicare il primo messaggio
    }

    const double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    // Read source values from incoming BunkerStatus.
    double rpm_right = msg.actuator_states[0].rpm;
    double rpm_left  = msg.actuator_states[1].rpm;

    double v_r = (rpm_right / 60.0) * 2.0 * M_PI * WHEEL_RADIUS / GEAR_REDUCTION;
    double v_l = (rpm_left  / 60.0) * 2.0 * M_PI * WHEEL_RADIUS / GEAR_REDUCTION;

    // Velocità lineare e angolare media
    const double v = (v_r + v_l) / 2.0;
    const double w = (v_r - v_l) / L;

    if (dt > 0.0 && dt < 0.5){
    // Stato corrente
    const double yaw_mid = yaw_ + w * dt / 2.0;
    const double yaw_end = yaw_ + w * dt;

    // k1 — inizio
    const double k1_x = v * std::cos(yaw_);
    const double k1_y = v * std::sin(yaw_);

    // k2 — metà intervallo
    const double k2_x = v * std::cos(yaw_mid);
    const double k2_y = v * std::sin(yaw_mid);

    // k3 — fine
    const double k3_x = v * std::cos(yaw_end);
    const double k3_y = v * std::sin(yaw_end);

    // Aggiornamento RK4
    yaw_ += w * dt;
    x_   += (dt / 6.0) * (k1_x + 4.0*k2_x + k3_x);
    y_   += (dt / 6.0) * (k1_y + 4.0*k2_y + k3_y);
  }

    nav_msgs::msg::Odometry odom_msg;

    // Fill Odometry header and frames.
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link2";

    // Fill pose.
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Quaternion from yaw (roll = pitch = 0).
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odom_msg.pose.pose.orientation.w = std::cos(yaw_ * 0.5);

    // Fill twist.
    odom_msg.twist.twist.linear.x  = v;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = w;

    publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = now;
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link2";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = std::sin(yaw_ * 0.5);
    t.transform.rotation.w = std::cos(yaw_ * 0.5);

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_update_time_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  
  bool initialized_ = false;
  bool odom_initialized_ = false;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}
