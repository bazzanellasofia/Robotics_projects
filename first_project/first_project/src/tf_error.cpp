#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "first_project/msg/tf_error_msg.hpp"   

using namespace std::chrono_literals;

class TfError : public rclcpp::Node
{
public:
    TfError() : Node("tf_error")
    {
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_publisher_ = this->create_publisher<first_project::msg::TfErrorMsg>("tf_error_msg", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TfError::on_timer, this));
    }

private:

    float travelled_distance_ = 0.0;   
    float prec_x_ = 0.0;               
    float prec_y_ = 0.0;
    int64_t start_time_sec_ = -1;  // Timestamp della prima trasformazione ricevuta

    void on_timer()
    {
        geometry_msgs::msg::TransformStamped base_link;
        geometry_msgs::msg::TransformStamped base_link2;

        try {
            base_link  = tf_buffer_->lookupTransform("odom", "base_link",  tf2::TimePointZero);
            base_link2 = tf_buffer_->lookupTransform("odom", "base_link2", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Lookup failed: %s", ex.what());
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "base_link in odom -> x: %.2f  y: %.2f\n"
            "base_link2 in odom -> x: %.2f  y: %.2f",
            base_link.transform.translation.x,
            base_link.transform.translation.y,
            base_link2.transform.translation.x,
            base_link2.transform.translation.y);

        // Errore tra base_link e base_link2
        float error_x = base_link.transform.translation.x - base_link2.transform.translation.x;
        float error_y = base_link.transform.translation.y - base_link2.transform.translation.y;
        float error   = std::sqrt(error_x * error_x + error_y * error_y);

        // Calcola tempo relativo usando il timestamp della trasformazione della bag
        int64_t current_sec = base_link.header.stamp.sec;
        if (start_time_sec_ == -1) {
            start_time_sec_ = current_sec;  // Primo messaggio ricevuto
        }
        int32_t elapsed_sec = static_cast<int32_t>(current_sec - start_time_sec_);

        // Distanza percorsa da base_link2
        float dx = base_link2.transform.translation.x - prec_x_;
        float dy = base_link2.transform.translation.y - prec_y_;
        travelled_distance_ += std::sqrt(dx * dx + dy * dy);  

        prec_x_ = base_link2.transform.translation.x;
        prec_y_ = base_link2.transform.translation.y;

        // Pubblica messaggio
        first_project::msg::TfErrorMsg error_msg;
        
        error_msg.header.stamp    = this->get_clock()->now();
        error_msg.header.frame_id = "odom";
        error_msg.tf_error = error;
        error_msg.time_from_start = elapsed_sec;
        error_msg.travelled_distance = travelled_distance_;

        tf_publisher_->publish(error_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr tf_publisher_;  // ← mancava
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfError>());
    rclcpp::shutdown();
    return 0;
}