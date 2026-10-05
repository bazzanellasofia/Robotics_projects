#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

struct Target {
    double x;
    double y;
    double theta;
};

class CsvController : public rclcpp::Node {
public:
    CsvController() : Node("csv_controller"), current_goal_index_(0) {
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("second_project");
        std::string csv_file_path = pkg_share_dir + "/csv/goals.csv";
        load_csv(csv_file_path);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&CsvController::check_server_and_send, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;  
    std::vector<Target> goals_;
    size_t current_goal_index_;

    void load_csv(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "ERRORE CRITICO: Impossibile aprire il CSV: %s", filename.c_str());
            return;
        }

        std::string line;
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            Target t;

            if (std::getline(ss, token, ',')) t.x = std::stod(token);
            if (std::getline(ss, token, ',')) t.y = std::stod(token);
            if (std::getline(ss, token, ',')) t.theta = std::stod(token);

            goals_.push_back(t);
        }
        RCLCPP_INFO(this->get_logger(), "Caricati %zu goal dal CSV.", goals_.size());
    }

    void check_server_and_send() {
        if (!client_ptr_->action_server_is_ready()) {
            RCLCPP_INFO(this->get_logger(), "In attesa di Nav2...");
            return; 
        }
        timer_->cancel();
        send_next_goal();
    }

    void send_next_goal() {
        if (current_goal_index_ >= goals_.size()) {
            RCLCPP_INFO(this->get_logger(), "MISSIONE COMPIUTA! Tutti i goal sono stati raggiunti.");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        
        Target current_target = goals_[current_goal_index_];
        goal_msg.pose.pose.position.x = current_target.x;
        goal_msg.pose.pose.position.y = current_target.y;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, current_target.theta);
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = 
            std::bind(&CsvController::goal_response_callback, this, std::placeholders::_1);
            
        send_goal_options.result_callback = 
            std::bind(&CsvController::result_callback, this, std::placeholders::_1);
        
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Invio goal %zu/%zu: [X: %.2f, Y: %.2f, Theta: %.2f]", 
                    current_goal_index_ + 1, goals_.size(), current_target.x, current_target.y, current_target.theta);
        
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Goal RIFIUTATO, riprovo tra 2 secondi...");
            retry_timer_ = this->create_wall_timer(
                2000ms, [this]() {
                    retry_timer_->cancel();
                    send_next_goal();
                });
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accettato! Navigazione in corso...");
        }
    }

    void result_callback(const GoalHandleNav::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "--> Goal raggiunto con successo!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "--> Goal ABORTITO!");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "--> Goal CANCELLATO.");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "--> Risultato sconosciuto.");
                break;
        }
        
        rclcpp::sleep_for(1s); 
        current_goal_index_++;
        send_next_goal();
    }
};

int main(int argc, char ** argv) {
    setbuf(stdout, NULL);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CsvController>());
    rclcpp::shutdown();
    return 0;
}
