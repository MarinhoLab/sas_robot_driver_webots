#include "sas_robot_driver_webots/sas_robot_driver_webots.hpp"
#include <dqrobotics/utils/DQ_Constants.h>


namespace sas
{

RobotDriverWebots::RobotDriverWebots(const std::shared_ptr<rclcpp::Node> &node,
                                     const RobotDriverWebotsConfiguration &configuration,
                                     std::atomic_bool *break_loops)
    :RobotDriver{break_loops}, current_status_{STATUS::IDLE}, configuration_{configuration},
    node_{node}, st_break_loops_{break_loops}, finish_motion_{false}
{
    wbi_ = std::make_shared<DQ_WebotsInterface>(configuration_.sampling_period);
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::RobotDriverWebots is brought to you by Juan Jose Quiroz Omana.");
}

VectorXd RobotDriverWebots::get_joint_positions()
{
    return q_;
}

void RobotDriverWebots::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    q_target_ = desired_joint_positions_rad;
}


void RobotDriverWebots::connect()
{
    if (current_status_ == STATUS::IDLE)
    {
        if(!wbi_->connect(configuration_.robot_definition))
            throw std::runtime_error("::Unable to connect to Webots.");
        current_status_ = STATUS::CONNECTED;
        status_msg_ = "connected!";
    }
}



void RobotDriverWebots::initialize()
{
    if (current_status_ == STATUS::CONNECTED)
    {
        wbi_->trigger_next_simulation_step();
        q_ = wbi_->get_joint_positions(configuration_.robot_joint_position_sensor_names);
        q_target_ = q_;
        status_msg_ = "initialized!";
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::RobotDriverWebots initializing...");

        // Start thread
        finish_motion_ = false;
        if (control_mode_thread_.joinable())
            control_mode_thread_.join();
        control_mode_thread_ = std::thread(&RobotDriverWebots::_control_mode, this);
    }
}

void RobotDriverWebots::deinitialize()
{
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::RobotDriverWebots deinitialize().");

    finish_motion_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    status_msg_ = "Deinitialized.";
    current_status_ = STATUS::DEINITIALIZED;

    if (configuration_.reset_simulation)
        wbi_->reset_simulation();
}

void RobotDriverWebots::disconnect()
{
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::RobotDriverWebots disconnect().");
    if (control_mode_thread_.joinable())
        control_mode_thread_.join();
    status_msg_ = "Disconnected.";
    current_status_ = STATUS::DISCONNECTED;
}


/**
 * @brief RobotDriverWebots::_control_mode
 */
void RobotDriverWebots::_control_mode()
{
    while(!finish_motion_ && !(*st_break_loops_))
    {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::RobotDriverWebots control loop running!");
        q_ = wbi_->get_joint_positions(configuration_.robot_joint_position_sensor_names);
        wbi_->set_joint_target_positions(configuration_.robot_joint_names, q_target_);
        wbi_->trigger_next_simulation_step();
    }
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::RobotDriverWebots control loop break.");
}



RobotDriverWebots::~RobotDriverWebots()
{

}



}
