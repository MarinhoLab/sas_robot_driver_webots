#include "sas_robot_driver_webots/sas_robot_driver_webots.hpp"
#include <dqrobotics/utils/DQ_Constants.h>


namespace sas
{

RobotDriverWebots::RobotDriverWebots(const RobotDriverWebotsConfiguration &configuration, std::atomic_bool *break_loops)
    :RobotDriver{break_loops}, current_status_{STATUS::IDLE}, configuration_{configuration}
{
    wbi_ = std::make_shared<DQ_WebotsInterface>(configuration_.sampling_period);
}

VectorXd RobotDriverWebots::get_joint_positions()
{
    return q_;
}

void RobotDriverWebots::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    wbi_->set_joint_target_positions(configuration_.robot_joint_names, desired_joint_positions_rad);
}

std::tuple<VectorXd, VectorXd> RobotDriverWebots::get_joint_limits()
{
    //TODO: Obtain the joint limits from the simulator. This does not seem to be trivial as of now.
    int dof = get_joint_positions().size();
    auto joint_positions_max = VectorXd::Ones(dof)*2*pi;
    auto joint_positions_min = -joint_positions_max;
    return {joint_positions_min, joint_positions_max};
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

void RobotDriverWebots::disconnect()
{
    if (control_mode_thread_.joinable())
        control_mode_thread_.join();
    status_msg_ = "Disconnected.";
    current_status_ = STATUS::DISCONNECTED;
}

void RobotDriverWebots::initialize()
{
    if (current_status_ == STATUS::CONNECTED)
    {
        wbi_->trigger_next_simulation_step();
        q_ = wbi_->get_joint_positions(configuration_.robot_joint_position_sensor_names);
        q_target_ = q_;
        _start_control_mode_thread();
        status_msg_ = "initialized!";
    }
}

void RobotDriverWebots::deinitialize()
{
    finish_motion_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    status_msg_ = "Deinitialized.";
    current_status_ = STATUS::DEINITIALIZED;
}

void RobotDriverWebots::_control_mode()
{
    while(!finish_motion_ || !st_break_loops_)
    {
        q_ = wbi_->get_joint_positions(configuration_.robot_joint_position_sensor_names);
        wbi_->set_joint_target_positions(configuration_.robot_joint_names, q_target_);
        wbi_->trigger_next_simulation_step();
    }
}

void RobotDriverWebots::_start_control_mode_thread()
{
    finish_motion_ = false;
    if (control_mode_thread_.joinable())
        control_mode_thread_.join();
    control_mode_thread_ = std::thread(&RobotDriverWebots::_control_mode, this);
}



}
