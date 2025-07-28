#include "sas_robot_driver_webots/sas_robot_driver_webots.hpp"
#include <dqrobotics/utils/DQ_Constants.h>


namespace sas
{

RobotDriverWebots::RobotDriverWebots(
    const RobotDriverWebotsConfiguration &configuration,
    std::atomic_bool *break_loops)
    :RobotDriver{break_loops}, configuration_{configuration},
    st_break_loops_{break_loops}, finish_motion_{false}
{
    _make_webots_pointer(configuration.sampling_period);
}

RobotDriverWebots::RobotDriverWebots(std::atomic_bool *break_loops)
:RobotDriver{break_loops}, st_break_loops_{break_loops}, finish_motion_{false}
{
    _make_webots_pointer(32);
}

void RobotDriverWebots::set_parameters(const RobotDriverWebotsConfiguration &configuration)
{
    configuration_ = configuration;
}

/**
 * @brief RobotDriverWebots::get_status
 * @return
 */
std::string RobotDriverWebots::get_status() const
{
    return status_msg_;
}

/**
 * @brief RobotDriverWebots::get_joint_positions get the joint positions
 * @return The desired joint positions
 */
VectorXd RobotDriverWebots::get_joint_positions()
{
    return q_;
}

/**
 * @brief RobotDriverWebots::set_target_joint_positions sets the target joint positions
 * @param desired_joint_positions_rad
 */
void RobotDriverWebots::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    q_target_ = desired_joint_positions_rad;
}


/**
 * @brief RobotDriverWebots::connect
 */
void RobotDriverWebots::connect()
{
  if(!wbi_->connect(configuration_.robot_definition))
            throw std::runtime_error("::Unable to connect to Webots.");
        wbi_->set_sampling_period(configuration_.sampling_period);
        status_msg_ = "connected!";
}


/**
 * @brief RobotDriverWebots::initialize
 */
void RobotDriverWebots::initialize()
{
    for (auto i =0; i<INITIAL_SAMPLES_; i++)
    {
        q_ = wbi_->get_joint_positions(configuration_.robot_joint_position_sensor_names);
        wbi_->trigger_next_simulation_step();
    }
    q_target_ = q_;
    status_msg_ = "initialized!";

    // Start thread
    finish_motion_ = false;
    if (control_mode_thread_.joinable())
        control_mode_thread_.join();
    control_mode_thread_ = std::thread(&RobotDriverWebots::_control_mode, this);

}


/**
 * @brief RobotDriverWebots::deinitialize
 */
void RobotDriverWebots::deinitialize()
{

    finish_motion_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    status_msg_ = "Deinitialized.";

    if (configuration_.reset_simulation)
        wbi_->reset_simulation();
}


/**
 * @brief RobotDriverWebots::disconnect
 */
void RobotDriverWebots::disconnect()
{
    if (control_mode_thread_.joinable())
        control_mode_thread_.join();
    status_msg_ = "Disconnected.";
}


/**
 * @brief RobotDriverWebots::_control_mode
 */
void RobotDriverWebots::_make_webots_pointer(const int &sampling_period)
{
    wbi_ = std::make_shared<DQ_WebotsInterface>(sampling_period);
}

void RobotDriverWebots::_control_mode()
{
    while(!finish_motion_ && !(*st_break_loops_))
    {

        q_ = wbi_->get_joint_positions(configuration_.robot_joint_position_sensor_names);
        wbi_->set_joint_target_positions(configuration_.robot_joint_names, q_target_);
        wbi_->trigger_next_simulation_step();
    }

}



}
