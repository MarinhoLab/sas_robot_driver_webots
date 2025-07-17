#pragma once
#include <atomic>
#include <memory>
#include <sas_core/sas_robot_driver.hpp>
#include <dqrobotics/interfaces/webots/DQ_WebotsInterface.h>
#include <thread>

using namespace Eigen;

namespace sas
{

struct RobotDriverWebotsConfiguration
{
    //int port;
    //int timeout = 1000;
    std::string robot_definition;

    int sampling_period;
    std::vector<std::string> robot_joint_names;
    std::vector<std::string> robot_joint_position_sensor_names;
    bool reset_simulation;
};

class RobotDriverWebots: public RobotDriver
{
private:
    enum class STATUS{
        IDLE,
        CONNECTED,
        INITIALIZED,
        DEINITIALIZED,
        DISCONNECTED,
    };
    STATUS current_status_;

private:
    RobotDriverWebotsConfiguration configuration_;
    std::shared_ptr<DQ_WebotsInterface> wbi_;
    VectorXd q_;
    VectorXd q_target_;
    std::string status_msg_;
protected:
    std::atomic_bool* st_break_loops_;

    void _control_mode();
    std::thread control_mode_thread_;
    void _start_control_mode_thread();
    std::atomic<bool> finish_motion_;

public:
    // Prevent copies as usually drivers have threads
    RobotDriverWebots(const RobotDriverWebots&)=delete;
    RobotDriverWebots()=delete;
    virtual ~RobotDriverWebots() = default;

    RobotDriverWebots(const RobotDriverWebotsConfiguration &configuration, std::atomic_bool* break_loops);

    /// Everything below this line is an override
    /// the concrete implementations are needed
    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;
    std::tuple<VectorXd, VectorXd> get_joint_limits() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

};

}
