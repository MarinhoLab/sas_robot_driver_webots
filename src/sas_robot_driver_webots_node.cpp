#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_robot_driver_webots/sas_robot_driver_webots.hpp>
#include <dqrobotics/utils/DQ_Math.h>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process{false};
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    // Since Webots captures the SIGINT signals, I decided to initialize Webots first in order to
    // override the SIGINT interruption.
    auto robot_driver_webots= std::make_shared<sas::RobotDriverWebots>(&kill_this_process);

    // This code is executed after the initialization of Webots. Therefore, the SIGINT interruption that is
    // provided by Webots is overrided by sig_int_handler.
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");


    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_webots_myrobot");

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverWebotsConfiguration configuration;

        /* Not implemented yet, but it is on the roadmap.
        std::vector<double> joint_limits_min;
        std::vector<double> joint_limits_max;
        sas::get_ros_parameter(node,"joint_limits_min",joint_limits_min);
        sas::get_ros_parameter(node,"joint_limits_max",joint_limits_max);
        configuration.joint_limits = {deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_min)),
                                      deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_max))};
        */

        sas::get_ros_parameter(node,"robot_joint_names",configuration.robot_joint_names);
        sas::get_ros_parameter(node,"robot_definition",configuration.robot_definition);
        sas::get_ros_parameter(node,"sampling_period",configuration.sampling_period);
        sas::get_ros_parameter(node,"robot_joint_position_sensor_names",configuration.robot_joint_position_sensor_names);
        sas::get_ros_parameter(node,"reset_simulation",configuration.reset_simulation);
        //sas::get_ros_optional_parameter(node,"robot_definition",configuration.robot_definition, std::string("127.0.0.1"));
        //sas::get_ros_optional_parameter(node,"port",configuration.port, 23000);
        //sas::get_ros_optional_parameter(node, "timeout", configuration.timeout, 1000);
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");

        // This constructor follows the SAS style. However, in this case, Webots overrides the SIGINT interruption, which
        // could be incompatible with the SAS implementation.
        //RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverWebots with DEF: \""+configuration.robot_definition+"\"");
        //auto robot_driver_webots= std::make_shared<sas::RobotDriverWebots>(configuration, &kill_this_process);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Setting RobotDriverWebots parameters with DEF: \""+configuration.robot_definition+"\"");
        robot_driver_webots->set_parameters(configuration);

        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        sas::get_ros_parameter(node,"thread_sampling_time_sec", robot_driver_ros_configuration.thread_sampling_time_sec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_webots,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }


    //sas::display_signal_handler_none_bug_info(node);
    return 0;
}
