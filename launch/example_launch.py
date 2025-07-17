"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    robot_joint_names = [
                        "shoulder_pan_joint",
                        "shoulder_lift_joint",
                        "elbow_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint"
                        ]
    robot_joint_position_sensor_names = [
                    "shoulder_pan_joint_sensor",
                    "shoulder_lift_joint_sensor",
                    "elbow_joint_sensor",
                    "wrist_1_joint_sensor",
                    "wrist_2_joint_sensor",
                    "wrist_3_joint_sensor"
                    ]                     

    return LaunchDescription([
        DeclareLaunchArgument(
                    'sigterm_timeout',
                    default_value='30'
                ),
        Node(
            package='sas_robot_driver_unitree_z1',
            executable='sas_robot_driver_unitree_z1_node',
            name='ur3',
            namespace="sas",
            parameters=[{
                "robot_name": "ur3_1",
                "thread_sampling_time_sec": 0.002,    
                "robot_definition": "ur3",
                "robot_joint_names": robot_joint_names,
                "sampling_period": 32, 
                "reset_simulation" True,
                "robot_joint_position_sensor_names": robot_joint_position_sensor_names
                "joint_limits_min": [-360.0, -360.0, -360.0, -360.0, -360.0, -720.0],  # The last joint has no limit
                "joint_limits_max": [360.0, 360.0, 360.0, 360.0, 360.0, 720.0],  # The last joint has no limit
            }]
        ),

    ])