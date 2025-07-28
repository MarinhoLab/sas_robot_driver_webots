![Static Badge](https://img.shields.io/badge/status-experimental-critical)![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![Static Badge](https://img.shields.io/badge/Webots-R2025a-orange)

# sas_robot_driver_webots

A SAS driver based on [cpp-interface-webots](https://github.com/juanjqo/cpp-interface-webots) and [DQ Robotics](https://github.com/dqrobotics/cpp) to integrate Webots in ROS2 Jazzy.




https://github.com/user-attachments/assets/61a00c57-6f49-484f-89db-f7d139ffc121



## Instructions (for Devs)

Prerequisites
- [DQ Robotics](https://github.com/dqrobotics/cpp)
- [cpp-interface-webots](https://github.com/juanjqo/cpp-interface-webots)
- ROS Jazzy and [SAS](https://smartarmstack.github.io/)
- [Webots](https://cyberbotics.com/) 
- Open the [ur3.wbt](https://github.com/juanjqo/sas_robot_driver_webots/blob/main/worlds/ur3.wbt) scene

### Update LD_LIBRARY_PATH to include /usr/local/webots/lib/controller/

```shell
echo "# Update the environment variable LD_LIBRARY_PATH to include /usr/local/webots/lib/controller/, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/webots/lib/controller/" >> ~/.bashrc
```

### Build this ROS Package. 
For instance, if you build your ROS packages in `~/ros2_ws/`:
``` shell
cd ~/ros2_ws/src/
git clone https://github.com/juanjqo/sas_robot_driver_webots
cd ..
colcon build && source install/setup.bash
```

```shell
ros2 launch sas_robot_driver_webots example_launch.py 
```

### Move the robot on Webots:

```shell
ros2 topic pub /sas/ur3/set/target_joint_positions std_msgs/msg/Float64MultiArray "data: [-0.5,0,0,0,0,0]"
```
