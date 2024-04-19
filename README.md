# VTOL_MSN_TakeOff_And_Landing_ROS2 for Airsim 

VTOL aircraft Multi-Sensor Navigation, in Take Off And Landing scenario.
Based on the Error-State Kalman Filter, a fusion navigation framework that combines INS, GNSS, Vision, and UWB modules.  
 <img src="https://github.com/Space-Exploration-UAVTeam/VTOL_MSN_TakeOff_And_Landing/blob/master/imgs/123.png"  width="1200" />  

## 1. Changes from ROS1 version
IMU data from Airsim topic:
```
/airsim_node/[DroneName]/imu/imu
```
GNSS data from Airsim topic: 
```
/airsim_node/[DroneName]/global_gps
```
AprilTag data from AprilTag_ros2 pkg, which got camera data from Airsim topic: 
```
/airsim_node/[DroneName]/bottom_cam/Scene
```
Airsim dose not have a UWB sensor or a derict yaw topic, so we obtain these two from Airsim topic
```
/airsim_node/[DroneName]/odom_local_ned
```
Airsim dose not have a GNSS satellite number or anything related to the GNSS positioning quality, so we ignore the "/satellite" topic.  

## 2. Build 
Clone the repository to your catkin workspace (for example `~/ros2_ws/`):
```
cd ~/ros2_ws/src/
git clone https://github.com/Space-Exploration-UAVTeam/VTOL_MSN_TakeOff_And_Landing_ROS2.git
git branch airsim
```
Then build the package with:
```
cd ~/ros2_ws/
colcon build
source ~/ros2_ws/install/setup.bash
```

## 3. Run
```
ros2 launch vtol_msn_takeoff_landing vtol_msn_takeoff_landing_launch.py
```
