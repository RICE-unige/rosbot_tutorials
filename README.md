
# ROSbot Tutorials

Welcome to the **ROSbot Tutorials** repository! This repository contains detailed instructions for working with the ROSbot 2.0 to complete assignment projects and classwork in the **Experimental Robotics** course 🚀.

---

## 📚 Table of Contents

1. [Prerequisites](#prerequisites)
2. [Getting Started](#getting-started)
3. [Assignments and Projects](#assignments-and-projects)
4. [Repository Structure](#repository-structure)
5. [Support and Troubleshooting](#support-and-troubleshooting)

---

## 🛠 Prerequisites

Before you begin, ensure you meet the following requirements:

### Hardware:
- Access to a **ROSbot 2.0**.
- A compatible computer for programming and simulation:
  - **Linux Distribution** (specifically **Ubuntu 20.04**) is recommended for compatibility with **ROS 1 Noetic** and **ROS 2 Foxy**.
  - Use one of the following methods if you don't have native Linux:
    - Virtual Machine
    - Docker
    - WSL (Windows Subsystem for Linux)
    - Dual Boot PC

### Software:
- **ROS 1** (Noetic or Melodic) and **ROS 2 Foxy** installed on your computer.

---

## 🚀 Getting Started

### Step 1: Connect to the ROSbot
1. Connect your laptop to the **[FRITZ!Box 6850 XU] WiFi** or the specific WiFi the robots are connected to.
   - **WiFi Password**: `06537900605465042712`
2. Turn on the ROSbot and wait for about **2 minutes** for it to boot up.
3. Copy the **IP Address** written on the robot. This will be referred to as `<ROBOT_IP>`.

### Step 2: SSH into the ROSbot
1. Open a terminal and run the following command to SSH into the robot:
   ```bash
   ssh husarion@<ROBOT_IP>
   ```
   - **Password**: `husarion`

### Step 3: Set Up Parameters on Your Laptop
1. Ensure your laptop is connected to the same WiFi as the ROSbot.
2. Get the IP address of your laptop:
   ```bash
   hostname -I
   ```
   - Use the IP starting with `192.168.178.XX` if multiple IPs are listed. This will be referred to as `<PC_IP>`.
3. Open your `.bashrc` file for editing:
   ```bash
   gedit ~/.bashrc
   ```
4. Add the following lines to your `.bashrc` file:
   ```bash
   export ROS_MASTER_URI=http://<ROBOT_IP>:11311
   export ROS_IP=<PC_IP>
   ```
5. Save the file and source it:
   ```bash
   source ~/.bashrc
   ```

### Step 4: Start the ROSbot
1. Go back to the terminal where you SSH’d into the robot and run:
   ```bash
   roslaunch tutorial_pkg all.launch
   ```
2. Open a new terminal on your laptop and run:
   ```bash
   rostopic list
   ```
   - Ensure you **DO NOT** run `roscore` on your laptop, as the ROSbot is the ROS Master.

3. You can now use your ROS workspace on your laptop to control the robot.

---

## ROS 2 Users

To use ROS 2 to control the robot, follow these additional steps:

### Step 1: Install ROS 2 Foxy
- Follow the instructions [here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) to install ROS 2 Foxy.

### Step 2: Install `ros1_bridge`
- Install the `ros1_bridge` package:
  ```bash
  sudo apt install ros-foxy-ros1-bridge
  ```

### Step 3: Prepare Shells
- Open **three terminal shells** and source the appropriate environments:
  1. **Shell 1**:
     ```bash
     source /opt/ros/noetic/setup.bash
     ```
     - Use this shell for ROS 1 operations.
  2. **Shell 2**:
     ```bash
     source /opt/ros/noetic/setup.bash
     source /opt/ros/foxy/setup.bash
     ```
     - Use this shell to run the ROS 1 Bridge.
  3. **Shell 3**:
     ```bash
     source /opt/ros/foxy/setup.bash
     ```
     - Use this shell for ROS 2 operations.

### Step 4: Set Up `parameter_bridge`
1. Create a ROS 1 workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   catkin_create_pkg load_params
   cd load_params
   mkdir launch params
   touch launch/load_params.launch params/topics.yaml
   ```
2. Add the following code to `launch/load_params.launch`:
   ```xml
   <launch>
     <rosparam command="load" file="/home/omotoye/catkin_ws/src/load_params/params/topics.yaml"/>
   </launch>
   ```
   NB: Change the file path in the rosparam command, run `pwd` to get the path of the file `topics.yaml`
3. Add the following to `params/topics.yaml`:
   ```yaml
   topics:
     [
       { topic: /tf, type: tf2_msgs/msg/TFMessage, queue_size: 10 },
       { topic: /camera/rgb/image_raw/compressed, type: sensor_msgs/msg/CompressedImage, queue_size: 10 },
       { topic: /cmd_vel, type: geometry_msgs/msg/Twist, queue_size: 10 },
       { topic: /odom, type: nav_msgs/msg/Odometry, queue_size: 50 },
       { topic: /scan, type: sensor_msgs/msg/LaserScan, queue_size: 50 },
     ]
   ```
   - Add additional topics as needed with their correct message types.
4. Compile the workspace:
   ```bash
   cd ~/catkin_ws/
   catkin_make
   source devel/setup.bash
   ```
5. Launch the parameter file:
   ```bash
   roslaunch load_params load_params.launch
   ```

### Step 5: Start the Bridge
- Go to the shell with both Noetic and Foxy sourced and run:
  ```bash
  ros2 run ros1_bridge parameter_bridge
  ```

### Step 6: Verify and Test
- In the ROS 2 shell, test functionality:
  - Move the robot:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
  - View camera feed:
    ```bash
    ros2 run rqt_image_view rqt_image_view
    ```
  - Visualize laser scan and TF in RViz2:
    ```bash
    rviz2
    ```

---

## 🛠 Support and Troubleshooting

If you encounter any issues:
- Contact the professor.
- Reach out to the tutors on Teams:
  - Alice Nardelli
  - Omotoye Shamsudeen Adekoya
