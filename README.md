# ROSbot Tutorials

Welcome to the **ROSbot Tutorials** repository! This repository contains detailed instructions for working with the ROSbot 2.0 to complete assignment projects and classwork in the **Experimental Robotics** course 🚀.

---

## 📚 Table of Contents

1. [Prerequisites](#prerequisites)
2. [Getting Started](#getting-started)
3. [ROS 2 Users](#ros-2-users)
4. [Connecting via VPN](#connecting-via-vpn)
5. [Support and Troubleshooting](#support-and-troubleshooting)

---

<a name="prerequisites"></a>

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
- **ROS 1 Noetic** and **ROS 2 Foxy** installed on your computer.

> [!IMPORTANT]  
> Ensure your hardware and software setup meets the course's requirements to avoid issues during the project.

---

<a name="getting-started"></a>

## 🚀 Getting Started

### Step 1: Connect to the ROSbot
1. Connect your laptop to the **[FRITZ!Box 6850 XU] WiFi** or the specific WiFi the robots are connected to.
   - **WiFi Password**: `06537900605465042712`
2. Turn on the ROSbot and wait for about **2 minutes** for it to boot up.
3. Copy the **IP Address** written on the robot. This will be referred to as `<ROBOT_IP>`.

> [!NOTE]  
> Ensure your laptop is on the same WiFi network as the ROSbot to establish a connection.

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

> [!CAUTION]  
> Running `roscore` on your laptop while connected to the ROSbot will cause conflicts with the ROSbot's master node.

3. You can now use your ROS workspace on your laptop to control the robot.

### Step 5: Verify and Test

- **Move the robot**:
  ```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard
  ```
- **View camera feed**:
  ```bash
  rosrun image_view image_view image:=/camera/rgb/image_raw _image_transport:=compressed
  ```
- **Visualize laser scan and TF in RViz**:
  ```bash
  rviz
  ```

> [!NOTE]  
> If any of these commands fail due to missing packages, install them using:
> ```bash
> sudo apt install ros-noetic-PACKAGE_NAME
> ```

---

<a name="ros-2-users"></a>

## 🤖 ROS 2 Users

To use ROS 2 to control the robot, follow these additional steps:

### Step 1: Install ROS 2 Foxy
- Follow the instructions [here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) to install ROS 2 Foxy.

### Step 2: Install the `ros1_bridge` Package
Install the `ros1_bridge` package to enable communication between ROS 1 and ROS 2:
```bash
sudo apt install ros-foxy-ros1-bridge
```

### Step 3: Set Up `parameter_bridge`
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
     <rosparam command="load" file="$(find load_params)/params/topics.yaml"/>
   </launch>
   ```

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
> [!IMPORTANT]  
> Ensure the correct message types and queue sizes for your application.

4. Compile the workspace:
   ```bash
   cd ~/catkin_ws/
   catkin_make
   source devel/setup.bash
   ```

### Step 4: Prepare Shells
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

### Step 5: Start the Parameter Server
- In the shell where ROS 1 was sourced, start the parameter server:
  ```bash
  roslaunch load_params load_params.launch
  ```

### Step 6: Start the Bridge
- Run the bridge in the terminal with both Noetic and Foxy sourced:
  ```bash
  ros2 run ros1_bridge parameter_bridge
  ```

---

## 🌐 Connecting via VPN

To access the ROSbot remotely over the internet through a VPN connection, follow these steps:

### Step 1: Request an Authentication Code
1. **Contact the Teaching Assistants (TAs)** to request an authentication code for the VPN.
2. You will receive a personalized auth key that can only be used once and will expire in **7 days** if not used.

### Step 2: Set Up the VPN Connection
> [!NOTE]
> Ensure you are connected to the internet (e.g., via **Eduroam** or any other internet connection) before proceeding. Do **not** connect solely to the FRITZ!Box network, as it does not provide internet access required for the VPN.

1. Open a terminal and set your authentication key:
   ```bash
   TS_AUTH_KEY=<auth_key_from_TAs>
   ```
2. Verify that the auth key is saved successfully:
   ```bash
   echo $TS_AUTH_KEY
   ```
   - Ensure the output matches your provided auth key.

> [!IMPORTANT]
> Do **not** run the setup commands inside a Docker container. This setup is intended for native Ubuntu environments, WSL, virtual machines, or dual-boot systems.

3. Run the following command to install the VPN client and establish the connection:
   ```bash
   curl -fsSL https://tailscale.com/install.sh | sh && sudo tailscale up --auth-key=$TS_AUTH_KEY
   ```

### Step 3: Verify VPN Connection
1. Obtain your VPN IP address:
   ```bash
   hostname -I
   ```
   - Look for an IP address starting with `100.xxx.xxx.xxx`.

### Step 4: Continue with ROSbot Connection
1. **SSH into the ROSbot using its VPN IP**:
   ```bash
   ssh husarion@<ROS_VPN_IP>
   ```
   - **Password**: `husarion`

2. **Verify the connection**:
   - Once connected, you should have access to the ROSbot's terminal over the VPN.

3. **Configure Your Environment**:
   - Set the ROS Master URI to point to the ROSbot's VPN IP:
     ```bash
     export ROS_MASTER_URI=http://<ROS_VPN_IP>:11311
     export ROS_IP=<YOUR_VPN_IP>
     ```
   - Add these environment variables to your `.bashrc` to make them persistent:
     ```bash
     echo "export ROS_MASTER_URI=http://<ROS_VPN_IP>:11311" >> ~/.bashrc
     echo "export ROS_IP=<YOUR_VPN_IP>" >> ~/.bashrc
     source ~/.bashrc
     ```

### Additional Notes:
> [!NOTE]  
> Ensure that your machine remains connected to the internet while using the VPN. Disruptions in your internet connection may affect the VPN stability.

### Troubleshooting
- **Cannot connect to VPN**:
  - Ensure that the VPN client is properly installed and running.
  - Check your internet connection and try restarting the VPN client.
  
- **SSH connection issues**:
  - Verify that the ROSbot is powered on and connected to the VPN.
  - Confirm that you are using the correct VPN IP address.

> [!WARNING]  
> Do **not** share your VPN credentials with others. If you encounter persistent issues, contact the course instructors or TAs.

---

## 🛠 Support and Troubleshooting

If you encounter any issues:
- Contact the professor.
- Reach out to the tutors on Teams:
  - Alice Nardelli
  - Omotoye Shamsudeen Adekoya
- Open an issue on the GitHub repository.

> [!WARNING]  
> Do not attempt to reconfigure the ROSbot's hardware/software unless instructed by a tutor or professor.

---