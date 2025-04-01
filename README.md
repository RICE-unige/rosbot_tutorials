# ROSbot Tutorials

Welcome to the **ROSbot Tutorials** repository! This repository contains detailed instructions for working with the ROSbot 2.0 to complete assignment projects and classwork in the **Experimental Robotics** course 🚀.

---

## 📚 Table of Contents

1. [Prerequisites](#prerequisites)
2. [Getting Started](#getting-started)
3. [ROS 2 Users](#ros-2-users)
4. [Using the ROS2 Humble Bridge (Docker)](#using-the-ros2-humble-bridge-docker)
5. [Connecting via VPN](#connecting-via-vpn)
6. [Support and Troubleshooting](#support-and-troubleshooting)

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

<a name="using-the-ros2-humble-bridge-docker"></a>

## 🐳 Using the ROS2 Humble Bridge (Docker)

For users who wish to leverage **ROS2 Humble** (on Ubuntu 22.04 Jammy) to bridge to a ROS1 Noetic system using Docker, follow these steps:

### Step 1: Build and Extract the ROS1 Bridge Package
1. **Clone the Builder Repository and Build the Image:**
   ```bash
   git clone https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder.git
   cd ros-humble-ros1-bridge-builder
   docker build . -t ros-humble-ros1-bridge-builder
   ```
2. **Extract the Precompiled Bridge:**
   Run the following command on your host (outside any container) to create the `ros-humble-ros1-bridge` folder:
   ```bash
   docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
   ```
> [!NOTE]  
> Remember the absolute path to the extracted folder (e.g., `/home/yourusername/ros-humble-ros1-bridge-builder/ros-humble-ros1-bridge`).

### Step 2: Start a ROS1 Noetic System
You can run ROS1 Noetic in a Docker container (or on your host) using rocker. For example:
```bash
rocker --x11 --user --privileged \
       --volume /dev/shm:/dev/shm --network=host -- \
       ros:noetic-ros-base-focal \
       'bash -c "sudo apt update; sudo apt install -y ros-noetic-rospy-tutorials tilix; tilix"'
```
Inside the ROS1 container, start the ROS Master:
```bash
source /opt/ros/noetic/setup.bash
roscore
```

### Step 3: Run the ROS2 Humble Container with the Bridge Mounted
Launch a ROS2 Humble container with host networking and mount the extracted bridge folder. Replace the path below with your actual absolute path:
```bash
docker run -it --network=host \
  --name ros2_humble \
  -v /home/yourusername/ros-humble-ros1-bridge-builder/ros-humble-ros1-bridge:/root/ros-humble-ros1-bridge \
  osrf/ros:humble-desktop-full
```

### Step 4: Launch the Bridge Inside the ROS2 Container
Once inside the container, set up the environment and run the dynamic bridge:
```bash
source /opt/ros/humble/setup.bash
source /root/ros-humble-ros1-bridge/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
> [!TIP]  
> The `--bridge-all-topics` flag forces bridging for all topics even if no active subscriber exists on one side. This is useful for ensuring topics like `/cmd_vel` are available for teleoperation.

### Step 5: Test the Connection
1. **On the ROS1 Side:**  
   In another terminal within the ROS1 container, run:
   ```bash
   source /opt/ros/noetic/setup.bash
   rosrun rospy_tutorials talker
   ```
2. **On the ROS2 Side:**  
   In a separate terminal within the ROS2 container, run a listener (or your teleoperation node):
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp listener
   ```
   For teleop testing:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Verify that topics (e.g., `/cmd_vel`) published in ROS2 are visible on the ROS1 side using:
   ```bash
   rostopic list
   ```
   and
   ```bash
   rostopic echo /cmd_vel
   ```

---

<a name="connecting-via-vpn"></a>

## 🌐 Connecting via VPN

To access the ROSbot remotely over the internet through a VPN connection, follow these steps:

### Step 1: Request an Authentication Code
1. **Contact the Teaching Assistants (TAs)** to request an authentication code for the VPN.
2. You will receive a personalized auth key that can only be used once and will expire in **7 days** if not used.

### Step 2: Set Up the VPN Connection
> [!NOTE]
> Ensure you are connected to the internet (e.g., via **Eduroam** or any other internet connection) before proceeding. Do **not** connect to the FRITZ!Box network, as it does not provide internet access required for the VPN.

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

### Optional: Using WSLg for VPN Setup

> [!TIP]
> **WSLg (Windows Subsystem for Linux GUI)** allows you to run Linux GUI applications directly on Windows, providing an integrated desktop experience. This is useful if you prefer or need to use Linux tools alongside your Windows environment without setting up a separate virtual machine.

> [!IMPORTANT]
> To use WSLg, ensure you are running **Windows 10** (version 2004 and higher) or **Windows 11**. Make sure your Windows installation is up to date.

#### Step 1: Install WSLg with Ubuntu 20.04
1. **Install WSLg** by running the following command in an elevated PowerShell or Command Prompt:
   ```powershell
   wsl --install -d Ubuntu-20.04
   ```
2. Follow the prompts to complete the Ubuntu 20.04 installation, setting up your Linux username and password.

3. **Launch Ubuntu 20.04** by searching for "Ubuntu 20.04" in the Start Menu and clicking on the application.

> [!NOTE]
> You can also start Ubuntu 20.04 from PowerShell by setting it as the default distro and using the `bash` command:
> 1. Set Ubuntu 20.04 as the default WSL distro:
>    ```powershell
>    wsl -s Ubuntu-20.04
>    ```
> 2. Launch Ubuntu 20.04 by simply entering `bash` in PowerShell:
>    ```powershell
>    bash
>    ```
>    - This will open the Ubuntu 20.04 terminal.

#### Step 2: Proceed with VPN Setup in WSLg
1. Open the Ubuntu terminal from the Start Menu or PowerShell.
2. Follow **Step 2: Set Up the VPN Connection** as described above within the Ubuntu terminal.

> [!IMPORTANT]
> Ensure that you are using WSLg with Ubuntu 20.04 and not running the VPN setup commands inside a Docker container.

---

<a name="support-and-troubleshooting"></a>

## 🛠 Support and Troubleshooting

If you encounter any issues:
- Contact the professor.
- Reach out to the tutors on Teams:
  - Alice Nardelli
  - Omotoye Shamsudeen Adekoya
- Open an issue on the GitHub repository.

> [!WARNING]  
> Do not attempt to reconfigure the ROSbot's hardware/software unless instructed by a tutor or professor.
