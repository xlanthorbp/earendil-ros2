# Earendil Bot — Autonomous Rover Simulation

An autonomous rover simulation built with **ROS 2 Humble** and **Gazebo Classic**. This project demonstrates a complete autonomous mission architecture, including GPS-based waypoint navigation, sensor fusion, obstacle avoidance, and computer vision-based ArUco marker tracking. The entire environment is fully containerized using an optimized **Multi-stage Docker** build.

## 🌟 Key Features

*   **Fully Dockerized:** Zero-dependency setup using Docker and NVIDIA Container Toolkit. Uses a highly optimized multi-stage build for fast layer caching and minimal image size.
*   **Sensor Fusion & GPS Navigation:** Dual EKF (UKF) and NavSat for precise global positioning and waypoint navigation.
*   **Computer Vision Autonomy:** State-machine driven ArUco marker tracking for confined space traversal without relying on GPS.
*   **Mission Orchestration:** Python-based coordinator for executing multi-stage tasks sequentially, with interactive waypoint selection driven by YAML configurations.
*   **Open Source:** Licensed under the Apache-2.0 License.

---

## 🛠️ Installation (Ubuntu 22.04 Minimal Setup)

Here are the fastest, minimal instructions for a fresh Ubuntu 22.04 machine to install Docker and the NVIDIA Toolkit, assuming you just want to run the simulation.

### 1. Install Docker Engine
Run this command to download and run Docker's official automated installation script:
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

### 2. Install NVIDIA Container Toolkit
Run these commands to add the NVIDIA repository and install the toolkit so Docker can access the physical graphics card:
```bash
# Add the NVIDIA repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker to use NVIDIA and restart the service
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## 🚀 Running the Simulation

Since you didn't add the user to the Docker group, you need to use `sudo` to build and run the image.

1. **Build the Docker Image:**
   ```bash
   sudo docker build -t earendil_image:latest .
   ```

2. **Run the Launch Script:**
   ```bash
   chmod +x run_docker.sh
   sudo ./run_docker.sh
   ```

3. **Start the Gazebo World & Robot:**
   Once inside the container's terminal, start the simulation. The launch file automatically handles spawning the robot and triggering the localization and Nav2 chains sequentially.
   ```bash
   ros2 launch earendil_bot sim.launch.py
   ```

*(If you are transferring the image via USB instead of building it from source, use `sudo docker load -i earendil_image.tar` before running the launch script).*

---

## 🎮 Executing Missions (Python Nodes)

Once the simulation is running, you can orchestrate the rover's autonomous capabilities using the following Python nodes. Open a new terminal inside the container to run these commands.

### 🤖 Autonomous Mission Coordinator
*   **`ros2 run earendil_bot mission_start`**: The main coordinator script. It executes the full multi-stage mission sequentially (Exit Base -> Select Waypoint -> ArUco Tunnel -> Return Home). It waits for each node to finish successfully before starting the next one.

### 🎯 Individual Mission Nodes
You can also run any mission stage manually to test specific behaviors:
*   **`ros2 run earendil_bot base_exit`**: Commands the rover to drive out of the starting enclosure using LiDAR/Odometry and saves its exit position as the "home" coordinate.
*   **`ros2 run earendil_bot mission_nav`**: An interactive node that loads GPS coordinates from `config/missions.yaml`. It displays a menu allowing you to select a target destination (e.g., Antenna, Crater, Lava Tube) and navigates the rover there using Nav2.
*   **`ros2 run earendil_bot tunnel_aruco_nav`**: Uses OpenCV to detect ArUco markers (ID 100 & 101) at the tunnel gates. It calculates the midpoint between the markers and safely navigates the rover through the confined tunnel without relying on GPS.
*   **`ros2 run earendil_bot base_return`**: Navigates the rover back to the "home" coordinates saved during the `base_exit` phase.

### 🕹️ Manual Teleoperation Nodes
*   **`ros2 run earendil_bot tank_teleop`**: Allows manual control of the rover using the keyboard (W, A, S, D) with a tank-steering control scheme.
*   **`ros2 run earendil_bot joy_teleop`**: Subscribes to joystick inputs to control the rover manually via a physical gamepad or custom Arduino joystick.

### 🛠️ Utility Nodes (Running Automatically)
These nodes are **automatically started** by the `sim.launch.py` script. You do not need to run them manually unless you are debugging:
*   **`scan_ground_filter`**: Subscribes to the 2D LiDAR `/scan` topic and filters out laser hits that strike the ground (due to slopes or craters) based on IMU pitch angles. This prevents Nav2 from falsely perceiving terrain as obstacles.
*   **`pointcloud_ground_filter`**: Subscribes to the 3D depth camera `/camera/points` topic and filters out ground points based on height and IMU orientation, enabling safe 3D obstacle avoidance.
*   **`circle_escape`**: A recovery behavior node designed to maneuver the rover out of enclosed circular spaces if it gets stuck. (Can be triggered manually via `ros2 run earendil_bot circle_escape`).

---

## 📁 Project Structure

```text
├── Dockerfile                  # ROS 2 multi-stage build instructions (optimized for caching)
├── entrypoint.sh               # Sourcing scripts for ROS
├── run_docker.sh               # GPU + X11 passthrough launch script
├── LICENSE                     # Apache-2.0 License
└── src/earendil_bot/           # Main ROS 2 Package
    ├── config/                 # YAML configs (Nav2, EKF, Twist Mux, missions.yaml)
    ├── description/            # URDF, Xacro, and Gazebo sensor definitions
    ├── earendil_bot/           # Python autonomy nodes (CV, Nav, State Machines)
    ├── launch/                 # Launch files (event-based sequencing)
    └── world/                  # Custom Gazebo environment meshes and models
```

## 📜 License

This project is licensed under the [Apache-2.0 License](LICENSE).
