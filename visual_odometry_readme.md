# Visual Odometry Handover Package

## 1. Overview
This archive contains the scripts, ROS 2 package, camera calibration files, and recorded dataset for the Monocular Visual Odometry (VO) system. 

The system runs across three devices on the same local network:
1. **Raspberry Pi Zero (`pi_zero_edge`)**: Captures frames from the downward-facing OV9281 camera and streams video over the network.
2. **Ubuntu Host PC (`ubuntu_host_vision`)**: Receives the video stream, tracks floor features using OpenCV (ORB + RANSAC), and sends displacement data via UDP (Port 5006).
3. **NVIDIA Jetson Orin Nano (`jetson_ros2`)**: Receives the UDP data inside the ROS 2 Humble Docker container and publishes it to the `/camera/odom` topic.

---

## 2. Folder Structure

```text
arioviz_handover/
├── calibration_data/
│   ├── calibration_data.npz       # Raw calibration images data
│   └── camera_parameters.npz      # Camera matrix and distortion parameters
├── pi_zero_edge/
│   ├── stream_producer.py         # Main script to stream camera video
│   ├── live_stream.py             # Script to preview camera and check focus/lighting
│   ├── record_manual.py           # Script to record images for calibration
│   └── test_camera.py             # Script to test if the camera is detected
├── ubuntu_host_vision/
│   └── stream_catcher.py          # Main script that calculates movement and sends UDP
├── jetson_ros2/
│   └── sensor_bridge/             # ROS 2 package for receiving UDP data
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       └── sensor_bridge/
│           ├── __init__.py
│           └── vo_udp_bridge.py   # ROS 2 node that publishes /camera/odom
└── odom_take5/
    ├── odom_take5_0.db3           # Recorded rosbag2 file (VO vs Wheel Odometry)
    └── metadata.yaml              # Metadata for opening the bag file in PlotJuggler
```

---

## 3. Requirements
* All three devices (Pi Zero, Ubuntu PC, Jetson) must be connected to the same Wi-Fi/local network.
* Check the IP addresses inside `stream_producer.py` and `stream_catcher.py` and update them if the network IPs have changed.
* Python 3, OpenCV, and NumPy must be installed on the Ubuntu Host PC.

---

## 4. How to Run the System

Please start the scripts in the following order:

### Step 1: Jetson Orin Nano (ROS 2 Receiver)
1. Open the ROS 2 Humble Docker container on the Jetson (make sure UDP port 5006 is accessible or use `--network host`).
2. Copy the `sensor_bridge` folder into your ROS 2 workspace `src` folder and build it:
   ```bash
   cd ~/ros2_ws
   cp -r /path/to/arioviz_handover/jetson_ros2/sensor_bridge src/
   colcon build --packages-select sensor_bridge
   source install/setup.bash
   ```
3. Run the receiver node:
   ```bash
   ros2 run sensor_bridge vo_udp_bridge
   ```

### Step 2: Raspberry Pi Zero (Camera Streamer)
1. Turn on the LED ring light.
2. SSH into the Raspberry Pi Zero and run the stream script:
   ```bash
   cd /path/to/pi_zero_edge
   python3 stream_producer.py
   ```

### Step 3: Ubuntu Host PC (Vision Processing)
1. Make sure `stream_catcher.py` points to the correct path for `camera_parameters.npz` (found in the `calibration_data` folder).
2. Run the processing script:
   ```bash
   cd /path/to/ubuntu_host_vision
   python3 stream_catcher.py
   ```

---

## 5. How to Check the Output and Recorded Data

### Checking Live ROS 2 Data
Inside the Jetson Docker container, open a new terminal and run:
```bash
ros2 topic echo /camera/odom
```

### Viewing the Recorded Bag File (`odom_take5`)
To view the test recording comparing the camera odometry (`/camera/odom`) and wheel odometry (`/odom`):
1. Open PlotJuggler:
   ```bash
   ros2 run plotjuggler plotjuggler
   ```
2. Import `odom_take5/metadata.yaml` (or `odom_take5_0.db3`).
3. Drag `/camera/odom/pose/pose/position/x` and `/odom/pose/pose/position/x` into the graph to compare the movement.