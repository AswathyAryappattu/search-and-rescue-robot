#  Search and Rescue Robot

An autonomous robotic system built on ROS1 that navigates disaster environments in simulation to detect and locate victims using computer vision and sensor-based perception.

---

##  Overview

This project implements a fully autonomous search and rescue robot simulated in Gazebo. The robot explores unknown environments using SLAM-based navigation while simultaneously detecting potential victims through three complementary methods: HSV color detection, optical flow-based motion tracking, and heat signature detection.

---

##  Features

- Autonomous exploration of unknown environments using SLAM and LiDAR
- Victim detection via **HSV color segmentation** (e.g. detecting skin tones or clothing colors)
- **Optical Flow** based motion detection to identify moving survivors
- **Heat signature** detection to locate human body heat
- **Hough Transform** for structural feature detection in the environment
- Real-time visualization in RViz
- Fully simulated in **Gazebo**

---

##  System Requirements

| Requirement | Version |
|---|---|
| OS | Ubuntu 20.04 |
| ROS | ROS1 Noetic |
| Python | 3.8+ |
| Simulator | Gazebo 11 |
| OpenCV | 4.x |

---

##  Installation

**1. Clone the repository into your catkin workspace:**
```bash
cd ~/catkin_ws/src
git clone https://github.com/AswathyAryappattu/search-and-rescue-robot.git
```

**2. Install dependencies:**
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build the workspace:**
```bash
catkin_make
source devel/setup.bash
```

---

## 🚀 How to Run

**Launch the Gazebo simulation:**
```bash
roslaunch search_rescue gazebo.launch
```

**Launch the full search and rescue system:**
```bash
roslaunch search_rescue main.launch
```

**Visualize in RViz:**
```bash
rosrun rviz rviz
```

---

##  Project Structure

```
search_rescue/
├── launch/              # ROS launch files
├── scripts/             # Python nodes
│   ├── hsv_detection.py        # HSV colour-based victim detection
│   ├── optical_flow.py         # Motion tracking via Optical Flow
│   └── heat_detection.py       # Heat signature detection
├── src/                 # C++ nodes
├── config/              # ROS parameters and config files
├── worlds/              # Gazebo world files
└── urdf/                # Robot model files
```

---

## ⚙️ How It Works

### Navigation
The robot uses **GMapping SLAM** with a 2D LiDAR to build an occupancy grid map of the environment in real time. An autonomous exploration node navigates the robot to unexplored regions using frontier-based exploration.

### Victim Detection

| Method | Description |
|---|---|
| HSV Detection | Detects victims by segmenting specific colour ranges (e.g. skin tones, clothing) in the camera feed |
| Optical Flow | Tracks motion between consecutive frames to identify moving survivors |
| Heat Signature | Detects human body heat using thermal camera simulation |
| Hough Transform | Detects structural features and lines in the environment to aid navigation |

---

##  Results

- Successfully detected simulated victims in multiple Gazebo environments
- Stable autonomous navigation with no collisions across all test runs
- All three detection methods operational simultaneously in real time

---

## 🔧 Built With

- [ROS1 Noetic](http://wiki.ros.org/noetic)
- [OpenCV](https://opencv.org/)
- [Gazebo](https://gazebosim.org/)
- [Python 3](https://www.python.org/)

---

##  Author

**Aswathy A S**
[GitHub](https://github.com/AswathyAryappattu)

---

## 📄 Status

✅ Completed
