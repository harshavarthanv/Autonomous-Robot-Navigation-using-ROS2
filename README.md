# ENPM809Y - Final Project: Autonomous Robot Navigation using ROS2

## **Project Overview**
This repository contains the ROS2 implementation for **Autonomous Robot Navigation** using ArUco markers and logical cameras. The robot detects **ArUco markers**, retrieves **predefined waypoints**, identifies **parts using logical cameras**, and navigates to **waypoints** using **ROS2 Action Clients**.

---

## **Table of Contents**
- [Features](#features)
- [Installation](#installation)
- [Building the Package](#building-the-package)
- [Running the Project](#running-the-project)
- [Project Structure](#project-structure)

---

## **Features**
✔ Reads **ArUco markers** using ROS2 topic `/aruco_markers`.  
✔ Retrieves **waypoint parameters** from `waypoint_params.yaml`.  
✔ Uses **logical cameras** to detect parts and compute their poses in the map frame.  
✔ Implements a **ROS2 Action Client** for multi-waypoint navigation.  
✔ Utilizes **TF2 transformations** to align world and camera frames.  
✔ Works with **ROS2 Nav2** for path planning and motion execution.  

---

## **Installation**
### **Prerequisites**
Ensure you have the following dependencies installed before running the package:
- **Ubuntu 22.04** (or later)
- **ROS2 Humble**
- **Gazebo** (for simulation)
- **Nav2 (Navigation Stack 2)**
- **ArUco marker detection**
- **TF2 for transformations**
- **Colcon for package compilation**

### **Step 1: Clone the Repository**
```bash
cd ~
mkdir -p ~/final_ws/src
cd ~/final_ws/src
git clone <your-repository-url> group16_final
```
### **Step2: Install Dependencies**
```bash
cd ~/final_ws
rosdep install --from-paths src --ignore-src -r -y
```

### **Step 3: Build the Package**
```bash
cd ~/final_ws
colcon build --packages-select group16_final
source install/setup.bash
```
## **Running the Project**
### **Step 1: Launch the ROS2 Simulation**
```bash
ros2 launch final_project final_project.launch.py
```
### **Step 2: Start the ArUco Marker Detection Node**
```bash
ros2 run group16_final listen
```

### **Step 3: Run the Waypoint Navigation Node**
```bash
ros2 launch group16_final wayp.launch.py
```

## **Project Structure**
```
group16_final/
│── config/
│   ├── waypoint_params.yaml  # Contains predefined waypoints
│
│── include/group16_final/
│   ├── GoalPublisher.hpp     # Header file for goal publishing
│
│── launch/
│   ├── wayp.launch.py        # ROS2 launch file for the navigation node
│
│── src/
│   ├── listen.cpp            # ArUco marker detection and parameter retrieval
│
│── CMakeLists.txt            # ROS2 CMake build configuration
│── package.xml               # Package dependencies and metadata
│── README.md                 # Documentation
```

