# ROS 2 Workspace

This repository contains ROS 2 packages developed for learning and projects.

## ROS Version
- ROS 2 Jazzy (Ubuntu 24.04)

---

## 📦 Packages Overview

### 🔹 `actions_cpp`
C++ examples demonstrating ROS 2 **Actions**, including action servers and clients for long-running tasks.

### 🔹 `actions_py`
Python-based ROS 2 **Action** examples showing asynchronous goal handling.

---

### 🔹 `lifecycle_cpp`
C++ nodes using **Managed Lifecycle Nodes**, demonstrating state transitions such as configure, activate, and shutdown.

### 🔹 `lifecycle_py`
Python implementations of **ROS 2 lifecycle nodes** for controlled node execution.

---

### 🔹 `py_pubsub`
Basic **Publisher–Subscriber** examples in Python, used to understand ROS 2 topics and message passing.

### 🔹 `py_srvcli`
Python **Service–Client** examples illustrating synchronous communication in ROS 2.

---

### 🔹 `my_cpp_pkg`
Custom ROS 2 package written in **C++**, containing user-defined nodes and logic for experimentation.

### 🔹 `my_package`
My first ROS2 package.

---

### 🔹 `num_pkg`
Simple number publisher & numerber counter subscriber.
---

### 🔹 `my_robot`
Experiment with ROS2 pkgs, nodes, topics etc.

### 🔹 `my_robot_bringup`
Launch files and configuration required to **launch lifecycle nodes** in lifecycle pkgs.

### 🔹 `my_robot_interfaces`
Custom **message, service, and action definitions** shared across robot packages.

---

### 🔹 `dobo_description`
URDF/Xacro description of the **DOBO robot**, including its mechanical structure and visuals.

### 🔹 `dobo_bringup`
Launch and configuration files to start the **DOBO robot system** in simulation or real hardware.


## Build Instructions
```bash
colcon build
source install/setup.bash


👤 Author

Akhila Wedamestrige
ROS 2 | Robotics | Autonomous Systems
