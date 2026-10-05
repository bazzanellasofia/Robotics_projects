# Perception, Localization and Mapping for Mobile Robots

This repository contains the ROS 2 projects developed for the **Perception Localization and Mapping for Mobile Robots** course. The workspace is divided into two main packages, focusing on fundamental mobile robotics concepts such as odometry computation, coordinate frame transformations (TF), SLAM, and autonomous navigation.

## 📂 Repository Structure

The workspace consists of two main ROS 2 packages:

### 1. First Project (`first_project`)
Focuses on basic mobile robot kinematics and coordinate transforms.
* **`src/odometer.cpp`**: Computes and broadcasts the robot's odometry.
* **`src/tf_error.cpp`**: Analyzes and computes transform (TF) errors.
* **Launch & Visualization**: Run via `first_project.launch.py` and visualized using `first_project.rviz`.

### 2. Second Project (`second_project`)
Focuses on Simultaneous Localization and Mapping (SLAM) and autonomous navigation using the Nav2 stack in a simulated environment.
* **Mapping**: Utilizes `slam_toolbox` (configured via `slam_toolbox_stage.yaml`) to generate a 2D occupancy grid map (`my_final_map`). Managed by `mapping.launch.py`.
* **Navigation**: Autonomous path planning and obstacle avoidance managed by `navigation.launch.py` and `nav2_params.yaml`.
* **`src/csv_controller.cpp`**: A custom C++ node that reads target waypoints from a CSV file (`csv/goals.csv`) and sequentially sends them to the navigation stack.
* **Environment**: Simulation running in `world/stage_world.world`.

## 🛠️ Technologies & Tools
* **Framework:** ROS 2
* **Languages:** C++, Python
* **Tools:** RViz, SLAM Toolbox, Nav2, Stage Simulator

## 🚀 How to Run

1. Clone the repository into your ROS 2 workspace's `src` folder:
   ```bash
   git clone [https://github.com/bazzanellasofia/Robotics_projects.git](https://github.com/bazzanellasofia/Robotics_projects.git)
