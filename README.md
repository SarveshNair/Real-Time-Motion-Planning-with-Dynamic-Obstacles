# Real-Time Motion Planning with Dynamic Obstacles

This project implements **RT-RRT*** (Real-Time Rapidly-exploring Random Tree Star) for motion planning in dynamic environments. The system integrates the **Pure Pursuit Controller** to ensure efficient and adaptive path tracking for differential drive robots. The project is tested and visualized using simulation tools like **Gazebo** and **RViz**.

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Directory Structure](#directory-structure)
6. [Contributors](#contributors)
7. [License](#license)

---

## Introduction
Real-time motion planning is a critical challenge in robotics, particularly in environments with dynamic obstacles. This project addresses the following:
- Efficient path planning using the **RT-RRT*** algorithm.
- Smooth path tracking with the **Pure Pursuit Controller**.
- Robust simulation and visualization in realistic environments.

---

## Features
- **RT-RRT*** Algorithm:
  - Online tree rewiring for dynamic updates.
  - Real-time responsiveness in obstacle-rich scenarios.

- **Pure Pursuit Controller**:
  - Geometric path tracking to minimize errors.
  - Differential drive control using inverse kinematics.

- **Simulation Tools**:
  - **Gazebo** for realistic physics-based testing.
  - **RViz** for 3D visualization of robot states and paths.

---

## Installation

### Prerequisites
1. Python (>= 3.7)
2. ROS (Robot Operating System)
3. Gazebo and RViz
4. Additional Python libraries:
   ```bash
   pip install -r requirements.txt
   ```

### Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/SarveshNair/Real-Time-Motion-Planning-with-Dynamic-Obstacles.git
   cd Real-Time-Motion-Planning-with-Dynamic-Obstacles
   ```

2. Build the ROS workspace:
   ```bash
   catkin_make
   ```

---

## Usage

### Running the Simulation
1. Launch Gazebo and RViz:
   ```bash
   roslaunch motion_planning simulation.launch
   ```

2. Run the RT-RRT* algorithm:
   ```bash
   python src/rtrrt_star.py
   ```

3. Visualize the robot and planned paths in RViz.

### Customizing Parameters
- Modify `config/params.yaml` to adjust:
  - Obstacle density
  - Robot speed and dimensions
  - Simulation environment settings

---

## Directory Structure
```
Real-Time-Motion-Planning-with-Dynamic-Obstacles/
├── src/
│   ├── main.py           # Entry point for simulations
│   ├── pure_pursuit.py   # Pure Pursuit controller implementation
│   ├── rtrrt_star.py     # RT-RRT* algorithm implementation
│   ├── simulation.py     # Simulation setup and controls
│   └── utils.py          # Utility functions
├── config/
│   └── params.yaml       # Configuration file for simulations
├── docs/                 # Documentation and reports
├── README.md             # Project description
├── LICENSE               # License information
└── requirements.txt      # Python dependencies
```

---

## Contributors
- **Kashif Khurshid Noor**: RT-RRT* algorithm implementation
- **Sarvesh Surendran Nair**: Path tracking and evaluation metrics
- **Hrishikesh Nirgude**: Collision checking and trajectory execution

---

