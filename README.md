# Quanser SLAM Navigation Project

## Overview
This project demonstrates SLAM (Simultaneous Localisation and Mapping) using a **Quanser robot** equipped with a depth/color camera and sensors. The goal was to localise the robot using Monte Carlo particle filtering and autonomously navigate through a mapped environment using the A* pathfinding algorithm.

## Hardware
![Quanser Bot](quanser_robot.png)

- **Platform:** Quanser QBot (based on iRobot Create)
- **Sensors:** Depth camera (e.g., Kinect/Xbox 360)
- **Processing:** Onboard Raspberry Pi running Python

## Objectives
- Build a 2D map of the environment
- Identify landmarks and frontiers for navigation
- Localise using particle filter
- Plan and follow an A* path to a user-selected open space

## Key Algorithms
- **Image Processing**: OpenCV-based segmentation for landmark and frontier detection
- **Localisation**: Monte Carlo particle filter for pose estimation
- **Pathfinding**: A* algorithm for generating obstacle-free paths
- **Control**: PID loop for movement correction

## Navigation Output
![Map with A* Path](Map_with_path.png)

- The green dot indicates the bot's pose
- The red line shows the A* path computed to the open goal area

## Repository Structure
```
📁 project-root/
├── quanser_robot.png         # Robot image
├── Map.png                   # Original SLAM map
├── Map_with_path.png         # Map with A* path overlay
├── next_locations.py         # Open space segmentation
├── compare_maps.py           # Landmark detection + map merging
├── particle_filter.py        # Localisation via Monte Carlo method
├── astar_pathfinding.py      # A* pathfinding implementation
├── controller.py             # PID-based movement control
└── README.md                 # This file
```

## Acknowledgements
- Quanser platform and documentation
- OpenCV, NumPy, and Python libraries for development
