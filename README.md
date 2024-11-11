```
ARC/
|-- .vscode/                     # Editor configuration files
|-- build/                       # Compiled outputs
|-- devel/                       # Development workspace
|-- install/                     # Installation files
|-- src/                         # Source code directory
    |-- gazebo_env/              # Gazebo simulation environment
        |-- gazebo_models/       # Gazebo models
        |-- launch/              # Launch files for Gazebo & Jackal
        |-- worlds/              # World files for simulation includes maps and worlds
    |-- jackal/                  # Jackal robot configurations and control
        |-- jackal_control/      # Control configurations
        |-- jackal_description/  # Robot description files
        |-- jackal_navigation/   # Navigation configurations
            |-- launch/          # Navigation launch files
            |-- maps/            # Map files for navigation
            |-- params/          # Navigation parameters
    |-- LeGO-LOAM/               # LiDAR Odometry and Mapping
    |-- planning/                # Planning algorithms and configurations
        |-- config/              # Planning configurations
        |-- include/planning/    # Header files for planners
            |-- astar_planner.h
            |-- rrtstar_planner.h
        |-- launch/              # Planning launch files
            |-- jackal_nav.launch
            |-- jackal_slam_nav.launch
        |-- scripts/             # Python scripts for planning
            |-- pub_goal.py
        |-- src/                 # Source code for planning algorithms
            |-- astar_planner.cpp
            |-- rrtstar_planner.cpp
        |-- planner_plugins.xml  # Plugin configurations for planners
    |-- pointcloud_to_laserscan-lunar-devel/  # Pointcloud to laser scan conversion
    |-- scripts/                 # Miscellaneous scripts
        |-- error_analysis.py    # Script for error analysis
        |-- path_analysis.py     # Script for path analysis
|-- .catkin_workspace            # ROS Catkin workspace identifier
|-- CMakeLists.txt               # CMake build configuration file
|-- package.xml                  # ROS package manifest
```
