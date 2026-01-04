# futuraps_perception

This repository contains a ROS 2 perception package developed as part of the FutuRaPS project.  
The package was implemented in connection with a master’s thesis on perception for autonomous pesticide spraying in raspberry agriculture.

The perception module is designed to operate as a standalone component and provides canopy-related information for downstream spraying control. It processes RGB-D point cloud data and extracts geometric and density-related parameters relevant for different spraying modes.

## Package overview

The package includes functionality for:
- Point cloud filtering and preprocessing
- Canopy geometry and surface normal estimation
- Canopy density and structure analysis
- Visualization and RViz support
- ROS 2 services and messages for parameter extraction

The system is intended to run within a ROS 2 Humble environment and was developed and tested both in simulation and on a mobile robotic spraying platform.

## Build

Clone the repository into the `src` folder of a ROS 2 workspace and build using:

```bash
colcon build --packages-select futuraps_perception
```

## Thesis reference

This package accompanies the project thesis:

*Development of a Standalone Perception Module for Three Different Modes of Pesticide Spraying in Raspberry Agriculture*

