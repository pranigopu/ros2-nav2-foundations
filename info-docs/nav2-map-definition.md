<h1>NAV2 MAP DEFINITION</h1>

---

**Contents**:

- [What is a map?](#what-is-a-map)
- [Map origin](#map-origin)
- [Map resolution](#map-resolution)
- [Using a custom map in Nav2](#using-a-custom-map-in-nav2)
- [Using a custom environment for Turtlebot3](#using-a-custom-environment-for-turtlebot3)

---

> **Key reference**: [*ROS2 Nav2 Tutorial*, **RoboticsBackend.com**](https://roboticsbackend.com/ros2-nav2-tutorial/)

# What is a map?
A map, in the context of navigation, is a representation of the obstacles and navigation-relevant features (e.g. lanes, stop/slowdown points, etc.) within an environment. This representation can be spatial, numerical (e.g. via numerical grids) or logical (e.g. as sets of vertices and connecting edges). In the current context, when we speak of a map as used by Nav2, we speak of a spatial representation of obstacles, where occupancy information is given in the PGM format and metadata such as resolution and origin (within a coordinate space) is given as parameters within a YAML file.

# Map origin
Map origin is the coordinate (within a defined coordinate space) for the bottom-left corner of the map image. In a 2D PGM image serialised as a list of rows, this is the coordinate for the 1st element of the last row (read from top-to-bottom, left-to-right).

# Map resolution
Map resolution is the number of meters (or, in general, the number of units according to which the coordinate space is defined) corresponding to the length/breadth of a single pixel (note that pixels are always squares). In this context, map resolution is directly proportionate to the map size in terms of the reference units (e.g. meters); the larger the resolution, the larger the represented area (e.g. in meters squared) covered by a single pixel of the map image.

# Using a custom map in Nav2
See [Nav2 for Custom Robot and Custom Map](./nav2-for-custom-robot-and-custom-map.md).

# Using a custom environment for Turtlebot3
See [Gazebo Simulation in Custom Environment using Turtlebot3](./gazebo-simulation-in-custom-environment_using_turtlebot3.md).