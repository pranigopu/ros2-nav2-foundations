<h1>MIDDLEWARE</h1>

---

**Contents**:

- [Definition](#definition)
- [ROS](#ros)

---

# Definition
_A type of software program._

- Provides services to software applications on top of the OS
- Can be described as "software glue"
- Serves as communications interface between machines

# ROS
[ROS 2 uses DDS as its middleware](./dds-in-ros2.md). ROS itself:

- Is a middleware _suite_, i.e.:
    - It utilises middleware functionality at its core
    - Uses this core to provide services to robotics systems
    - Does so via plug-and-play libraries and tools, etc.
- Acts on top of the OS of the computers involved in the system