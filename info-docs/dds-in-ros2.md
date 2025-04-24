<h1>DDS IN ROS 2</h1>

---

**Contents**:

- [Data distribution service (DDS)](#data-distribution-service-dds)
- [DDS in ROS 2](#dds-in-ros-2)
  - [Changing DDS vendors](#changing-dds-vendors)
  - [ROS middlware (RMW) implementations](#ros-middlware-rmw-implementations)
    - [Using multiple RMW implementations](#using-multiple-rmw-implementations)
    - [Specifying an RMW implementation](#specifying-an-rmw-implementation)
    - [Installing DDS implementations](#installing-dds-implementations)

---

# Data distribution service (DDS)
- DDS is a networking [middleware](./middleware.md)
- PURPOSE: Simplify complex network programming
- IMPLEMENTS: Publish–subscribe pattern for sending and receiving:
    - Data among the nodes
    - Events among the nodes
    - Commands among the nodes

---

Nodes that produce information (publishers):

- Create "topics"; e.g.: _temperature, location, pressure_
- Publish "samples"

In this context, DDS has the following key function:

_Deliver samples to subscribers declaring an interest in the topic._

---

- In essence, DDS is a standard defined for real-time systems
- It is a OMG\* machine-to-machine standard
- PURPOSE: Enable data exchanges that are:
    - Dependable
    - High-performance
    - Interoperable (i.e. are not machine-specific)
    - Real-time
    - Scalable data
- APPROACH: Publish–subscribe pattern

\* _Object Management Group (a standards development organisation)_

---

> **Reference**: [_Data Distribution Service_, **Wikipedia.org**](https://en.wikipedia.org/wiki/Data_Distribution_Service)

# DDS in ROS 2
> **Key references**:
>
> - [_ROS on DDS_ (explains why DDS is a good choice for ROS 2)](https://design.ros2.org/articles/ros_on_dds.html)
> - [_DDS implementations_, **docs.ros.org**](https://docs.ros.org/en/iron/Installation/DDS-Implementations.html)

---

DDS is the middleware used by ROS 2.

_Clearing possible confusion_...

- ROS 2 is sometimes considered as a middleware
- However, the middleware aspect is the core of ROS 2's functions
- ROS 2 itself is a solution that expands on this core
- ROS 2, hence, is more of a **middleware suite**, i.e.:
    - Utilises core middleware functionality
    - Adds libraries and tools, etc. to this core

## Changing DDS vendors
DDS is a set of standards => It must be implemented. Now, note:

- The default DDS vendor is eProsima's Fast DDS
- ROS 2 build will automatically build support for other vendors <br> _Provided they have been installed and sourced correctly_
- The DDS vendor used can be changed at runtime <br> _Provided it has been correctly installed_

## ROS middlware (RMW) implementations
> **Key reference**: [_Working with multiple ROS 2 middleware implementations_, **docs.ros.org**](https://docs.ros.org/en/iron/How-To-Guides/Working-with-multiple-RMW-implementations.html)

### Using multiple RMW implementations
To have multiple RMW implementations available for use, you must have either (1) installed the ROS 2 binaries and any additional dependencies _tailored for specific RMW implementations_, or (2) built ROS 2 from source with multiple RMW implementations in the workspace; in this case, the RMW implementations are included in the ROS 2 build by default if their compile-time dependencies are met.

### Specifying an RMW implementation
Both C++ and Python nodes support an environment variable `RMW_IMPLEMENTATION` that allows the user to select the RMW implementation to use when running ROS 2 applications. The user may set this variable to a specific implementation identifier, such as `rmw_cyclonedds_cpp`, `rmw_fastrtps_cpp`, `rmw_connextdds`, or `rmw_gurumdds_cpp`.

### Installing DDS implementations
See [_DDS implementations_, **docs.ros.org**](https://docs.ros.org/en/iron/Installation/DDS-Implementations.html)