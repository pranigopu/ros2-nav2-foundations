<h1>ROS 2 CORE CONCEPTS</h1>

---

**Contents**:

- [Topic](#topic)
- [Service](#service)
- [Action](#action)

---

# Topic
**Background**:

- ROS 2 breaks complex systems down into many modular nodes
- Topics act as a bus for nodes to exchange messages
- Hence, they are a vital element of the ROS graph

---

A topic is a programmatically-defined many-to-many data-stream channel.

=> *Is a method of communication between nodes.*

***Communication based on publisher-subscriber model.***

---

**Illustration**:

![](../media/publisher-subscriber-architecture-illustration-for-topics.gif)

> **Source**: *See reference below*

---

> **Reference**: [*Understanding topics*, **docs.ros.org**](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

# Service
Provide data when they are specifically called by a client.

=> *Is a method of communication between nodes.*

***Communication based on call-and-response model.***

**NOTE**:

- There can be only one service server per service
- However, there can be many service clients calling a service <br> => It enables 1-to-many communication, unlike topics

---

**Services vs. topics**:

| Service | Topic | 
| --- | --- |
| Call-response model | Publisher-subscriber model |
| Provide data when called by client | Allow nodes to <ol><li>Subscribe to data streams</li> <li>Get continual updates</li> |

---

**Illustration**:

![](../media/call-response-architecture-illustration-for-services.gif)

> **Source**: *See reference below*

---

> **Reference**: [*Understanding services*, **docs.ros.org**](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

# Action
- One of the communication types in ROS 2
- Intended for long running tasks
- Consist of three parts:
    - Goal (i.e. expected functionality)
    - Feedback
    - Result

---

**Actions are built on topics and services**:

- Their functionality is similar to services
- However, unlike services actions are preemptable <br> I.e.: *You can cancel them while executing*
- They also provide steady feedback, as opposed to services <br> *Services only return single response*

---

**Architecture**:

- Actions use a client-server model, <br> *Similar to the publisher-subscriber model described for topics*
- An "action client" node sends a goal to an "action server" node - The action server node:
    - Acknowledges the goal
    - Returns a stream of feedback and a result

---

**Illustration**:

![](../media/action-architecture-illustration.gif)

> **Source**: *See reference below*

---

> **Reference**: [*Understanding actions*, **docs.ros.org**](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)