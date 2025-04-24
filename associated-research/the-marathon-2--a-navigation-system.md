<h1>The Marathon 2: A Navigation System</h1>

**_ROS 2 Nav2-related paper_**

Read the paper [here](./the-marathon-2--a-navigation-system.pdf).

---

**Contents**:

- [Explanation of some terms](#explanation-of-some-terms)
  - [Behaviour tree (BT)](#behaviour-tree-bt)
  - [Node](#node)
  - [Type safety](#type-safety)
  - [Recovery behaviour](#recovery-behaviour)
  - [Sensor modality](#sensor-modality)
  - [Inflation of static map/obstacles](#inflation-of-static-mapobstacles)
  - [Holonomic vs. non-holonomic systems](#holonomic-vs-non-holonomic-systems)
  - [Kalman filter](#kalman-filter)
  - [Adaptive Monte Carlo localisation](#adaptive-monte-carlo-localisation)
- [Key strengths/qualities of Navigation2](#key-strengthsqualities-of-navigation2)
- [Design](#design)
  - [Asynchronous servers](#asynchronous-servers)
  - [Global planner and local planner (controller)](#global-planner-and-local-planner-controller)
  - [Perception](#perception)
  - [State estimation](#state-estimation)
- [Key limitations of Navigation2](#key-limitations-of-navigation2)
  - [Holonomic constraint in path planner](#holonomic-constraint-in-path-planner)
  - [Lack of explicit obstacle detection](#lack-of-explicit-obstacle-detection)

---

**NOTE**:

- Info = Information
- Algo = Algorithm

# Explanation of some terms
## Behaviour tree (BT)
A method to control task flow using a tree data structure.

---

=> Every task fulfils one of the following conditions:

- Leads to a unique subset of potential tasks
- Leads to no further tasks (leaf nodes in the tree)

---

BTs control flow via a hierarchy of conditions, enabling:

- Efficient representation of compound conditions
- Efficient conditional task-switching/task-backtracking
- Interpretable implementation of complex behaviour

---

**RELEVANCE**:

_BT are used as the primary structure of the Navigation framework._

Specifically, Navigation2 uses a configurable BT to orchestrate:

- Planning
- Control
- Recovery tasks

---

**References**:

- [_Behavior Trees in Robotics and AI: An Introduction_ by Michele Colledanchise and Petter Ögren, **arxiv.org**](https://arxiv.org/abs/1709.00084)
- [_Introduction to behavior trees_ by Sebastian Castro, **Robohub.org**](https://robohub.org/introduction-to-behavior-trees/)

## Node
(In ROS 2)

- An independent executable
- Designed for performing a specific modular task

---

**RELEVANCE**:

Core ROS 2 concept.

---

> **Reference**: [_Understanding ROS2 Nodes_ (official ROS2 tutorial documentation)](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

## Type safety
(In a programming language)

An abstract construct enabling avoidance of type errors.

E.g.:

- Compiler will validate types while compiling
- Will throw an error if you try to assign wrong type to a variable

**NOTE**: _Type safety is also ensured at runtime._

---

**RELEVANCE**:

Navigation2 builds upon `BehaviorTree.CPP`:

This is an open source C++ library supporting:

- **Type-safe** asynchronous actions
- Composable behaviour trees
- Logging/profiling infrastructures for development

---

> **References**:
>
> - [_Type Safety in Programming Languages_, **Baeldung.com**](https://www.baeldung.com/cs/type-safety-programming)
> - [_What is Type-safe_, **StackOverflow.com**](https://stackoverflow.com/questions/260626/what-is-type-safe)

## Recovery behaviour
Actions/action-sequences to mitigate complete navigation failures.

---

- By convention, are ordered from conservative to aggressive
- Can be defined for failures in one of the following:
    - A particular area of tasks
    - System failures

---

**RELEVANCE**:

Enables robustness against navigational failures; behaviours used:

- Clear costmap
- Spin
- Wait

## Sensor modality
(Also called a stimulus modality)

An aspect of a stimulus or what is perceived after a stimulus.

I.e.:

An identifiable class of sensation.

I.e.:

A distinct channel or type of sensory info, e.g.:

- Vision
- Hearing
- Touch

---

**RELEVANCE**:

A layered costmap approach is used, which allows:

- User to customise the hierarchy of layers from:
    - Various **sensor modalities**
    - Resolutions
    - Rate limits (rate = sampling rate of possible positions)
- Single costmap to be coherently updated by a number of:
    - Data sources
    - Algorithms

This is key to rich world representations of agent environment.

**NOTE**: _Leverages improvements\* in depth sensing technologies._

\* _In range and resolution._

---

> **References**:
>
> - [_Sensory Modality_, **ScienceDirect.com**](https://www.sciencedirect.com/topics/engineering/sensory-modality)
> - [_12.1C: Sensory Modalities_, **LibreTexts, Health**](https://med.libretexts.org/Bookshelves/Anatomy_and_Physiology/Anatomy_and_Physiology_(Boundless)/12%3A_Peripheral_Nervous_System/12.1%3A_Sensation/12.1C%3A_Sensory_Modalities)

## Inflation of static map/obstacles
- Expanding obstacle coverage beyond their actual coverage
- This is to account for dimensions of a navigation agent

---

**RELEVANCE**:

Helps avoid obstacles without repeated collision checks

=> Efficiency in path planning and long-term navigation

## Holonomic vs. non-holonomic systems
**NOTE**: DoF = Degrees of Freedom, i.e.:

_Set of all movement directions/vectors._

---

**Holonomic**:

Controllable DoF = Total DoF

=> Final state does not depend on path taken to reach it

=> Possible movement is independent of current orientation

=> System can move as a whole in any direction at any time

E.g.: Circular omni-directional robots.

---

**Non-holonomic**:

Controllable DoF < Total DoF

=> Final state depends on path taken to reach it

=> Possible movement is dependent on current orientation

=> System has steering and/or orientation-change constraints

E.g.: Car-like robots.

---

**RELEVANCE**:

(As of 2020)

A\* planner **does not** create feasible paths for robots that are:

- Non-circular
- **Non-holonomic**

Hence, we see the limitation in the Navigation2 A\* implementation.

---

> **References**:
>
> - [_Robot Locomotion_, **RobotPlatform.com**](https://www.robotplatform.com/knowledge/Classification_of_Robots/Holonomic_and_Non-Holonomic_drive.html)
> - [_A brief overview of holonomic systems and their usability in robotics_ by Abhishek Jadhav, **Wevolver.com**](https://www.wevolver.com/article/holonomic-robot)

## Kalman filter
A recursive tracking, estimation and prediction algo that:

- Tracks system-states across time
- Estimates current system-state based on past estimates (if any)
- Predicts future system-state based on uncertainty distribution

**NOTE**: _May implement a multi-sensor fusion framework._

---

**NOTE**:

- Kalman filter is effective for local pose estimation
- However, may not be for global pose estimation <br> _As it assumes Gaussian distribution of uncertainty_\*

\* _Which may not be valid for an environment._

---

**RELEVANCE**:

Localisation tool for local pose estimation.

**NOTE**: _Local => Relative to some starting point_

---

> **References**:
> 
> - [**KalmanFilter.net**](https://www.kalmanfilter.net/default.aspx)
> - [_Introduction to Kalman Filter and Its Applications_ by Youngjoo Kim and Hyochoong Bang](https://www.intechopen.com/chapters/63164)

## Adaptive Monte Carlo localisation
**MONTE CARLO LOCALISATION (MCL)**:

A recursive global pose estimation method that:

- Estimates distribution of uncertainty via samples\*
- Combines past samples with current sensor and actuation data <br> _To obtain probability of the agent having a certain pose_

\* _Samples refer to guesses of the agent's current pose._

Learn about MCL: [Monte Carlo localisation for Mobile Robots](./monte-carlo-localisation-for-mobile-robots.pdf)

---

**ADAPTIVE MCL**:

Adaptive MCL is MCL that adapts number of samples\* over time.

\* _Samples refer to guesses of the agent's current pose._

Learn about adaptive particle filters:

- [KLD Sampling: Adaptive Particle Filters](./kld-sampling--adaptive-particle-filters.pdf)
- [_Adaptive Monte Carlo localisation_, **RoboticsKnowledgebase.com**](https://roboticsknowledgebase.com/wiki/state-estimation/adaptive-monte-carlo-localisation/)

---

**RELEVANCE OF MCL**:

Helps understand AMCL.

**RELEVANCE OF AMCL**:

Primary localisation tool for global pose estimation.

**NOTE**: _Global => Relative to environment map_

# Key strengths/qualities of Navigation2
- Designed for a high-degree of:
    - Configurability
        - Runtime loading of reusable nodes
        - Loading and running of user-defined BTs
        - Ability to choose from a variety of algo.s in runtime
    - Future expansion (i.e. additional algos/functions)
- Intended to support a wide variety of:
    -  Robot types (differential, holonomic, ackermann, legged)
    -  Environments (crowded spaces, offices, warehouses, etc.)
    -  Applications (research, industrial production, etc.)
- Design took into account\* requirements for robotics products:
    - Safety (in real-life usage/application)
    - Security (of data and communications)
    - Determinism (so that behaviours are reliable/predictable)

 \* _Without loss of generality in function._

# Design
## Asynchronous servers
Serves are defined for:

- Recovery
- Planner
- Controller

**NOTE**: _Each may be customised or replaced by the user._

Each server contains:

- Environmental model relevant for its operation
- A network interface to ROS 2 (to communicate with ROS 2 nodes)
- A set of algo plugins to be defined at run-time

---

**NOTE**:

_Plugin = Additional functionality distinct from main program_

Here, plugins refer to interfaces between BT and available algos.

## Global planner and local planner (controller)
**Global planner**:

Compute shortest route to a goal.

**Controller**:

Uses local info to compute:

- Best local path
- Control signals

## Perception
- 3D representation of the environment to project obstacles
- Costmap (2D array) for static obstacle detection and planning

**Costmap**:

Layered costmap is used:

- A single costmap to be coherently updated by a number of:
    - Data sources
    - Algos
- Each layer can extend or modify the costmap it inherits
- Each layer sends the new information to planners and controllers

## State estimation
- Kalman filter for local pose estimation
    - Provides smoothed base odometry from N arbitrary sources:
        - Wheel
        - Multiple IMUs
        - Visual odometry algos
        - etc.
- AMCL for global pose estimation
    - Local pose cannot alone factor in integrated odometric drift
    - Thus global localisation solution remains necessary for nav

_Both the above are localisation methods._

**NOTE**:

- Local pose = Relative to some starting/reference point
- Global pose = Relative to environment map

 # Key limitations of Navigation2
 ## Holonomic constraint in path planner
 (As of 2020)

_Planner using A* expansion and assumes a 2D holonomic particle._

=>

A\* planner does not create feasible paths for robots that are:

- Non-circular
- Non-holonomic

POTENTIAL ADDRESSAL...

Extensions are in development to support robots that are:

- Non-holonomic
- Of arbitrary shape

## Lack of explicit obstacle detection
- No explicit obstacle detection was utilised in experiments
- System was able to successfully navigate without this <br> _In the presence of many dynamic obstacles_

HOWEVER...

Robot operational safely will be further enabled by:

- Explicit obstacle detections
- Predictive models of obstacles (especially dynamic obstacles)