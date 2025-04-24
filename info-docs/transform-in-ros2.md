<h1>TRANSFORM (TF)</h1>
---

**Contents**:

- [Definition](#definition)
- [Static vs. dynamic transform](#static-vs-dynamic-transform)
  - [Example](#example)
- [`tf2` library in ROS 2](#tf2-library-in-ros-2)
  - [TF tree](#tf-tree)
- [Example of TFs used in navigation](#example-of-tfs-used-in-navigation)

---

**_In the context of TFs, frame = coordinate frame (CF)_**

---

**References**:

- [_Using tf2 with ROS 2_, **docs.ros.org**](https://docs.ros.org/en/eloquent/Tutorials/tf2.html)
- [_ros2tf_, **mathworks.com**](https://www.mathworks.com/help/ros/ref/ros2tf.html)

# Definition
TF = A geometric relationship between 2 CFs

TF is defined by relationships between:

- Origin points defining reference points
- Axis vectors defining movement dimensions

Hence, a TF consists of:

- Translation (between the CFs' origins)
- Rotation (between the CFs' axes)

---

Different functions may generate and use different TFs, based on:

- Reference point of data collection
- Convenience in computations

E.g.: Odometry's origin pose likely differs from map origin pose.

# Static vs. dynamic transform
A TF is essentially an answer to the question:

_Where is one CF with respect to another CF?_

I.e.:

_How is one CF placed with respect to another CF?_

---

If CF1 moves w.r.t. CF2, TF between them is dynamic.

E.g.: CF1 = Robot base frame, CF2 = Map base frame

---

If CF1 does not move w.r.t. CF2, TF between them is static.

E.g.: CF1 = Robot wheel frame, CF2 = Robot base frame

(_Assuming the robot's structure is fixed_)

## Example
Consider the TFs

- `odom` (odometry) to `base_link` (robot base)
- `base_link` (robot base) to `base_scan` (robot scanner) <br> _Assuming robot's structure is rigid_
- `map` (map) to `odom` (odometry) <br> _Assuming no [pose](./definitions.md#pose) error correction is done_

`odom` gives the position and orientation of the robot:

- Across time
- Relative to some starting point

`base_link` defines the robot's center and its relation its parts.

---

The robot's structure is rigid.

=>

Relation between robot center and scanner position is constant.

=>

`base_link` to `base_scan` is static.

---

`odom` relates robot center to a relative position and orientation.

_This is essentially assigning a position and orientation._

Now, `odom` changes across time.

=> Assigned position and orientation change across time.

=>

`odom` to `base_link` is dynamic.

---

`map` relates robot center to a map coordinate.

`odom` relates robot center to relative coordinate.

=>

- Both give robot position and orientation relative to some origin
- Both deal with the same dimensions (x and y displacements)
- Both origins and orientations are fixed at some starting point

=>

Assuming no pose error correction:

- `map` x-value is a fixed translation of `odom` x-value
- `map` y-value is a fixed translation of `odom` y-value
- `map` orientation is a fixed rotation of `odom` orientation

Hence, given no pose error correction:

`map` to `odom` is static, **_assuming no pose error correction_**.

**NOTE**: _If pose error correction is done, this TF is dynamic._

# `tf2` library in ROS 2
In ROS 2, the `tf2` library is used to handle TFs, i.e.:

- Keep track of multiple CFs over time
- Maintain relationships between the different CFs

## TF tree
`tf2` uses a tree data structure to track and relate CFs.

---

**Enables**:

Transforming geometric features in one CF to another CF.

_(Geometric features include points, lines and vectors)_

# Example of TFs used in navigation
![TF Frames in Navigation - Example](../media/tf-frames.png)

**NOTE**: _Not all TFs would be used/needed in a given use-case._