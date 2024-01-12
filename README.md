# Automated Guided Vehicle (AGV) Control System

## Overview
This project presents a sophisticated control system for an Automated Guided Vehicle (AGV). It integrates a variety of robotic principles and algorithms to offer a range of functionalities, including odometry, localization, path planning, and line following, all developed within the Robot Operating System (ROS) framework.

### Features
- **Odometry**: Implements motion tracking of the AGV for accurate positioning.
- **Kalman Filter**: Employed for enhanced localization of the vehicle, ensuring precision in navigation.
- **Control to Reference Point**: Enables the AGV to navigate and align with specific reference points.
- **Line Following**: Features an algorithm that allows the AGV to follow a predefined line path.
- **Path Planning using A***: Incorporates the A* algorithm for efficient and optimized path planning.
- **Python and ROS Integration**: Developed in Python, the system communicates effectively with ROS, showcasing seamless integration and operation.

### Technologies Used
- **Programming Language**: Python, a powerful, flexible language known for its readability and ease of use.
- **Framework**: Robot Operating System (ROS), a flexible framework for writing robot software, offering a collection of tools, libraries, and conventions.
- **Hardware**: Raspberry Pi, a compact and affordable computer, connected to our PC using SSH for remote access and control.

## Odometry Equations
The odometry of the AGV is calculated using the following equations:

1. Speed calculation:
   $$ v_s(t) = \frac{v_r(t) + v_l(t)}{2} $$
2. Angular velocity calculation:
   $$ \omega_s(t) = \frac{v_r(t) - v_l(t)}{L} $$
3. Change in angle calculation:
   $$ \dot{\varphi} =\frac{v_s(t)}{\sqrt{R^2(t) + D^2}}=\frac{v_s(t)}{D} \sin(\gamma(t)) =\frac{v_r(t) + v_l(t)}{2D} \sin(\gamma(t)) $$
4. Change in heading calculation:
   $$ \dot{\gamma} =  \omega_s - \dot{\varphi} =   \frac{v_r(t) - v_l(t)}{L} - \frac{v_r(t) + v_l(t)}{2D} \sin(\gamma(t))  $$
5. X-axis movement calculation:
   $$ \dot{x} = v_s(t) \cos(\gamma(t)) \cos(\varphi(t)) =  \frac{v_r(t) + v_l(t)}{2}\cos(\gamma(t)) \cos(\varphi(t))  $$
6. Y-axis movement calculation:
   $$ \dot{y} = v_s(t) \cos(\gamma(t)) \sin(\varphi(t)) =  \frac{v_r(t) + v_l(t)}{2}\cos(\gamma(t)) \sin(\varphi(t)) $$

These equations form the mathematical basis for our AGV's odometry and are crucial for the precise tracking of the vehicle's position and orientation.

---
*Note: The above README is formatted in Markdown and is ready for use in a GitHub project.*
