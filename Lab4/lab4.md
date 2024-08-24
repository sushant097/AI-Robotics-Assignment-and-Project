
# Lab 4 - Navigating with Potential Fields

## Overview

This folder contains the implementation and analysis for Lab 4, which focuses on robot navigation using potential fields. The lab covers various aspects of potential fields, including the tuning of parameters, the robot's behavior in the presence of obstacles, and a discussion of the advantages and disadvantages of this approach.

## Contents

- **Part A**: Initial Implementation of Potential Fields for Robot Navigation.
- **Part B**: Adjustment of Parameters and Analysis of Robot Behavior.
- **Part C**: Implementation of Different Repulsive Force Profiles.
- **Part D**: Testing and Comparison in Various Environments.
- **Part E**: Discussion on Parameter Tuning, Robot Behavior, and Potential Fields Approach.

## Implementation Details
**The full source code found inside [ai_labs](./ai_labs/)**.


### Part A: Initial Implementation

The initial implementation involves setting up a basic potential fields algorithm where the robot is influenced by attractive forces towards the goal and repulsive forces away from obstacles. This foundational setup allows the robot to navigate towards a target while avoiding collisions.

### Part B: Parameter Adjustment and Behavior Analysis

In this section, the parameters governing the attractive and repulsive forces were adjusted to observe their effects on the robot's navigation. The process involved:

1. **Initial Theoretical Estimates**: Starting with default values based on the robot's specifications and expected operating environment.
2. **Empirical Testing**: Implementing the algorithm with these values and observing the robot's behavior in a controlled environment.
3. **Iterative Adjustments**: Adjusting the parameters iteratively to optimize the robot's performance, ensuring it neither gets too close to obstacles nor fails to move effectively towards the goal.
4. **Balancing Forces**: Achieving a balance between the attractive and repulsive forces, ensuring smooth and efficient navigation.

### Part C: Repulsive Force Profile Implementation

This part explores the impact of using different repulsive force profiles, particularly a linear repulsive force. The linear profile results in:

- **Gradual Obstacle Response**: The robot experiences a more gradual increase in repulsive force as it approaches obstacles, leading to smoother avoidance maneuvers.
- **Proportional Distance Response**: The strength of the repulsion is proportional to the distance from the obstacle, allowing for nuanced obstacle avoidance, especially in environments with varying obstacle densities.

### Part D: Testing in Various Environments

The robot's navigation was tested in different simulated environments (referred to as worlds A, B, and C). The testing revealed how changes in obstacle density, types, and goal distances affected the robot's performance, requiring further adjustments in the calculation of goal and obstacle forces.

### Part E: Discussion

#### 1. Parameter Tuning Process
The process of determining parameter values involved a combination of theoretical estimates, empirical testing, and iterative adjustments. While advanced machine learning techniques might automate this process, the complexity and variability of robot-environment interactions make manual tuning a more straightforward and effective approach in controlled environments.

#### 2. Robot Behavior with Obstacles and Goal
When the robot senses both obstacles and a goal, it balances the attractive and repulsive forces. The attractive force pulls the robot towards the goal, while the repulsive force pushes it away from obstacles, resulting in a path that navigates towards the goal while avoiding collisions.

#### 3. Impact of Linear Repulsive Force Profile
Implementing a linear repulsive force profile leads to smoother and more controlled obstacle avoidance. However, if there is no significant behavioral change, it might be due to the environment not fully challenging the limits of the linear profile or the parameters not being distinct enough.

#### 4. Advantages of Potential Fields Approach
- **Simplicity**: Easy to understand and implement.
- **Real-time Operation**: Suitable for dynamic environments.
- **Local Minima Avoidance**: Properly tuned parameters help avoid local minima.

#### 5. Disadvantages of Potential Fields Approach
- **Local Minima**: The robot can get stuck in positions where opposing forces balance out.
- **Oscillations**: The robot may oscillate in narrow passages or near sharp obstacles.
- **Lack of Path Optimization**: The approach does not inherently optimize for the shortest or quickest path.
- **Parameter Sensitivity**: The approach is highly sensitive to parameter tuning, which can be challenging and environment-specific.

## Conclusion

This lab demonstrates the use of potential fields for robot navigation, highlighting both the strengths and limitations of the approach. The potential fields method is simple and effective in many scenarios but requires careful parameter tuning and can struggle with certain complex environments. The discussion provides insights into the practical considerations when implementing and optimizing potential fields for real-world applications.


**The full solution pdf found in [here](Submission/Submission_report_lab4.pdf).**
