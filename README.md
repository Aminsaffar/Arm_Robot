# UR3 Robot Pick-and-Place Simulation with Seven-Segment Trajectory Planning

![arm robot pick and place](https://raw.githubusercontent.com/Aminsaffar/Arm_Robot/refs/heads/main/img/pick_and_place.gif "arm robot pick and place")

## Project Overview

This project simulates a UR3 robot performing a pick-and-place task using inverse kinematics (IK) and seven-segment motion planning. The key focus areas are:

- **Kinematic Definitions:** Using transformation matrices $(SE(3))$ for defining the robot’s task-space targets.
- **Inverse Kinematics:** Calculation of joint angles for the end-effector at key positions.
- **Seven-Segment Motion Planning:** Smooth trajectory generation using a kinematic planning model that optimizes acceleration and deceleration phases.
- **PyBullet Simulation:** Visualizing the robot's motion using a physics engine.


---

## Table of Contents

1. [Installation](#installation)
2. [Usage](#usage)
3. [Project Structure](#project-structure)
4. [Implementation Details](#implementation-details)
   - [Inverse Kinematics](#inverse-kinematics)
   - [Seven-Segment Trajectory Planning](#seven-segment-trajectory-planning)
   - [PyBullet Simulation](#pybullet-simulation)
   - [Data Visualization](#data-visualization)
5. [Results](#results)
6. [License](#license)
7. [Acknowledgments](#acknowledgments)

---

## Installation

### Prerequisites

- Python 3.x
- Required libraries: Install the necessary Python packages by running:

```bash
pip install roboticstoolbox-python spatialmath matplotlib numpy pybullet
```

### Cloning the Repository

Clone this repository and navigate into the project directory:

```bash
git clone https://github.com/Aminsaffar/Arm-Robot-Pick-and-Place.git
cd Arm-Robot-Pick-and-Place
```

---

## Usage

To run the project, follow these steps:

1. Ensure that all necessary dependencies are installed.
2. Run the script by executing:

```bash
python Arm-Robot-Pick-and-Place.py
```

3. The program will simulate the UR3 robot's pick-and-place task with a smooth seven-segment trajectory in PyBullet. It will also plot various kinematic properties (e.g., joint angles, velocities, accelerations, and jerks).


---

## Project Structure

```plaintext
.
├── README.md         # This file
├── Arm-Robot-Pick-and-Place.py   # Main Python script for the project
├── plots/            # Folder where generated plots are saved
├── results/          # Folder to store simulation logs or output data
└── URDFs/            # URDF files for the UR3 robot and other objects (table, block)
```

---
## Implementation Details

### Inverse Kinematics

**Goal**: Calculate joint angles that position the end-effector at specific task-space locations.

#### Steps:
1. **Transformation Matrices $(SE(3))$**: Define the key positions using homogeneous transformation matrices $(SE(3))$, which include translation and rotation information for the UR3's end-effector in 3D space.
   
   For each key point $\mathbf{T}$:
```
T = | R  t |
    | 0  1 |
```
   Where:
   - $\mathbf{R}$ is the $3 \times 3$ rotation matrix.
   - $\mathbf{t}$ is the $3 \times 1$ translation vector.

3. **Key Points**: Define transformation matrices for the following key points:
   - Start (home position): $\mathbf{T}_{home}$
   - Pick position: $\mathbf{T}_{pick}$
   - Place position: $\mathbf{T}_{place}$

4. **Inverse Kinematics**: Using the **Levenberg-Marquardt** method (IK_LM), solve the joint angles $\mathbf{q}$ for each target pose:
   $\mathbf{q} = IK(\mathbf{T})$
   where $\mathbf{q}$ is the vector of joint angles $\left[q_1, q_2, \dots, q_6\right]$.

---

### Seven-Segment Trajectory Planning

**Goal**: Generate a smooth trajectory for each joint that consists of acceleration, constant velocity, and deceleration phases.

#### Formulas and Steps:

1. **Motion Phases**: The trajectory is divided into seven segments for each joint.
   - Segment 1–3: Acceleration phase.
   - Segment 4: Constant velocity phase.
   - Segment 5–7: Deceleration phase.

2. **Velocity, Acceleration, and Time**:
   - **Maximum Velocity**: $V_{\text{max}}$
   - **Maximum Acceleration**: $A_{\text{max}}$
   - **Distance between joint positions**: $\Delta q$

3. **Phase Calculations**:
   - Total time for the motion:
     $T = \frac{\Delta q}{V_{\text{max}}}$
   - Time for the acceleration phase $T_{a}$ and deceleration phase $T_{d}$ (assuming symmetric motion):
     $T_a = T_d = \frac{V_{\text{max}}}{A_{\text{max}}}$
   - Time for the constant velocity phase $T_{c}$:
     $T_c = T - 2T_a$
   
4. **Trajectory Generation**:
   - For each phase, position, velocity, and acceleration are computed using these formulas:
     - **Acceleration phase** $(0 \leq t \leq T_a)$:
       
       $q(t) = \frac{1}{2}A_{\text{max}}t^2$
       
     - **Constant velocity phase** $(T_a \leq t \leq T_c)$:
       $q(t) = V_{\text{max}}t + q(T_a)$
     - **Deceleration phase** $(T_c \leq t \leq T)$:
       $q(t) = q(T_c) - \frac{1}{2}A_{\text{max}}(t - T_c)^2$

5. **Interpolation**: After calculating the trajectory for each joint, the positions are interpolated to ensure synchronization between all joints, ensuring that no joint exceeds its maximum velocity or acceleration limits.

---

### PyBullet Simulation

**Goal**: Simulate the robot performing the task and visualize its motion.

#### Steps:

1. **Load URDF Models**: Load the UR3 robot's URDF file and other objects (such as the table and block) into the PyBullet environment.
   
2. **Apply Joint Trajectories**: For each time step, apply the computed joint angles from the seven-segment trajectory to the robot.

3. **Real-time Visualization**: The simulation shows the robot moving smoothly according to the generated trajectory.

---

### Data Visualization

After running the simulation, the following data is plotted:

1. **Joint Angles** $q_i(t)$ over time for all six joints.
2. **Joint Velocities** $\dot{q}_i(t)$, **Accelerations** $\ddot{q}_i(t)$, and **Jerks** $\dddot{q}_i(t)$ showing smooth transitions.
3. **End-Effector Trajectory**: 3D path followed by the robot’s end-effector in Cartesian space.

---

## Results

### Plots

The following graphs are generated by the program and saved in the `plots/` folder:

1. **Joint Angles** during the pick-and-place task.
2. **Joint Velocities, Accelerations, and Jerks** showing smooth transitions between phases.
3. **End-Effector Trajectory** visualized in 3D space.

### PyBullet Simulation

A real-time simulation of the robot’s motion using PyBullet, visualizing the UR3 robot's interaction with the environment.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- **Robotics Toolbox for Python** by Peter Corke, used for kinematic modeling.
- **PyBullet** for providing the physics-based simulation environment.

---
