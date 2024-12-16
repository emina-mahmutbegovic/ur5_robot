# ROS UR5 Robot Application

This repository contains a modular ROS application for simulating and controlling a UR5 robot in Gazebo. The application is divided into several packages, each with specific functionality.

---

# Prerequisites
- **ROS Noetic:** Ensure your ROS environment is properly set up.
- **Python3**
- **FLASK:** Required for the Python client.
- **Gazebo:** For visualizing robot movements.

---

# Packages Overview

## 1. **libs**
- Provides essential libraries for:
  - **Publishing Robot State:** Handles the publishing of the robot's current state.
  - **Generating Joint Angles from TCP Points:** Allows the calculation of joint angles based on TCP (Tool Center Point) coordinates.

## 2. **ur5_api**
- **Server Application:** Provides services for interacting with the robot. 
- **Client Application:** A Flask-based Python client for interacting with the server.

### Available Services
- **`MoveJoint.srv`**: Moves the robot between two joint positions.
- **`MoveLinear.srv`**: Moves the robot linearly between two poses.
- **`ReadRobotState.srv`**: Reads the robot's current joint and transform state.

### Usage Examples

#### **MoveJoint.srv**
**Request:**
```json
{
    "point1": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    "point2": [0.6, 0.5, 0.4, 0.3, 0.2, 0.1],
    "velocity": 1.5,
    "acceleration": 0.8
}
```

**Response:**
```json
{
    "joint_state": {
        "header": {
            "stamp": stamp_in_seconds,
            "frame_id": "base_link"
        },
        "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        "actual": {
            "positions": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            "velocities": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            "accelerations": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }
    }
}
```
#### **MoveLinear.srv**
**Request:**
```json
{
    "pose1": [0.1, 0.2, 0.3],
    "pose2": [0.6, 0.5, 0.4],
    "velocity": 1.5,
    "acceleration": 0.8
}
```
**Response:**
```json
{
    "transform": {
        "transforms": [{
            "euler_angles": {"roll": 0.1, "pitch": 0.2, "yaw": 0.3},
            "translation": {"x": 0.1, "y": 0.2, "z": 0.3},
            "rotation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.7}
        }]
    }
}
```

#### **ReadRobotState.srv**
**Request:** (*No parameters required*)

**Response:**
```json
{
    "joint_state": {...},
    "transform": {...}
}
```
### Running the API:
1. **Simulation Setup:**
   - Open a terminal:
     ```bash
     catkin_make
     source devel/setup.bash
     roslaunch ur5_gazebo ur5_eff_controller.launch
     ```
   - Press the **play button** in Gazebo to visualize movements.

2. **Run the Server:**
   - In a new terminal:
     ```bash
     source devel/setup.bash
     roslaunch ur5_api ur5_server.launch
     ```

3. **Run the Client:**
   - In another terminal:
     ```bash
     source devel/setup.bash
     roslaunch ur5_api ur5_client.launch
     ```

4. **Send Requests:**
   - Use Postman or any terminal-based HTTP client.
   - A Postman collection is included in this repository for testing.

## 3. **ur5_gazebo**
Provides Gazebo simulations for the UR5 robot.

### Launch Files:
1. **`spawn_ur5_eff_controller.launch`:** Enables interaction with the robot using MoveIt and RViz.
2. **`ur5_eff_controller.launch`:** Allows external control of the robot (e.g., via `ur5_api`).
3. **`ur5_rviz.launch`:** Enables interaction of the UR5 robot with the Joint State Publisher GUI.

## 4. **ur5_moveit_config**
- Used in combination with the `spawn_ur5_eff_controller.launch` file for robot interaction.
- Available robot positions:
  - **up**
  - **down**
  - **left**
  - **right**

## 5. **ur5_state_publisher**
Provides four test cases for generating joint and linear movements:

### Test Cases:
1. **Joint Movement to One Point:** `ur5_move_joint_1p.launch`
2. **Joint Movement Between Two Points:** `ur5_move_joint_2p.launch`
3. **Linear Movement to One Point:** `ur5_move_linear_1p.launch`
4. **Linear Movement Between Two Points:** `ur5_move_linear_2p.launch`

### Running Tests:
1. **Launch Simulation:**
   - Open a terminal:
     ```bash
     catkin_make
     source devel/setup.bash
     roslaunch ur5_gazebo ur5_eff_controller.launch
     ```

2. **Run Test Cases:**
   - In a new terminal:
     ```bash
     source devel/setup.bash
     roslaunch ur5_state_publisher <test_case>.launch
     ```
     Replace `<test_case>` with:
     - `ur5_move_joint_1p.launch`
     - `ur5_move_joint_2p.launch`
     - `ur5_move_linear_1p.launch`
     - `ur5_move_linear_2p.launch`

     Use simple UI for publishing points/poses.

## Notes
- Ensure all packages are sourced with `source devel/setup.bash` before running any launch files.
- Use the included Postman collection for testing the `ur5_api`.
