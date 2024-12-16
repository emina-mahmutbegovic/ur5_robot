# ROS UR5 Robot Application

This repository contains a modular ROS application for simulating and controlling a UR5 robot in Gazebo. The application is divided into several packages, each with specific functionality.

---
# Basic Usage
1. Clone the repository via
```bash
git clone git@github.com:emina-mahmutbegovic/ur5_robot.git
```
2. Navigate to the cloned application:
```bash
cd ur5_robot
```
3. Build the workspace:
```
catkin_make
```
4. Source the workspace:
```
source devel/setup.bash
```

---

# Prerequisites
- **ROS Noetic/Melodic:** Ensure your ROS environment is properly set up. [ROS Installation Guideline](http://wiki.ros.org/ROS/Installation)
- **Universal Robots**:
```bash
sudo apt install ros-<ros-distro>-universal-robots-description
```
- **Python3.8: [Installation Docs](https://www.python.org/downloads/)**
- **Pip3**: 
```bash
sudo apt-get install python3-pip
```

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

   Install python requirements:
   ```bash
   pip3 install -r src/ur5_api/src/scripts/requirements.txt
   ```
   Run launch file:
     ```bash
     source devel/setup.bash
     roslaunch ur5_api ur5_client.launch
     ```
Once the application is running, the Flask API can be accessed at `http://localhost:5000`.

4. **Send Requests:**
   - Use Postman or any terminal-based HTTP client.
   - A Postman collection is included in this repository for testing.

## 3. **ur5_api_copilot**

### Overview
`ur5_api_copilot` is a Catkin package that provides a simple Flask-based API interface utilizing the **GEMINI AI** large language model (LLM) via the **LangChain** library. The application exposes two main endpoints:

### Endpoints
1. **`/suggest_json`**
   - **Description**: Takes a ROS `.srv` file as a prompt message and returns a JSON request body suggestion.

2. **`/explain_route`**
   - **Description**: Takes an API route and returns an explanation of the route.

### Prerequisite
To use this API, you must provide a valid **Google API Key**.
- You can generate a free API key via [Google AI Pricing](https://ai.google.dev/pricing#1_5flash).

### Setting the API Key
1. **Environment Variable**:
   ```bash
   export GOOGLE_API_KEY="your-google-api-key"
   ```
   
2. **Prompt Input**:
   If the environment variable is not set, you will be prompted to enter the API key when running the application.

---

### Installation
#### Note: 
This application requires **python>=3.9**. To ensure that everything works properly, setup virtual environment first:
```bash
cd src/ur5_api_copilot/scripts
python3 -m venv venv
source venv/bin/activate
```
Install Python3.9:
```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y python3.9 python3.9-distutils python3.9-venv
```
Update the alternatives system:
```bash
sudo update-alternatives --config python3
```
Select the proper Python version.
Ensure pip3 is present within the virtual environment. If any problems occur, run:
```bash
python3 get-pip.py
```

#### Step 1: Install Dependencies
Install Python dependencies from the `requirements.txt` file:
```bash
pip3 install -r src/ur5_api_copilot/scripts/requirements.txt
```

#### Step 2: Build the Catkin Workspace
Build the workspace using `catkin_make`:
```bash
catkin_make
```

#### Step 3: Source the Workspace
Source the setup file to ensure ROS can find the package:
```bash
source devel/setup.bash
```

#### Step 4: Launch the Application
Launch the Flask API using ROS:
```bash
rosrun ur5_api_copilot ur5_api_copilot.py
```
---

### Usage

Once the application is running, the Flask API can be accessed at `http://localhost:6000`. Use the following endpoints:

#### 1. **`/suggest_json`**
- **Method**: `POST`
- **Description**: Takes a ROS `.srv` file as input and returns a JSON request body suggestion.
- **Example Request**:
  ```bash
  curl -X POST -H "Content-Type: application/json" -d '{"prompt": "# MoveJoint.srv\nfloat64[] point1\nfloat64[] point2\nfloat64 velocity\nfloat64 acceleration\n---\ncontrol_msgs/JointTrajectoryControllerState joint_state"}' http://localhost:6000/suggest_json
  ```

#### 2. **`/explain_route`**
- **Method**: `POST`
- **Description**: Takes an API route as input and returns an explanation of its purpose.
- **Example Request**:
  ```bash
  curl -X POST -H "Content-Type: application/json" -d '{"prompt": "/suggest_json"}' http://localhost:6000/explain_route
  ```

## 4. **ur5_gazebo**
Provides Gazebo simulations for the UR5 robot.

### Launch Files:
1. **`spawn_ur5_eff_controller.launch`:** Enables interaction with the robot using MoveIt and RViz.
2. **`ur5_eff_controller.launch`:** Allows external control of the robot (e.g., via `ur5_api`).
3. **`ur5_rviz.launch`:** Enables interaction of the UR5 robot with the Joint State Publisher GUI.

## 5. **ur5_moveit_config**
- Used in combination with the `spawn_ur5_eff_controller.launch` file for robot interaction.
- Available robot positions:
  - **up**
  - **down**
  - **left**
  - **right**

## 6. **ur5_state_publisher**
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
