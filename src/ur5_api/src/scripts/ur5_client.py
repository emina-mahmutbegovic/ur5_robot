# Copyright (c) 2024 Emina Mahmutbegovic
#
# All rights reserved.

import rospy
from tf.transformations import euler_from_quaternion
from ur5_api.srv import MoveJoint
from ur5_api.srv import MoveLinear
from ur5_api.srv import ReadRobotState
from flask import Flask, request, jsonify

app = Flask(__name__)

rospy.wait_for_service('read_joint_robot_state')
rospy.wait_for_service('read_transform_robot_state')
rospy.wait_for_service('move_joint')
rospy.wait_for_service('move_linear')

# Define service proxies for the ROS service
read_state_service_proxy = rospy.ServiceProxy('read_joint_robot_state', ReadRobotState)
read_transform_state_service_proxy = rospy.ServiceProxy('read_transform_robot_state', ReadRobotState)
move_joint_service_proxy = rospy.ServiceProxy('move_joint', MoveJoint)
move_linear_service_proxy = rospy.ServiceProxy('move_linear', MoveLinear)

def get_joint_state_from_response(response):
    """
    Extracts and constructs the joint state from a ROS service response.

    Args:
        response: The ROS service response containing joint_state.

    Returns:
        A dictionary representing the joint state.
    """
    joint_state = {
        "header": {
            "stamp": response.joint_state.header.stamp.to_sec(),
            "frame_id": response.joint_state.header.frame_id
        },
        "joint_names": response.joint_state.joint_names,
        "actual": {
            "positions": list(response.joint_state.actual.positions),
            "velocities": list(response.joint_state.actual.velocities),
            "accelerations": list(response.joint_state.actual.accelerations)
        },
        "desired": {
            "positions": list(response.joint_state.desired.positions),
            "velocities": list(response.joint_state.desired.velocities),
            "accelerations": list(response.joint_state.desired.accelerations)
        },
        "error": {
            "positions": list(response.joint_state.error.positions),
            "velocities": list(response.joint_state.error.velocities),
            "accelerations": list(response.joint_state.error.accelerations)
        }
    }

    return joint_state

def get_transform_from_response(response):
    """
    Generates a transform dictionary from the response and calculates roll, pitch, and yaw.

    Args:
        response: The ROS service response containing the transform.

    Returns:
        A dictionary representing the transform, including roll, pitch, and yaw.
    """
    # Extract quaternion from the response
    quaternion = (
        response.transform.transforms[0].transform.rotation.x,
        response.transform.transforms[0].transform.rotation.y,
        response.transform.transforms[0].transform.rotation.z,
        response.transform.transforms[0].transform.rotation.w,
    )
    
    # Calculate roll, pitch, and yaw
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    # Build the transform dictionary
    transform = {
        "transforms": [{
            "translation": {
                "x": response.transform.transforms[0].transform.translation.x,
                "y": response.transform.transforms[0].transform.translation.y,
                "z": response.transform.transforms[0].transform.translation.z
            },
            "rotation": {
                "x": response.transform.transforms[0].transform.rotation.x,
                "y": response.transform.transforms[0].transform.rotation.y,
                "z": response.transform.transforms[0].transform.rotation.z,
                "w": response.transform.transforms[0].transform.rotation.w
            },
            "euler_angles": {
                "roll": roll,   
                "pitch": pitch, 
                "yaw": yaw   
            }
        }]
    }

    return transform

# Read current joint state
@app.route('/joint_robot_state', methods=['GET'])
def get_joint_robot_state():
    """Retrieve the current state of the robot."""
    try:
        # Call the ROS service
        response = read_state_service_proxy()

        # Prepare the data for JSON response
        joint_state = get_joint_state_from_response(response)

        return jsonify({
            "joint_state": joint_state,
        }), 200

    except rospy.ServiceException as e:
        return jsonify({"error": "Failed to call ReadRobotState service", "details": str(e)}), 500

# Read current transform state
@app.route('/transform_robot_state', methods=['GET'])
def get_transform_robot_state():
    """Retrieve the current transform state of the robot."""
    try:
        # Call the ROS service
        response = read_transform_state_service_proxy()

        # Prepare the data for JSON response
        transform = get_transform_from_response(response)

        return jsonify({
            "transform": transform
        }), 200

    except rospy.ServiceException as e:
        return jsonify({"error": "Failed to call ReadRobotState service", "details": str(e)}), 500

# Move joint endpoint
@app.route('/move_joint', methods=['POST'])
def move_joint():
    """Generate joint movement of the robot."""
    try:
        # Parse JSON request
        data = request.get_json()

        # Validate input data
        if not all(key in data for key in ('point1', 'point2', 'velocity', 'acceleration')):
            return jsonify({"error": "Missing required fields"}), 400

        point1 = data['point1']
        point2 = data['point2']
        velocity = data['velocity']
        acceleration = data['acceleration']

        if not (isinstance(point1, list) and isinstance(point2, list) and isinstance(velocity, (int, float)) and isinstance(acceleration, (int, float))):
            return jsonify({"error": "Invalid input data types"}), 400

        # Call the ROS service
        response = move_joint_service_proxy(point1, point2, velocity, acceleration)

        # Prepare the data for JSON response
        joint_state = get_joint_state_from_response(response)

        return jsonify({
            "joint_state": joint_state,
        }), 200

    except Exception as e:
        rospy.logerr(f"Error processing move_joint request: {e}")
        return jsonify({"error": str(e)}), 500

# Move linear endpoint
@app.route('/move_linear', methods=['POST'])
def move_linear():
    """Generate linear movement of the robot."""
    try:
        # Parse JSON request
        data = request.get_json()

        # Validate input data
        if not all(key in data for key in ('pose1', 'pose2', 'velocity', 'acceleration')):
            return jsonify({"error": "Missing required fields"}), 400

        pose1 = data['pose1']
        pose2 = data['pose2']
        velocity = data['velocity']
        acceleration = data['acceleration']

        if not (isinstance(pose1, list) and isinstance(pose2, list) and isinstance(velocity, (int, float)) and isinstance(acceleration, (int, float))):
            return jsonify({"error": "Invalid input data types"}), 400

        # Call the ROS service
        response = move_linear_service_proxy(pose1, pose2, velocity, acceleration)

        # Prepare the data for JSON response
        transform = get_transform_from_response(response)

        return jsonify({
            "transform": transform,
        }), 200

    except Exception as e:
        rospy.logerr(f"Error processing move_linear request: {e}")
        return jsonify({"error": str(e)}), 500
if __name__ == '__main__':
    app.run(debug=True, port=5000)
