import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

# Connect to PyBullet and set up the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Set the path to project and assets folder
project_path = os.path.dirname(__file__)
assets_path = os.path.join(project_path, 'assets')

# Load the plane model
plane_id = p.loadURDF("plane.urdf")
p.changeDynamics(plane_id, -1, lateralFriction=20.0)  # Adjust the lateralFriction value as needed

# Set the path to your custom robot URDF
robot_urdf_path = os.path.join(assets_path, "simple_robot.urdf")

# Function to load the robot model
def load_robot():
    global robot_id
    try:
        if 'robot_id' in globals():
            p.removeBody(robot_id)
        robot_id = p.loadURDF(robot_urdf_path, [0, 0, 0.1])
        print("Robot model loaded successfully")
    except Exception as e:
        print(f"Error loading URDF: {e}")

load_robot()

# Function to smoothly rotate the robot by 90 degrees
def smooth_rotate_90_degrees(direction):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    current_euler = p.getEulerFromQuaternion(orn)
    
    if direction == "left":
        target_yaw = current_euler[2] + math.pi / 2
    elif direction == "right":
        target_yaw = current_euler[2] - math.pi / 2
    else:
        raise ValueError("Invalid direction. Use 'left' or 'right'.")
    
    target_quaternion = p.getQuaternionFromEuler([current_euler[0], current_euler[1], target_yaw])
    times = np.array([0, 1])
    rotations = R.from_quat([orn, target_quaternion])
    slerp = Slerp(times, rotations)
    
    duration = 2.0
    start_time = time.time()
    
    while time.time() - start_time < duration:
        t = (time.time() - start_time) / duration
        interpolated_quaternion = slerp([t])[0].as_quat()
        p.resetBasePositionAndOrientation(robot_id, pos, interpolated_quaternion)
        p.stepSimulation()
        time.sleep(1 / 480)

# Variables for smooth movement
current_left_velocity = 0
current_right_velocity = 0
acceleration_rate = 0.1  # Adjust this rate for smoother acceleration/deceleration
default_speed = 15.0
default_turn_factor = 0.7

front_left_wheel_joint = 0
front_right_wheel_joint = 1
rear_left_wheel_joint = 2
rear_right_wheel_joint = 3

# Main simulation loop
while p.isConnected():
    keys = p.getKeyboardEvents()
    target_left_velocity = 0
    target_right_velocity = 0

    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        target_left_velocity = -default_speed
        target_right_velocity = -default_speed
    elif p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        target_left_velocity = default_speed
        target_right_velocity = default_speed

    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        current_left_velocity = 0
        current_right_velocity = 0
        target_left_velocity = 0
        target_right_velocity = 0
        smooth_rotate_90_degrees("left")
    elif p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        current_left_velocity = 0
        current_right_velocity = 0
        target_left_velocity = 0
        target_right_velocity = 0
        smooth_rotate_90_degrees("right")

    # Gradually adjust the current velocities towards the target velocities
    if current_left_velocity < target_left_velocity:
        current_left_velocity += acceleration_rate
    elif current_left_velocity > target_left_velocity:
        current_left_velocity -= acceleration_rate

    if current_right_velocity < target_right_velocity:
        current_right_velocity += acceleration_rate
    elif current_right_velocity > target_right_velocity:
        current_right_velocity -= acceleration_rate

    # Apply the current velocities to the wheel joints
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=front_left_wheel_joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=current_left_velocity)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=rear_left_wheel_joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=current_left_velocity)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=front_right_wheel_joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=current_right_velocity)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=rear_right_wheel_joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=current_right_velocity)

    p.stepSimulation()
    time.sleep(1 / 480)
