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
        # Remove the existing robot if it's already loaded
        if 'robot_id' in globals():
            p.removeBody(robot_id)
        
        # Load the robot model from the URDF file
        robot_id = p.loadURDF(robot_urdf_path, [0, 0, 0.1])
        print("Robot model loaded successfully")
    except Exception as e:
        print(f"Error loading URDF: {e}")

# Load the robot initially
load_robot()

# Function to smoothly rotate the robot by 90 degrees
def smooth_rotate_90_degrees(direction):
    # Get the current orientation of the robot as a quaternion
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    
    # Convert the current quaternion to Euler angles
    current_euler = p.getEulerFromQuaternion(orn)
    
    # Calculate the target angle (add or subtract 90 degrees in radians)
    if direction == "left":
        target_yaw = current_euler[2] + math.pi / 2  # 90 degrees left
    elif direction == "right":
        target_yaw = current_euler[2] - math.pi / 2  # 90 degrees right
    else:
        raise ValueError("Invalid direction. Use 'left' or 'right'.")
    
    # Create the target quaternion from the new Euler angles
    target_quaternion = p.getQuaternionFromEuler([current_euler[0], current_euler[1], target_yaw])
    
    # Use Slerp for smooth interpolation between the quaternions
    times = np.array([0, 1])  # Start and end times for interpolation
    rotations = R.from_quat([orn, target_quaternion])
    slerp = Slerp(times, rotations)
    
    # Interpolate the rotation over time
    duration = 2.0  # Duration in seconds for the rotation
    start_time = time.time()
    
    while time.time() - start_time < duration:
        # Calculate the interpolation factor (from 0 to 1)
        t = (time.time() - start_time) / duration
        # Interpolate between the current and target quaternion
        interpolated_quaternion = slerp([t])[0].as_quat()
        
        # Set the new orientation
        p.resetBasePositionAndOrientation(robot_id, pos, interpolated_quaternion)
        
        # Step the simulation
        p.stepSimulation()
        time.sleep(1 / 480)  # Adjust the simulation time step for smoother motion

# Initialize wheel velocities
left_wheel_velocity = 0
right_wheel_velocity = 0
default_speed = 15.0  # Adjust speed as necessary
default_turn_factor = 0.7  # Adjust turning speed as necessary

# Define the correct joint indices for the wheels
front_left_wheel_joint = 0
front_right_wheel_joint = 1
rear_left_wheel_joint = 2
rear_right_wheel_joint = 3

# Main simulation loop
while p.isConnected():
    # Get keyboard events
    keys = p.getKeyboardEvents()

    # Reset velocities
    left_wheel_velocity = 0
    right_wheel_velocity = 0

    # Check for arrow key presses and set wheel velocities
    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        left_wheel_velocity = -default_speed
        right_wheel_velocity = -default_speed

    elif p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        left_wheel_velocity = default_speed
        right_wheel_velocity = default_speed

    # Check for arrow key presses and rotate the robot smoothly
    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        smooth_rotate_90_degrees("left")  # Rotate left

    elif p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        smooth_rotate_90_degrees("right")  # Rotate right

    # Apply velocity to the wheel joints
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=front_left_wheel_joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=left_wheel_velocity)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=rear_left_wheel_joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=left_wheel_velocity)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=front_right_wheel_joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=right_wheel_velocity)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=rear_right_wheel_joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=right_wheel_velocity)


    # Step the simulation
    p.stepSimulation()
    time.sleep(1 / 480)
