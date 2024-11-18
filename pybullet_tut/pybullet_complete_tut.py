# # === PyBullet Complete Tutorial ===
# Author: Ethan Thornberg
# Parts 1-3: Getting Started, Basic Concepts, and Physics Dynamics
# THIS IS NOT AN EXECUTABLE FILE, DO NOT RUN

import pybullet as p
import pybullet_data
import time
import numpy as np

# === Part 1: Getting Started with PyBullet ===

# 1. Introduction to PyBullet
# PyBullet is an open-source physics engine that allows for the simulation of rigid bodies, soft bodies, and other physics phenomena.
# It is widely used in robotics, AI research, and game development due to its ease of use and real-time physics simulation.

# 2. Installing PyBullet
# To install PyBullet, open your terminal and run:
# pip install pybullet
# Ensure you have Python 3 installed on your system.

# 3. Setting Up Your First PyBullet Simulation
# The first step in using PyBullet is connecting to the physics server, which manages the simulation environment.
p.connect(p.GUI)  # Use p.GUI for graphical mode or p.DIRECT for non-graphical mode

# 4. Understanding the PyBullet Coordinate System
# PyBullet uses a right-handed coordinate system:
# - The x-axis points to the right
# - The y-axis points forward
# - The z-axis points upward
# Units are in meters for length and seconds for time.

# 5. Connecting to the PyBullet Physics Server
# Connecting to the physics server initializes the simulation. Use the following command:
p.connect(p.GUI)

# To run simulations without a graphical interface, use:
# p.connect(p.DIRECT)

# 6. Disconnecting and Cleaning Up
# Always disconnect from the PyBullet server when you are done to free up resources.
p.disconnect()

# === Part 2: Basic Concepts and Object Management ===

# 1. Setting the URDF Search Path
# URDF (Universal Robot Description Format) files describe the structure of robots.
# PyBullet includes a set of default URDF models located in the pybullet_data module.
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 2. Loading Simple Objects (URDF, SDF, and OBJ Files)
# Load a simple ground plane and a robot model.
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])  # Place the robot 1 meter above the ground

# 3. Setting Gravity and Time Steps
# Gravity pulls objects downward. Earth's gravity is approximately -9.81 m/s².
p.setGravity(0, 0, -9.81)  # Set gravity in the negative z direction

# Time steps determine the frequency at which the simulation updates.
p.setTimeStep(1/240)  # Set the time step to 1/240 seconds for high-fidelity simulations

# 4. Understanding Object Properties (Mass, Friction, Inertia)
# You can retrieve and modify the physical properties of objects.
mass, inertia = p.getDynamicsInfo(robot_id, -1)[:2]
print("Mass:", mass)
print("Inertia:", inertia)

# Modify an object's mass and friction
p.changeDynamics(
    bodyUniqueId=robot_id,
    linkIndex=-1,             # The base link of the robot
    mass=2.0,                 # New mass in kilograms
    lateralFriction=0.5       # Friction coefficient (higher values increase friction)
)

# 5. Resetting Object Positions and Orientations
# You can reset the position and orientation of objects to specific values.
p.resetBasePositionAndOrientation(
    robot_id,
    posObj=[0, 0, 1],         # New position in [x, y, z]
    ornObj=[0, 0, 0, 1]       # Orientation as a quaternion [x, y, z, w]
)

# === Part 3: Working with Physics and Dynamics ===

# 1. Applying Forces and Torques
# Forces make objects move, while torques make objects rotate.
# Example: Apply an upward force to the base of the robot
p.applyExternalForce(
    objectUniqueId=robot_id,
    linkIndex=-1,             # Apply force to the base link
    forceObj=[0, 0, 100],     # Force vector in Newtons (100N upward)
    posObj=[0, 0, 0],         # Position where the force is applied
    flags=p.WORLD_FRAME       # Apply the force in the world frame
)

# Example: Apply a torque to make the robot spin around the z-axis
p.applyExternalTorque(
    objectUniqueId=robot_id,
    linkIndex=-1,
    torqueObj=[0, 0, 10],     # Torque vector in Newton-meters
    flags=p.WORLD_FRAME       # Apply the torque in the world frame
)

# 2. Managing Dynamics Properties (Mass, Friction, Restitution)
# Change an object's dynamics properties to affect how it interacts with the environment.
p.changeDynamics(
    bodyUniqueId=plane_id,
    linkIndex=-1,
    restitution=0.9  # High restitution makes the object bounce more
)

# 3. Kinematic vs. Dynamic Objects
# - Kinematic objects are moved manually and are not affected by forces or gravity.
# - Dynamic objects are affected by gravity and forces.
# Example: Make the robot kinematic
p.changeDynamics(
    bodyUniqueId=robot_id,
    linkIndex=-1,
    mass=0  # Setting mass to 0 makes the object kinematic
)

# 4. Simulating Joints and Constraints
# Robots are made up of links connected by joints. You can simulate joints with different types of constraints.
# Example: Print information about each joint in the R2D2 robot
num_joints = p.getNumJoints(robot_id)
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robot_id, joint_index)
    print(f"Joint {joint_index}:")
    print("  Name:", joint_info[1].decode('utf-8'))
    print("  Type:", joint_info[2])

# 5. Controlling Joint Motors and Actuators
# You can control joints using different modes: position, velocity, or torque control.
# Example: Control a joint to move with a specific velocity
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=0,               # Index of the joint to control
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=2.0,         # Target velocity in radians per second
    force=50                    # Maximum force the motor can apply
)

# === End of Parts 1-3 ===

# Remember to run the simulation loop to see the effects of forces, torques, and joint controls.
for _ in range(1000):
    p.stepSimulation()  # Advance the simulation
    time.sleep(1/240)   # Sleep to match the simulation time step

# Always disconnect from the PyBullet server when done
p.disconnect()

# === PyBullet Complete Tutorial ===
# Author: Ethan Thornberg
# Parts 4-6: Advanced Object Manipulation, Sensors and Perception, and Robot Control

import pybullet as p
import pybullet_data
import time

# === Part 4: Advanced Object Manipulation ===

# 1. Creating Visual and Collision Shapes
# PyBullet allows you to create custom objects by defining visual and collision shapes separately.
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.5,
    rgbaColor=[1, 0, 0, 1]  # Red sphere
)
collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.5, 0.5, 0.5]  # Box dimensions
)

# Combine visual and collision shapes into a multi-body object
multi_body_id = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=collision_shape_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[2, 0, 1]  # Position in the simulation
)

# 2. Combining Visual and Collision Shapes
# You can also load objects with built-in URDFs or create them manually for specific use cases.
# Example: Creating a capsule shape
capsule_collision_id = p.createCollisionShape(
    shapeType=p.GEOM_CAPSULE,
    radius=0.2,
    height=1.0
)
capsule_visual_id = p.createVisualShape(
    shapeType=p.GEOM_CAPSULE,
    radius=0.2,
    length=1.0,
    rgbaColor=[0, 1, 0, 1]  # Green capsule
)

# Create a multi-body with the capsule shape
capsule_body_id = p.createMultiBody(
    baseMass=0.5,
    baseCollisionShapeIndex=capsule_collision_id,
    baseVisualShapeIndex=capsule_visual_id,
    basePosition=[3, 0, 1]
)

# 3. Multi-Body Creation and Configuration
# Use multi-bodies to create more complex structures with multiple links and joints.

# 4. Using Soft Bodies (Cloth, Deformable Objects)
# PyBullet supports basic soft body physics, but setup can be complex.
soft_body_id = p.loadSoftBody(
    "cloth_z_up.obj",  # Path to a cloth mesh
    basePosition=[0, 0, 1],
    scale=1.0,
    mass=1.0,
    useNeoHookean=1,   # Use advanced elasticity model
    useBendingSprings=1,
    springElasticStiffness=30,
    springDampingStiffness=0.5
)

# 5. Handling Fluids and Particle Systems
# Note: PyBullet's support for fluids is experimental and not as robust as rigid body simulation.
# You can simulate simple particle systems, but complex fluid dynamics require external libraries.

# === Part 5: Sensors and Perception ===

# 1. Simulating Cameras and Visual Sensors
# PyBullet allows you to create virtual cameras to capture images from the simulation.
# Example: Set up a virtual camera
view_matrix = p.computeViewMatrix(
    cameraEyePosition=[1, 1, 1],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 0, 1]
)
projection_matrix = p.computeProjectionMatrixFOV(
    fov=60,  # Field of view
    aspect=1.0,
    nearVal=0.1,
    farVal=100
)

# Capture an image from the virtual camera
width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
    width=640, height=480,
    viewMatrix=view_matrix,
    projectionMatrix=projection_matrix
)

# 2. Using Ray Casting for Object Detection
# Ray casting helps in detecting objects along a line segment.
ray_start = [0, 0, 2]  # Start of the ray
ray_end = [0, 0, 0]    # End of the ray

# Perform the ray test
hit_info = p.rayTest(ray_start, ray_end)
print("Ray hit information:", hit_info)

# 3. Implementing Force Sensors and Contact Feedback
# Use PyBullet to get information about forces acting on robot joints.
force_sensor_data = p.getJointState(robot_id, 0)[2]  # Retrieve reaction force
print("Force sensor data:", force_sensor_data)

# 4. Adding Gyroscopes and Accelerometers
# PyBullet can simulate basic gyroscope and accelerometer data for robotics applications.
# Example: Simulate accelerometer data by calculating changes in velocity

# === Part 6: Robot Control and Path Planning ===

# 1. Introduction to Robot Control in PyBullet
# Robots in PyBullet are controlled by sending commands to their joints.
# You can use various control modes: position control, velocity control, and torque control.

# 2. Using Position, Velocity, and Torque Control
# Example: Position Control
target_angle = np.pi / 2  # 90 degrees
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=0,
    controlMode=p.POSITION_CONTROL,
    targetPosition=target_angle,
    force=50  # Max force to apply
)

# Example: Velocity Control
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=1,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=1.0,  # Velocity in radians/second
    force=10
)

# Example: Torque Control
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=2,
    controlMode=p.TORQUE_CONTROL,
    force=5  # Torque in Newton-meters
)

# 3. Implementing Inverse Kinematics
# Inverse kinematics (IK) is used to calculate the joint positions required to reach a specific end-effector position.
# Example: Compute IK for a robot arm
target_position = [0.5, 0.5, 0.5]  # Desired end-effector position
joint_positions = p.calculateInverseKinematics(
    bodyUniqueId=robot_id,
    endEffectorLinkIndex=2,  # Index of the end effector
    targetPosition=target_position
)
print("Joint positions for IK:", joint_positions)

# 4. Path Planning Algorithms (A*, RRT)
# While PyBullet doesn't have built-in pathfinding, you can implement algorithms like A* or RRT for robot navigation.

# Example: Placeholder for an A* pathfinding function
def a_star_pathfinding(start, goal, obstacles):
    # Implement your pathfinding logic here
    pass

# 5. Navigation and Obstacle Avoidance
# Use ray tests and collision detection to implement simple obstacle avoidance mechanisms.

# === End of Parts 4-6 ===

# Main Simulation Loop
p.setRealTimeSimulation(0)  # Disable real-time simulation for better control

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()


# === PyBullet Complete Tutorial ===
# Author: Ethan Thornberg
# Parts 7-9: Debugging and Visualization, User Input and Interaction, and Recording Simulations

import pybullet as p
import pybullet_data
import time

# Setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])
p.setGravity(0, 0, -9.81)

# === Part 7: Debugging and Visualization Tools ===

# 1. Using Debug Lines and Text Annotations
# Debug lines and text annotations are helpful for visualizing paths, directions, or key points in the simulation.
start_point = [0, 0, 1]
end_point = [1, 1, 1]
p.addUserDebugLine(
    lineFromXYZ=start_point,
    lineToXYZ=end_point,
    lineColorRGB=[1, 0, 0],  # Red line
    lineWidth=2
)

# Add text annotation to mark a point
p.addUserDebugText(
    text="Target Point",
    textPosition=end_point,
    textColorRGB=[0, 1, 0],  # Green text
    textSize=1.5
)

# 2. Drawing Coordinate Frames and Axes
# Visualize the coordinate frames of objects to better understand their orientations.
origin = [0, 0, 0]
p.addUserDebugLine(origin, [1, 0, 0], [1, 0, 0])  # Red line along the x-axis
p.addUserDebugLine(origin, [0, 1, 0], [0, 1, 0])  # Green line along the y-axis
p.addUserDebugLine(origin, [0, 0, 1], [0, 0, 1])  # Blue line along the z-axis

# 3. Adding User Debug Parameters (Sliders, Buttons)
# PyBullet provides sliders, buttons, and other controls for real-time interaction.
slider_id = p.addUserDebugParameter("Speed", 0, 10, 5)  # Min: 0, Max: 10, Default: 5
speed = p.readUserDebugParameter(slider_id)
print("Speed slider value:", speed)

# Example: Using a slider to control robot movement
while True:
    speed = p.readUserDebugParameter(slider_id)
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=0,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=speed,
        force=10
    )
    p.stepSimulation()
    time.sleep(1/240)

# 4. Monitoring Object State and Debugging Collisions
# Use PyBullet functions to print detailed object states for debugging.
base_position, base_orientation = p.getBasePositionAndOrientation(robot_id)
print("Base Position:", base_position)
print("Base Orientation (quaternion):", base_orientation)

# === Part 8: User Input and Interaction ===

# 1. Capturing Keyboard and Mouse Events
# PyBullet can capture keyboard and mouse events for real-time interaction.
# Example: Checking for keyboard input (use p.B3G_* for specific keys)
key_events = p.getKeyboardEvents()
if p.B3G_LEFT_ARROW in key_events:
    print("Left arrow key pressed!")

# 2. Handling User Input in Real-Time
# Use real-time input to control objects in the simulation.
def handle_input():
    key_events = p.getKeyboardEvents()
    if p.B3G_UP_ARROW in key_events:
        p.applyExternalForce(robot_id, -1, [0, 10, 0], [0, 0, 0], p.LINK_FRAME)
    if p.B3G_DOWN_ARROW in key_events:
        p.applyExternalForce(robot_id, -1, [0, -10, 0], [0, 0, 0], p.LINK_FRAME)

# 3. Creating Interactive Elements (Sliders, Dropdowns)
# Sliders and dropdowns can be used to change simulation parameters on the fly.
slider_force = p.addUserDebugParameter("Force", 0, 100, 50)
slider_angle = p.addUserDebugParameter("Angle", -3.14, 3.14, 0)

# Example: Reading slider values to control the robot
while True:
    force = p.readUserDebugParameter(slider_force)
    angle = p.readUserDebugParameter(slider_angle)
    print("Force:", force, "Angle:", angle)
    # Apply forces or move joints based on slider values
    p.stepSimulation()
    time.sleep(1/240)

# === Part 9: Recording and Saving Simulations ===

# 1. Saving and Loading Simulation States
# Use PyBullet to save and load simulation states, which is useful for debugging and sharing simulations.
state_id = p.saveState()  # Save the current state
p.restoreState(state_id)  # Restore the saved state

# 2. Recording Videos of the Simulation
# You can use PyBullet's built-in functions to record videos of your simulations.
p.startStateLogging(
    loggingType=p.STATE_LOGGING_VIDEO_MP4,
    fileName="simulation_video.mp4"
)

# Run the simulation for a while
for _ in range(240):
    p.stepSimulation()
    time.sleep(1/240)

p.stopStateLogging()  # Stop recording

# 3. Exporting Data for Analysis
# You can export data such as object trajectories and forces for offline analysis.
with open("trajectory_data.txt", "w") as file:
    position, orientation = p.getBasePositionAndOrientation(robot_id)
    file.write(f"Position: {position}, Orientation: {orientation}\n")

# === End of Parts 7-9 ===

# Main Simulation Loop
p.setRealTimeSimulation(0)  # Disable real-time simulation for better control

for _ in range(1000):
    handle_input()  # Handle user input
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()


# === PyBullet Complete Tutorial ===
# Author: Ethan Thornberg
# Parts 10-12: Advanced Constraints, Soft Body Physics, and Real-Time Simulation

import pybullet as p
import pybullet_data
import time

# Setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])
p.setGravity(0, 0, -9.81)

# === Part 10: Advanced Constraints and Joint Control ===

# 1. Creating Fixed Constraints
# Fixed constraints lock two objects together so they move as one.
box_id = p.loadURDF("cube_small.urdf", [1, 1, 1])
constraint_id = p.createConstraint(
    parentBodyUniqueId=robot_id,
    parentLinkIndex=-1,
    childBodyUniqueId=box_id,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],  # No axis of movement
    parentFramePosition=[0.5, 0, 1],  # Position in the parent's frame
    childFramePosition=[0, 0, 0]      # Position in the child's frame
)

# 2. Using Prismatic and Revolute Joints
# Prismatic joints allow linear movement, while revolute joints allow rotation.
# Example: Create a prismatic joint to slide a block along the x-axis
prismatic_constraint = p.createConstraint(
    parentBodyUniqueId=robot_id,
    parentLinkIndex=-1,
    childBodyUniqueId=box_id,
    childLinkIndex=-1,
    jointType=p.JOINT_PRISMATIC,
    jointAxis=[1, 0, 0],  # Movement along the x-axis
    parentFramePosition=[0, 0, 1],
    childFramePosition=[0, 0, 0]
)

# Example: Create a revolute joint to rotate a block around the z-axis
revolute_constraint = p.createConstraint(
    parentBodyUniqueId=robot_id,
    parentLinkIndex=-1,
    childBodyUniqueId=box_id,
    childLinkIndex=-1,
    jointType=p.JOINT_REVOLUTE,
    jointAxis=[0, 0, 1],  # Rotation around the z-axis
    parentFramePosition=[0, 0, 1],
    childFramePosition=[0, 0, 0]
)

# 3. Controlling Joints with Constraints
# Example: Control the prismatic joint position
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=0,
    controlMode=p.POSITION_CONTROL,
    targetPosition=0.2,  # Move the joint to 0.2 meters
    force=10             # Max force
)

# 4. Removing Constraints
# Use p.removeConstraint() to remove an existing constraint
p.removeConstraint(constraint_id)

# === Part 11: Soft Body Physics and Deformable Objects ===

# 1. Introduction to Soft Body Dynamics
# PyBullet can simulate soft body objects like cloth and deformable bodies.
# Note: Soft body physics is computationally expensive and may not run in real-time on all machines.

# 2. Loading Soft Body Models
# Example: Load a soft body cloth model
cloth_id = p.loadSoftBody(
    "cloth_z_up.obj",  # Path to a soft body mesh
    basePosition=[0, 0, 2],  # Initial position of the cloth
    scale=1.0,
    mass=0.5,
    useNeoHookean=1,        # Use advanced elasticity models
    useBendingSprings=1,
    springElasticStiffness=40,
    springDampingStiffness=0.5
)

# 3. Configuring Soft Body Properties
# Adjust the stiffness and damping properties to control how the soft body behaves
p.changeSoftBodyAnchor(
    softBodyUniqueId=cloth_id,
    nodeIndex=0,  # Index of the node to anchor
    bodyUniqueId=robot_id,
    linkIndex=-1  # Attach to the robot base
)

# Example: Change soft body damping
p.changeDynamics(
    cloth_id,
    -1,
    linearDamping=0.1,
    angularDamping=0.1
)

# 4. Collisions with Soft Bodies
# Soft bodies can interact with rigid bodies, causing deformations upon collision.
# Ensure that both the soft body and rigid bodies have proper collision properties configured.

# === Part 12: Real-Time Simulation and Optimization ===

# 1. Understanding Real-Time Simulation
# Real-time simulation aims to run at the same speed as the real world, with a time step of around 1/240 seconds.
p.setRealTimeSimulation(1)  # Enable real-time simulation

# Example: Use real-time simulation to interact with objects
# (Press arrow keys to apply forces)
while p.isConnected():
    key_events = p.getKeyboardEvents()
    if p.B3G_LEFT_ARROW in key_events:
        p.applyExternalForce(robot_id, -1, [-10, 0, 0], [0, 0, 0], p.WORLD_FRAME)
    elif p.B3G_RIGHT_ARROW in key_events:
        p.applyExternalForce(robot_id, -1, [10, 0, 0], [0, 0, 0], p.WORLD_FRAME)

    p.stepSimulation()
    time.sleep(1/240)

# 2. Optimizing Simulation Performance
# Use the following tips to improve simulation performance:
# - Use p.DIRECT mode for headless simulations (without a graphical interface)
# - Lower the simulation resolution or reduce the number of active objects
# - Profile and optimize collision detection and dynamics calculations

# 3. Synchronizing with Real-Time Applications
# If you are integrating PyBullet with a real-time application (e.g., a robotic system),
# ensure that the simulation timing is in sync with the external system.

# Example: Synchronize with a real-time loop
start_time = time.time()
for _ in range(1000):
    elapsed_time = time.time() - start_time
    p.stepSimulation()
    time.sleep(max(0, 1/240 - elapsed_time))  # Adjust sleep time for real-time synchronization

# === End of Parts 10-12 ===

# Disconnect from the simulation
p.disconnect()
