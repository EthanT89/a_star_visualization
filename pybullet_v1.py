import pybullet as p
import pybullet_data
import time

# Connect to PyBullet and set up the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load the plane and R2D2 model
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 0.5])

# Get the number of joints and store joint indices
joint_indices = [i for i in range(p.getNumJoints(robot_id))]
print(joint_indices)

# Print information about each joint
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    print("Joint Index:", joint_info[0])
    print("Joint Name:", joint_info[1].decode("utf-8"))
    print("Joint Type:", joint_info[2])
    print("Joint Lower Limit:", joint_info[8])
    print("Joint Upper Limit:", joint_info[9])
    print()

# Main simulation loop
while p.isConnected():
    keys = p.getKeyboardEvents()

    # Default wheel velocities
    left_wheel_velocity = 0
    right_wheel_velocity = 0
    default_wheel_speed = 20

    # Check for keyboard input to set wheel velocities
    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        # Turn left: slow down the left wheels, speed up the right wheels
        left_wheel_velocity = default_wheel_speed
        right_wheel_velocity = -1 * default_wheel_speed

    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        # Turn right: speed up the left wheels, slow down the right wheels
        left_wheel_velocity = -1 * default_wheel_speed
        right_wheel_velocity = default_wheel_speed

    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        # Move forward: both wheels move forward
        left_wheel_velocity = -1 * default_wheel_speed
        right_wheel_velocity = -1 * default_wheel_speed

    if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        # Move backward: both wheels move backward
        left_wheel_velocity = default_wheel_speed
        right_wheel_velocity = default_wheel_speed

    # Set the wheel velocities using VELOCITY_CONTROL
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=2, controlMode=p.VELOCITY_CONTROL, targetVelocity=right_wheel_velocity)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=3, controlMode=p.VELOCITY_CONTROL, targetVelocity=right_wheel_velocity)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=6, controlMode=p.VELOCITY_CONTROL, targetVelocity=left_wheel_velocity)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=7, controlMode=p.VELOCITY_CONTROL, targetVelocity=left_wheel_velocity)

    # Step the simulation
    p.stepSimulation()
    time.sleep(1 / 240)
