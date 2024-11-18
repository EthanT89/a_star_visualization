import pybullet as p
import pybullet_data
import time
import os

# Connect to PyBullet and set up the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Set the path to project and assets folder
project_path = os.path.dirname(__file__)
assets_path = os.path.join(project_path, 'assets')

# Load the plane model
plane_id = p.loadURDF("plane.urdf")

# Set the path to your custom robot URDF
robot_urdf_path = os.path.join(assets_path, "simple_robot.urdf")

# Initialize last modification time
last_mod_time = os.path.getmtime(robot_urdf_path)

# Function to load the robot model
def load_robot():
    global robot_id
    try:
        # Remove the existing robot if it's already loaded
        if 'robot_id' in globals():
            p.removeBody(robot_id)
        
        # Force PyBullet to reload the URDF by ensuring the file is fully saved
        time.sleep(0.1)  # Small delay to ensure the file has finished saving
        
        # Load the robot model from the URDF file
        robot_id = p.loadURDF(robot_urdf_path, [0, 0, 0.1])
        print("Robot model loaded successfully")
    except Exception as e:
        print(f"Error loading URDF: {e}")

# Load the robot initially
load_robot()

# Main simulation loop
while p.isConnected():
    # Check for changes in the URDF file
    current_mod_time = os.path.getmtime(robot_urdf_path)
    if current_mod_time != last_mod_time:
        last_mod_time = current_mod_time
        # Reload the robot model if the file has changed
        load_robot()
        print("Reloaded robot model due to URDF change")

    # Step the simulation
    p.stepSimulation()
    time.sleep(1 / 240)
