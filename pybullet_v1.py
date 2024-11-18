import pybullet as p
import pybullet_data

# Connect to the PyBullet physics server
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a simple plane and a robot model
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

# Set gravity for the simulation
p.setGravity(0, 0, -9.81)

# Main simulation loop
while True:
    p.stepSimulation()
