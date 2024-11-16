# Warehouse Pathfinding

A warehouse robot pathfinding simulation using Python, PyBullet, and the A* algorithm.

## Description
This project simulates pathfinding for robots in a warehouse environment. It uses **PyBullet** to create a more advanced and refined simulation of a warehouse, where robots need to find an optimal path from a starting point to an endpoint while avoiding obstacles.

### Features
- **Interactive Warehouse Environment**: Set up the environment with starting points, endpoints, and obstacles.
- **A* Algorithm Visualization**: See the step-by-step execution of the A* pathfinding algorithm in a 3D simulation.
- **Refined Collision and Physics**: Utilizes PyBullet for advanced collision detection and realistic robot dynamics.

## Getting Started
### Prerequisites
To run this project, you need to have Python installed along with the following library:
- **PyBullet**: Install it using the following command:
  ```
  pip install pybullet
  ```

### Installation
1. **Clone this repository**:
   ```
   git clone https://github.com/EthanT89/warehouse-pathfinding.git
   ```

2. **Navigate to the project directory**:
   ```
   cd warehouse-pathfinding
   ```

3. **Install dependencies**:
   ```
   pip install pybullet
   ```

### Running the Project
Run the main Python file to start the PyBullet simulation:
```
python main.py
```

### Controls
- **Environment Setup**: Define start, end, and obstacles using the user interface options provided by the PyBullet environment.
- **Simulation Start**: Run the A* algorithm to see the robot navigate through the warehouse.
- **Reset**: Reset the environment to configure a new scenario.

## How It Works
The project uses the **A* algorithm** for finding the shortest path from a start point to an endpoint within a warehouse environment. The PyBullet library allows for a more sophisticated and realistic simulation, taking into account physics, collisions, and multiple obstacles.

- **Start Point**: Marked in the simulation as a specific colored marker.
- **End Point**: Marked as a target point for the robot.
- **Obstacles**: Represented by physical objects in the warehouse.
- **Pathfinding Steps**: Robots actively navigate through the environment, with real-time path recalculations if necessary.
- **Final Path**: Visualized in the simulation environment as the robot moves to the target.

## Project Structure
- **main.py**: Contains the code to create the simulation, handle user inputs, and implement the A* pathfinding algorithm in the PyBullet environment.
- **README.md**: This file, containing project details.

## Future Improvements
- **Multiple Robot Simulation**: Extend the project to handle multiple robots working simultaneously within the PyBullet environment.
- **Dynamic Obstacles**: Add moving obstacles to simulate real warehouse conditions, with real-time recalculations of the path.
- **Energy Optimization**: Incorporate energy-efficient pathfinding to minimize battery usage for each robot, using the physics engine for accurate energy calculations.

## Contributing
Contributions are welcome! Please feel free to submit a Pull Request or create an issue if you have suggestions for improvement.

## License
This project is licensed under the MIT License. See the LICENSE file for more information.

## Acknowledgements
- Inspired by logistics systems used in real-world warehouses, like Amazon's fulfillment centers.
- Thanks to the open-source community for providing resources and tools to make learning easier.

## Contact
**Author**: Ethan Thornberg
- **Email**: ethan.l.thornberg@gmail.com
- **GitHub**: https://github.com/EthanT89
- **LinkedIn**: https://www.linkedin.com/in/ethan-thornberg/

