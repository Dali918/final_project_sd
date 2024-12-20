# Autonomous Vehicle Final Project
This project will combine the perception, motion planning, and trajectory tracking control modules of the vehicle, and allow you to see the behavior of the autonomous vehicle in the Gazebo environment under this integration. The motion planning and way-point generation modules are already integrated into the code. Your task is to add the perception and control modules, make the necessary connections, and analyze & understand the code as a whole for potential use in your senior design project.

### Components of the Application:

1. **Simulation Environment:**
   - You need a simulation environment to test the performance of the algorithms you developed for your autonomous vehicles. This environment simulates road scenarios and various environmental conditions.

2. **Perception Module:**
   - During the vehicle simulation, captured images are processed by the perception module. This module analyzes the road, segments the lanes, and determines waypoints to be used as references for the trajectory control.

3. **Controller Module:**
   - The controller module directs the vehicle's motion using these references and the actual position and heading of the vehicle. information from the perception module. It adjusts the vehicle's speed and orientation based on the identified waypoints.

## Initialize the Project
To clone the repository, use:
```bash
git clone https://github.com/Dali918/final_project_sd.git
```

## Run Commands
To build packages, In the top folder `final_project_sd` run 

```
colcon build
```

To launch the simulation module,

```
ros2 launch gazebo_project town_world.launch.py
```
To run the perception module. [**TODO**: has to be implemented], but generally
```
ros2 run perception_and_controller perception
```
To run the controller module [**TODO**: has to be implemented], but generally 
```
ros2 run perception_and_controller controller
```
