# pedsim_gazebo_plugin package

This package integrates the Pedestrian Simulator pedsim_simulator into Gazebo.

### Features
- it converts pedsim scenarios to gazebo worlds 
- it spawns pedsim agents into gazebo  
- it continously updates the poses of the spawned agents

### Sample usage
#### Step 1. Generate gazebo world for the corresponding pedsim_scenario (scenario_file should be stored in `pedsim_simulator/scenarios/` directory)

====
```
python3 scripts/pedsim_to_gazebo_world.py 
```
====
#### Step 2. Launch gazebo world and spawn pedsim agents using the launch file.
```
ros2 launch pedsim_gazebo_plugin gazebo_demo_launch.py 
```
#### Step 3. Customization: to synchronize your own robot with the diff_robot that in the pedsim_simulator: 
====
##### 1. Edit launch  to spawn your own diff_drive_robot, consider the same coordinates in the scenario file.
##### 2. Remap `/pedbot/control/cmd_vel` to the topic you are using to control your own robot 

