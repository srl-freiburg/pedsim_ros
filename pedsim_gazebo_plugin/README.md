# pedsim_gazebo_plugin package

This package integrates the Pedestrian Simulator pedsim_simulator into Gazebo.

### Features
- it converts pedsim scenarios to gazebo worlds 
- it spawns pedsim agents into gazebo  
- it continously updates the poses of the spawned agents

### Sample usage
#### Step 1. Generate gazebo world + launch file for the corresponding pedsim_scenario (scenario_file should be stored in `pedsim_simulator/scenarios/` directory)
====
```
$ rosrun pedsim_gazebo_plugin spawn_pedsim_agents.py
```
#### Step 2. After running the pedsim simulator, use the launch file generated from step 1   
```
$ roslaunch pedsim_gazebo_plugin <scenario_name>.launch
```
#### Step 3. To synchronize your own robot with the diff_robot that is included in the pedsim package: 
##### 1. Spawn your own robot (diff_drive_robot), consider the same coordinates in the scenario file.
##### 2. Remap `/pedbot/control/cmd_vel` to the topic you are using to control your own robot 

