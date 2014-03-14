![](https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator/images/sim_shot.png)

pedsim_ros
==========

ROS packages for PedSim (Pedestrian Simulator) based on social force
model. The implementation is based on a modified version of Christian
Gloor's [libpedsim][1](http://pedsim.silmaril.org/).


## Installation
Clone into you catkin workspace, then run compile;
```
git clone https://github.com/srl-freiburg/pedsim_ros.git
catkin_make
```


## Sample usage
```
roslaunch pedsim_simulator start_simulation.launch 
# See the Rviz window for visual information
```

## TODO
* Make better documentation
* Expose remaining parts of ```libpedsim``` api


## Contributors
* Billy Okal
* Dizan Vasquez
* Luigi Palmieri
* Sven Wehner

The package is still a work in progress and pull requests are highly
encouraged.


