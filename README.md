# `pedsim_ros`

ROS packages for PedSim (Pedestrian Simulator) based on social force
model of [Helbing et. al](http://arxiv.org/pdf/cond-mat/9805244.pdf). The implementation is based on a modified version of Christian Gloor's [libpedsim](http://pedsim.silmaril.org/) library which has been extended to include additional behaviors and activities. All visualization is done via [Rviz](http://wiki.ros.org/rviz).


### Requirements
- ROS ( including visualization rools -> `rviz` )
- C++11 compiler 
- Qt4 
- Eigen3 


### Installation
Clone into you catkin workspace, then run compile;
```
git clone https://github.com/srl-freiburg/pedsim_ros.git
catkin_make
```


### Sample usage
```
roslaunch pedsim_simulator start_simulation.launch 
```

#### TODO
[ ] Make more documentation
[ ] Add additional crowd behaviours


### Developers
* Billy Okal
* Omar Islas

### Contributors
* Dizan Vasquez
* Sven Wehner
* Timm Linder

The package is still a **work in progress** and pull requests are highly
encouraged.


