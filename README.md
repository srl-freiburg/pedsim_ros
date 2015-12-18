# `pedsim_ros`

![Crowd](/pedsim_simulator/images/crowd1.png?raw=true "Pedsim Crowd") ![Costmap](/pedsim_simulator/images/costmap.png?raw=true "Pedsim Costmap")


ROS meta package for PedSim (Pedestrian Simulator) based on social force
model of [Helbing et. al](http://arxiv.org/pdf/cond-mat/9805244.pdf). The implementation is based on a modified version of Christian Gloor's [libpedsim](http://pedsim.silmaril.org/) library which has been extended to include additional behaviors and activities. All visualization is done via [Rviz](http://wiki.ros.org/rviz).


### Requirements
- ROS ( including visualization rools -> `rviz` )
- C++11 compiler
- Qt4
- Eigen3

### Dependencies
* Pedsim library, available from [https://github.com/srl-freiburg/pedsim.git](https://github.com/srl-freiburg/pedsim.git)
* Animated markers available from [https://github.com/srl-freiburg/animated_markers](https://github.com/srl-freiburg/animated_markers)
* (Optional) Rviz plugins for tracking and social activity visualization. Installable via our `rviz` fork from [https://github.com/srl-freiburg/rviz](https://github.com/srl-freiburg/rviz).


### Installation
Clone into you catkin workspace, then catkin magic;

```
cd [workspace]/src
git clone https://github.com/srl-freiburg/pedsim.git
git clone https://github.com/srl-freiburg/pedsim_ros.git
git clone https://github.com/srl-freiburg/rviz.git
git clone https://github.com/srl-freiburg/animated_markers.git
cd ..
catkin_make  # or use the awesome catkin build -c
```


### Sample usage
```
roslaunch pedsim_simulator simple_pedestrians.launch
```

#### TODO
- [ ] Make more documentation
- [ ] Add additional crowd behaviours
- [ ] Scenario build tool (GUI)


### Developers
* Billy Okal
* Omar Islas
* Timm Linder


### Contributors
* Dizan Vasquez
* Sven Wehner

The package is still a **work in progress** and pull requests are highly
encouraged.


