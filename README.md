# Pedestrian Simulator
<img src=https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator/images/crowd1.png width=400/> | <img src=https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator/images/costmap.png width=400/>

A ROS meta package for a pedestrian simulator based on social force
model of [Helbing et. al](http://arxiv.org/pdf/cond-mat/9805244.pdf). The implementation is based on a modified version of Christian Gloor's [libpedsim](http://pedsim.silmaril.org/) library which has been extended to include additional behaviors and activities. All visualization is done via [Rviz](http://wiki.ros.org/rviz). The package is useful for robot navigation experiments with crowded scenes which are hard to acquire in practice.

### Features
- Individual walking using social force model for very large crowds in real time
- Group walking using the extended social force model
- Social activities simulation
- Sensors simulation (point clouds in robot frame for people and walls)
- XML based scene design
- Extensive visualization using Rviz
- Option to connect with gazebo for physics reasoning

### Requirements
- ROS with the visualization stack (currently tested on `hydro`, `indigo` )
- C++11 compiler
- Qt4
- Eigen3

### Dependencies
* (**Optional**) Our rviz fork with  additional costmap visualization colors (jet, hot, etc). Installable from [https://github.com/srl-freiburg/rviz](https://github.com/srl-freiburg/rviz).


### Installation

```
cd [workspace]/src
git clone https://github.com/srl-freiburg/pedsim_ros.git
# remaining clones are optional
git clone https://github.com/srl-freiburg/rviz.git
cd ..
catkin build -c
```

### Sample usage
```
roslaunch pedsim_simulator simple_pedestrians.launch
```

#### TODO
- [ ] Add additional crowd behaviours
- [ ] Scenario build tool (GUI)


### Developers
* Billy Okal
* Omar Islas
* Timm Linder


### Contributors
* Dizan Vasquez
* Sven Wehner

The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.


