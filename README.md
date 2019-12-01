# Pedestrian Simulator
<img src=https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator/images/crowd1.png width=400/> | <img src=https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator/images/costmap.png width=400/>

ROS packages for a 2D pedestrian simulator based on social force
model of [Helbing et. al](http://arxiv.org/pdf/cond-mat/9805244.pdf). The implementation is based on an extended version of Christian Gloor's [libpedsim](http://pedsim.silmaril.org/) library which has been extended to include additional behaviors and activities. This packages is useful for robot navigation experiments with crowded scenes which are hard to acquire in practice.

### Features
- Individual walking using social force model for very large crowds in real time
- Group walking using the extended social force model
- Social activities simulation
- Sensors simulation (point clouds in robot frame for people and walls)
- XML based scene design
- Extensive visualization using Rviz
- Option to connect with gazebo for physics reasoning

### Requirements
- ROS with the visualization stack (currently tested on `hydro`, `indigo`, `kinetic` ). For melodic, see the branch `melodic-dev`
- C++11 compiler

### Installation

```
cd [workspace]/src
git clone https://github.com/srl-freiburg/pedsim_ros.git  
cd pedsim_ros
git submodule update --init --recursive
cd ../..
catkin build -c  # or catkin_make
```

### Sample usage
```
roslaunch pedsim_simulator simple_pedestrians.launch
```
### Licence
The core `libpedsim` is licensed under LGPL. The ROS integration and extensions are licensed under BSD.

### Developers
* Billy Okal
* Timm Linder


### Contributors
* Dizan Vasquez
* Sven Wehner
* Omar Islas
* Luigi Palmieri

The package is a **work in progress** mainly used in research prototyping. Pull requests and/or issues are highly encouraged.

### Acknowledgements
These packages have been developed in part during the EU FP7 project [SPENCER](spencer.eu)
