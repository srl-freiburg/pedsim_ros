animated_markers
================

Animated mesh marker visualization plugin and accompanying ROS messages for Rviz

NOTE: This Rviz plugin has only been tested on ROS Hydro, Indigo. It is not guaranteed to work on ROS Groovy.


Motivation
----------
Rviz currently does not play back animations contained in 3D meshes loaded from a ```visualization_msgs/Marker with type MESH_RESOURCE```.
This might be useful for displaying nice visualizations of e.g. walking persons in an environment.

Instead of modifying the source code of the original Rviz ```default_plugin```, the ```animated_marker_rviz_plugin``` replicates part of that code
and adds the required functionality specific to this purpose.

``` 
Usage
-----
The usage of this package is very similar to using ```visualization_msgs/MarkerArray and visualization_msgs/Marker```. Instead, just publish
```animated_marker_msgs/MarkerArray```. The only supported marker type is ```AnimatedMarker.MESH_RESOURCE```.

The animated_marker_rviz_plugin automatically registers itself with Rviz once the package is sourced (type eg. "source devel/setup.sh"
in your catkin workspace). You can then add an ```AnimatedMarkerArray``` display by clicking on the "Add display" button.

Supported animation formats
---------------------------
Currently, only OGRE *.mesh files (along with *.skeleton files) are supported for animation. These can be exported using e.g. 
Easy Ogre Exporter (http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Easy+Ogre+Exporter) from 3DS Max
or Blender Exporter (http://www.ogre3d.org/tikiwiki/Blender+Exporter) from the free Blender software.

Credits
-------
The animated_marker_rviz_plugin code is based upon original code from the default_plugin in ros-visualization/rviz (original authors Dave Hershberger,
David Gossow, Josh Faust). The sample animated human mesh is a free low-poly mesh downloaded from mixamo.com, and the walking animation has been taken
from the Carnegie Mellon Motion Capture Database (or, more specifically, https://sites.google.com/a/cgspeed.com/cgspeed/motion-capture/cmu-bvh-conversion).
