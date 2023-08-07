#!/usr/bin/env python3
"""
Modified by: stephenadhi
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""
import os
import math
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

global gzb_world
def generate_pose(x1, y1, x2, y2):
    if x2-x1==0:
        gzb_world.write("\t<pose frame=''>{} {}  1.4 0 0 {}</pose>\n".format( (x2+x1)/2, (y2+y1)/2, 1.57 ))#x1, y1, math.atan((y2-y1)/(x2-x1)) )
    else:
        gzb_world.write("\t<pose frame=''>{} {}  1.4 0 0 {}</pose>\n".format( (x2+x1)/2, (y2+y1)/2, math.atan( (y2-y1)/(x2-x1) )  ))#x1, y1, math.atan((y2-y1)/(x2-x1)) )

def generate_size(x1, y1, x2, y2):
    l = math.sqrt( pow(y2-y1,2) + pow(x2-x1,2)  )
    gzb_world.write("\t<size> {} .2  2.8 </size>\n".format( l ))

def generate_size(x1, y1, x2, y2):
    l = math.sqrt( pow(y2-y1,2) + pow(x2-x1,2)  )
    gzb_world.write("\t<size> {} .2  2.8 </size>\n".format( l ))

def generate_obstacle(x1, y1, x2, y2, idx):
    gzb_world.write('''
    <model name='grey_wall_{}'>
      <static>1</static>
      <link name='link'>
    '''.format(idx))
    generate_pose(x1, y1, x2, y2)     
    gzb_world.write('''    
    <collision name='collision'>
      <geometry>
        <box>
        ''')
    generate_size(x1, y1, x2, y2)
    gzb_world.write(
    '''\t\t</box>
        </geometry>
        <max_contacts>10</max_contacts>
        <surface>
        <contact>
            <ode/>
        </contact>
        <bounce/>
        <friction>
            <torsional>
            <ode/>
            </torsional>
            <ode/>
        </friction>
        </surface>
    </collision>
    <visual name='visual'>
        <cast_shadows>0</cast_shadows>
        <geometry>
          <box>
        '''.format( 1,1))
    generate_size(x1, y1, x2, y2)
    gzb_world.write(
    '''\t\t</box>
    </geometry>
        <material>
            <script>
            <uri>model://grey_wall/materials/scripts</uri>
            <uri>model://grey_wall/materials/textures</uri>
            <name>vrc/grey_wall</name>
            </script>
        </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
    </link>
    </model>
    '''.format( 1,1))

def parseXML(xmlfile): 
    tree = ET.parse(xmlfile) 
    root = tree.getroot() 
    idx = 0
    for item in root:
        if item.tag == 'obstacle':
            idx= idx+1
            x1= item.attrib['x1']
            y1= item.attrib['y1']
            x2= item.attrib['x2']
            y2= item.attrib['y2']
            generate_obstacle( float(x1), float(y1), float(x2), float(y2), idx)

def generate_gzb_world(pedsim_file_name): 
    pedsim_pkg_path = get_package_share_directory('pedsim_simulator')
    xml_scenario =  pedsim_pkg_path + "/scenarios/" + pedsim_file_name
    gazebo_world =  os.path.dirname(os.getcwd()) + "/worlds/" + pedsim_file_name.split('.')[0] + ".world" 
    global gzb_world
    with open(gazebo_world, 'w') as gzb_world:
        gzb_world.write("<?xml version=\"1.0\" ?>\n")    
        gzb_world.write(
        '''<!-- this file is auto generated using pedsim_gazebo_plugin pkg -->
<sdf version="1.5">
  <world name="default">
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo_spawner</namespace>
      </ros>
      <update_rate>1.0</update_rate>
    </plugin>
    <!-- Ground Plane -->
    <include>
        <uri>model://ground_plane</uri>
    </include>
    <include>
        <uri>model://sun</uri>
    </include>
    ''')
        parseXML(xml_scenario)
        gzb_world.write('''
  </world>
</sdf>
''')
    print("gazbo world has been generated: {}".format(gazebo_world))

if __name__ == "__main__": 
    pedsim_file_name = input(">> enter pedsim scenaria name: file_name.xml \n")
    # generate gazebo world 
    generate_gzb_world(pedsim_file_name)     
