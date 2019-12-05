#!/usr/bin/env python
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""
import math
from rospkg import RosPack
import xml.etree.ElementTree as ET 

global gzb_world
def generate_pose(x1, y1, x2, y2):
    if x2-x1==0:
        print >> gzb_world, "\t  <pose frame=''>{} {}  1.4 0 0 {}</pose>".format( (x2+x1)/2, (y2+y1)/2, 1.57 )#x1, y1, math.atan((y2-y1)/(x2-x1)) )
    else:
        print >> gzb_world, "\t  <pose frame=''>{} {}  1.4 0 0 {}</pose>".format( (x2+x1)/2, (y2+y1)/2, math.atan( (y2-y1)/(x2-x1) )  )#x1, y1, math.atan((y2-y1)/(x2-x1)) )

def generate_size(x1, y1, x2, y2):
    l = math.sqrt( pow(y2-y1,2) + pow(x2-x1,2)  )
    print >> gzb_world, "\t     <size> {} .2  2.8 </size>".format( l )
    
    
    
def generate_obstacle(x1, y1, x2, y2, idx):
     print >> gzb_world, '''
     <model name='grey_wall_{}'>
      <static>1</static>
      <link name='link'>
      '''.format(idx)
     generate_pose(x1, y1, x2, y2)     
     print >> gzb_world, '''    
        <collision name='collision'>
          <geometry>
            <box>
            '''
     generate_size(x1, y1, x2, y2)
     print >> gzb_world, '''
      \t   </box>
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
            '''
     generate_size(x1, y1, x2, y2)
     print >> gzb_world, '''
      \t   </box>
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
    '''.format( 1,1)#math.sqrt( pow(y2-y1,2) + pow(x2-x1,2)  ) )



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

    
def generate_gzb_world( pedsim_file_name ): 
    rospack1 = RosPack()
    pkg_path = rospack1.get_path('pedsim_simulator')
    xml_scenario =  pkg_path + "/scenarios/" + pedsim_file_name #bo_airport.xml"
    rospack2 = RosPack()
    pkg_path = rospack2.get_path('pedsim_gazebo_plugin')
    gazebo_world =  pkg_path + "/worlds/" + pedsim_file_name.split('.')[0] + ".world" #bo_airport.xml"
    global gzb_world    
    with open(gazebo_world, 'w') as gzb_world:
        print >> gzb_world, "<?xml version=\"1.0\" ?>"    
        print >> gzb_world, '''
    <!-- this file is auto generated using pedsim_gazebo_plugin pkg -->    
    <sdf version="1.5">
      <world name="default">
      
      <!-- Ground Plane -->
      
      <include>
        <uri>model://ground_plane</uri>
      </include>
    
      <include>
        <uri>model://sun</uri>
      </include>
    
        '''
        # use the parse() function to load and parse an XML file
    #    xmlfile =  pkg_path + "/scenarios/obstacle.xml"
        parseXML(xml_scenario)
        print >> gzb_world, '''
            <plugin name="ActorPosesPlugin" filename="libActorPosesPlugin.so">
        </plugin>
    
    
      </world>
    </sdf>
    
        '''
    print "gazbo world has been generated: {}".format( gazebo_world)
      
      
      
def generate_launch_file( pedsim_file_name ): 
    rospack2 = RosPack()
    pkg_path = rospack2.get_path('pedsim_gazebo_plugin')
    launch_file =  pkg_path + "/launch/" + pedsim_file_name.split('.')[0] + ".launch" #bo_airport.xml"
    with open(launch_file, 'w') as launch:
        print >> launch, '''<launch>

        <!-- this file is auto generated using pedsim_gazebo_plugin pkg -->  
        
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
             <arg name="world_name" value="$(find pedsim_gazebo_plugin)/worlds/{}.world"/>
         </include>
         
         <!-- this node spawn pedsim actors to gazebo once, then the plugin updates their pose -->  
         <node pkg="pedsim_gazebo_plugin" type="spawn_pedsim_agents.py" name="spawn_pedsim_agents"  output="screen">
         </node>


</launch>
'''.format(pedsim_file_name.split('.')[0])
    print "launch file has been generated: {}".format( launch_file )
 

      
if __name__ == "__main__": 
    pedsim_file_name = raw_input(">> enter pedsim scenaria name: file_name.xml \n")

    # genrate gazebo wolrd 
    generate_gzb_world( pedsim_file_name )     
    generate_launch_file( pedsim_file_name ) 
    print ">> after you launch the scenario using pedsim_simulator, launch the generated world using: "
    print " \" $roslaunch pedsim_gazebo_plugin {}.launch\"  ".format( pedsim_file_name.split('.')[0] )
    