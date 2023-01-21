#!/bin/bash
# source ros2 installation
source /opt/ros/humble/setup.bash
source ../install/setup.bash

EXAMPLE_SCENARIOS="scenarios/*.xml"
CONTEXT_SCENARIOS="scenarios/contexts/*.xml"
resolution=0.1 # meters oer grid cell

# for f in $EXAMPLE_SCENARIOS
# do
#   file=${f#*/}
#   file_name=${file%.*}
#   output_file="maps/$file_name"
#   echo "Processing ${file} file..."
#   # run converter script on each file to produce .pgm and .yaml maps.
#   ros2 run pedsim_simulator pedsim_xml_to_occ_map -i ${f} -o ${output_file} -r ${resolution}
# done

for f in $CONTEXT_SCENARIOS
do
  file=${f#*/}
  file_name=${file%.*}
  output_file="maps/$file_name"
  echo "Processing ${file} file..."
  # run converter script on each file to produce .pgm and .yaml maps.
  ros2 run pedsim_simulator pedsim_xml_to_occ_map -i ${f} -o ${output_file} -r ${resolution}
done