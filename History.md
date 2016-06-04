
0.1.1 / 2016-06-04 
==================

 * Fixed point cloud sensor data problem with missing odometry topic
 * Added sensors module for simulation point clouds
 * Added (some) parameters to dynamic reconfigure
 * Consolidated relavant dependencies into one place to allow single
   clone development
 * Many improvements on the interface
 * Updated messages to match status of the SPENCER project 
 * Added simple mock scripts for creating repeatable tracks and groups
 * Cleanup many of the dependencies to have a lean setup

0.0.4 / 2015-03-30
==================

 * Updated launch files and config to match rviz plugins
 * cleanup of dependencies
 * adding compatibility modules with indigo
 * fix small bug with missing signal header

0.0.3b / 2014-11-19
==================

  * FIXED slow termination of the main node due to SIGINT and SIGDFL conflicts, thanks to @tlind
  * Fixes #5 removed simulator dependency on spencer messages, now using local pedsim_msgs copies
  * - AgentState.msg modified to follow Twist and Pose standards - robot teleop working with keyboard and xbox joystick - rviz file modified for visualization purposes
  * added more message types for self contained compilation
  * added animated markers to rviz config files

0.0.3 / 2014-08-22
==================

 * changes to standing and queueing scenes to generate more realistic data
 * better standing people modeling by tweaking different force weights
 * lots of cleanup, optimizations, speedups...
 * changed social state to a more consistent string notation
 * integrated spencer_social_relation_msgs for publishing simulated social activities
 * changed queue waiting time distribution to erlang
 * fixed spooky behavior in shopping modes
 * added walking people to the simulator and tuned behaviours, thanks to @tlind
 * fixed queue dancing behaviour issues (re-wrote personal space in queue code)
 * added spencer msgs to the simulator, not using standard spencer topics
 * now having people models and more in the simulator

0.0.2 / 2014-05-12
==================

 * added social status to agent status messages
 * minor edits when recording
 * more scenario files
 * new scenerios for collecting training data
 * minor changes in param when making the videos and cleanup
 * removing unnecessary files
 * new scene and launch file for social context experiments
 * polishing config to handle magic numbers
 * more refactoring and cleanup
 * more cleanup, minor fix in documentation
 * adding basic auto documentation
 * added publishing og waypoint markers
 * more cleanup and fixes
 * removing old files not needed anymore
 * continous update of center of mass in groups
 * minor cleanup, removing unnecessary slots
 * fixed standing/slow agents behaviour to simulate elders
 * changed queue service wait time distribution to exponential
 * fixed z visuals size error
 * making simple demo, minor fix on obstacle cells locations
 * fixed planning bug that did not factor obstacles
 * minor fix in slots and group params
 * new behaviour demonstration scene
 * package status update, more cleanup
 *  and we have nice teleoperation back!
 * daryl back in visualization, reworking teleop now with waypoint planners
 * add enable/disable switch for groups for easy behaviour switches
 * addign more debug statements
 * more cleanup, removing qt pieces
 * minor fixes and refactoring
 * fixed attraction visualization bug
 * minor fix in attractions
 * adding visuals for attractions
 * working queues, groups and attraction behaviour
 * restored robot command, combined agent status and visual info publishing
 * more cleanup, eliminating qt pieces, more files
 * more cleanup, eliminating qt pieces,
 * finally back to working groups and potentially queues etc
 * more files
 * adding sven files
 * backing up before adding components
 * more tuning, using neighborhood for group additions
 * tweaking parameters for group forces before branching the code to prep for intergration
 * now visualizing groups, force parameters need tuning
 * assigning people to groups...
 * repulsion and coherence
 * connecting forces between pedestrian groups and agents
 * pulling in groups: work in progress... more files
 * pulling in groups: work in progress...
 * moving the area to select agents to join queue to be length dependent
 * cleaning up states
 * adding infrastructure for group forces while making states more clear, additional standing behaviors
 * work in progress: rewritting queue and situation logic
 * congregating around attractions/standing... work in progress
 * adding more variables to config
 * allow explicit re-joining of queues (after a small window that is configurable)
 * reworking waiting points logic
 * randomized service times, queue buffers, queue directions and added flags to directly manipulate waypoints
 * cleaning up the experiment scenes
