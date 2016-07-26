#!/bin/bash
echo sourcing ...
source /home/cme-intel/.bashrc
echo starting ...

# STARTUP ROBOT SIMULATION +
echo GAZEBO starting ...
gnome-terminal --window-with-profile=cme-terminal-profile -x bash -c "roslaunch cob_bringup_sim robot.launch"
sleep 30s
# STARTUP ROBOT NAVIAGTION +
echo NAVIGATION starting ...
gnome-terminal --window-with-profile=cme-terminal-profile -x bash -c "roslaunch cob_navigation_global 2dnav_ros_eband.launch"
sleep 10s
# STARTUP RVIZ (not necessary) +
echo RVIZ starting ...
gnome-terminal --window-with-profile=cme-terminal-profile -x bash -c "roslaunch planning_demo rviz.launch"
sleep 10s
echo Set robot to starting position GAZEBO MODEL STATE - robot model
rosservice call /gazebo/set_model_state "{model_state: {model_name: robot, pose: {position: { x: -3.6, y: -2.1, z: 0.0 }, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: { linear: {x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 }}, reference_frame: world }}"
sleep 2s
# RESET RVIZ POS
echo RVIZ resetting ...
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: { stamp: now, frame_id: /map}, pose: { pose: { position: { x: -3.6, y: -2.1, z: 0.0}, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]}}' &
# RESET COSTMAP
echo COSTMAP resetting ...
rosservice call /move_base/clear_costmaps
sleep 2s
echo -------------------- END --------------------
