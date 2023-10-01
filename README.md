# Senors-Control
Group 6 | Project 7 | Turtlebot Robot Following Each Other
## Naming structure
sc_<packagename>
e.g. sc_gazebo_world
     sc_description

## Cloning the Repository
    git clone git@github.com:Wajeeha-B/Sensors-Control.git
    cd ~/catkin_ws/src/Sensors-Control/sc_gazebo_world/model
    cd ~/catkin_ws
    catkin_make

## Launching the Simulation
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch sc_gazebo_world turtlebot3_marker_V2.launch
    
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
    
## Spawning AR Tag
    rosrun gazebo_ros spawn_model -file /home/wajeeha/catkin_ws/src/Sensors-Control/sc_gazebo_world/model Apriltag36_11_00000/model.sdf -sdf -model Apriltag36_11_00000 -x -0.1 -y 0 -z 0.1
