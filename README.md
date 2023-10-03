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
    export TURTLEBOT3_MODEL1=waffle_pi
    export TURTLEBOT3_MODEL2=waffle_pi
    roslaunch sc_gazebo_world turtlebot3_marker_V3_V2\ .launch
    
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
    
## Spawning AR Tag
Change the directory according to your own path

rosrun gazebo_ros spawn_model -file /home/wajeeha/catkin_ws/src/Sensors-Control/sc_gazebo_world/model/Apriltag36_11_00000/model.sdf -sdf -model Apriltag36_11_00000 -x 0 -y 0 -z 0

## Video
https://studentutsedu-my.sharepoint.com/:v:/g/personal/wajeeha_batool_student_uts_edu_au/EXfFVYyE4yVFrK9yGzpa2qUBO8vwA6Brl32uWrpU21oHtQ?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZyIsInJlZmVycmFsQXBwUGxhdGZvcm0iOiJXZWIiLCJyZWZlcnJhbE1vZGUiOiJ2aWV3In19&email=Wajeeha.Batool%40student.uts.edu.au&e=7LjdB2
