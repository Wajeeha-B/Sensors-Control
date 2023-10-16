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
     Terminal 1:
    export TURTLEBOT3_MODEL1=waffle_pi
    export TURTLEBOT3_MODEL2=waffle_pi
    roslaunch sc_gazebo_world turtlebot3_marker_V3_V2\ .launch

    Terminal 2:
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

    Terminal 3:
    export TURTLEBOT3_MODEL=waffle_pi
     roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

## New Launching
roslaunch sc_gazebo_world_v1 main.launch

## Moving the robot
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot2/cmd_vel
    
## Spawning AR Tag
Change the directory according to your own path

rosrun gazebo_ros spawn_model -file /home/wajeeha/catkin_ws/src/Sensors-Control/sc_gazebo_world/model/Apriltag36_11_00000/model.sdf -sdf -model Apriltag36_11_00000 -x 0 -y 0 -z 0

## Video
https://drive.google.com/file/d/125ZiM6GYPrjkZ3lm5_p5PHiODzwtuHtM/view?usp=sharing
