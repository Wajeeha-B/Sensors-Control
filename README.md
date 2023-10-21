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
### Terminal 1:
```
roslaunch sc_gazebo_world main.launch
```
### Terminal 2 (To control the Turtlebot Guider):
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot2/cmd_vel
```
### Terminal 3 (Rosservice call):
```
rosservice call /mission "data: true"
```
### Alternative to Terminal 1 (Open the bag):
```
cd  catkin_ws/src/Sensors-Control/sc_gazebo_word/src/bag
rosbag play -r 0.1 --clock -l Main_record.bag
```
### Recording a bag:
**Open the simulation(gazebo)**
```
cd  ~/catkin_ws/src/Sensors-Control/sc_gazebo_world/src/bag
rosbag record /robot1/scan /robot2/imu -l 10
```
### Reset the gazebo world (might not work)
```
rosservice call /gazebo/reset_world "{}"
```


