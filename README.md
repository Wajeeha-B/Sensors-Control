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
### Terminal 1 (Launching the Sim):
```
roslaunch sc_gazebo_world main.launch
```
### Terminal 2 (To control the Turtlebot Guider):
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot2/cmd_vel
```
### Terminal 3 (Running the code):
```
rosrun sc_gazebo_world sc_gazebo_world
```
### Terminal 4 (Rosservice call):
```
rosservice call /mission "data: true"
```
To stop the movement of the follower robot, set data to false.
## Alternative to Launching the Simulation 
### Terminal 1 (Opening RosCore):
```
roscore
```
### Terminal 2 (Open the bag):
```
cd  catkin_ws/src/Sensors-Control/sc_gazebo_world/src/bag
rosbag play -r 0.1 --clock -l Main_record.bag
```
### Recording a bag:
**Open the simulation(gazebo)**
```
cd  ~/catkin_ws/src/Sensors-Control/sc_gazebo_world/src/bag
rosbag record /robot1/scan /robot2/imu -l 10
```
## Reset the gazebo world
```
rosservice call /gazebo/reset_world "{}"
```


