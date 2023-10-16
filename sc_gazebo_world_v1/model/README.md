# April tags

## Installation:
    export ROS_DISTRO=noetic
    source /opt/ros/$ROS_DISTRO/setup.bash
    
    cd ~/catkin_ws/src
    git clone https://github.com/AprilRobotics/apriltag_ros.git
    
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make


## SETUP:
    
### Set tag family (using 36h11 by default, so you don't need to do this step):
    gedit ~/catkin_ws/src/apriltag_ros/apriltag_ros/config/settings.yaml

### Add standalone tags:
    gedit ~/catkin_ws/src/apriltag_ros/apriltag_ros/config/tags.yaml
#### replace:
    standalone_tags: 
    [
    ]
#### with:   
    standalone_tags: 
    [
        {id: 0, size: 1.0, description: "TAG_0"}
    ]

    
### Calibrate Camera (`DONT WORRY ABOUT FOR NOW`):
#### Install calibration library:
    rosdep install camera_calibration

#### Calibrate steps (change --square 0.024 to actual size in metres):
    roslaunch usb_cam usb_cam-test.launch
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/camera/image_raw camera:=/camera
Note: Save file to a location, the 'ost.yaml' file contains the useful data
    
#### Edit:
    gedit ~/catkin_ws/src/launch/usb_cam-test.launch

#### Add line (file location to match where you saved 'ost.yaml'):
    <param name="camera_info_url" value="file:///home/main/catkin_ws/src/usb_cam/cam_calibration/ost.yaml" />

## LAUNCH:
### Launch a camera publishing node (2 options):
#### `OPTION 1:` USB CAM:
    roslaunch usb_cam usb_cam-test.launch

#### `OPTION 2:` turtleBot3:
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch sc_gazebo_world turtlebot3_marker_V3_V2\ .launch
    
    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

### Launch detection for turtlebot:
    roslaunch apriltag_ros continuous_detection.launch camera_name:="/camera" image_topic:="/rgb/image_raw" queue_size:="1"

### Launch detection for USB-camera:
    roslaunch apriltag_ros continuous_detection.launch camera_name:="/usb_cam" image_topic:="/rgb/image_raw" queue_size:="1"

### View detection:
    rqt_image_view
click `dropdown` -> `tag_detections_image`
    

### Working:
![AprilTag detection estimating pose with turtlebot3](https://github.com/KennyTafianoto/RS1-ProjectRover/blob/master/examples/apriltag_example.png)
        

### Reference List
April Tag Models: https://github.com/koide3/gazebo_apriltag.git  
AR Tracking 1: http://wiki.ros.org/apriltag_ros  
~~AR Tracking 2: http://wiki.ros.org/ar_track_alvar~~

Turtlebot3 + D435i:
- https://www.youtube.com/watch?v=hpUCG6K5muI
- https://github.com/rickstaa/realsense-ros-gazebo
- https://github.com/pal-robotics-forks/realsense
- https://github.com/pal-robotics-forks/realsense/tree/kinetic-devel/realsense2_description/urdf

SLAM with D435i:
- https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i
