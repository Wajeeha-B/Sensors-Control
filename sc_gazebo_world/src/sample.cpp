#include "sample.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

// #include "std_msgs/Float64.h"
// #include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

using std::cout;
using std::endl;

//Default constructor of the sample class
Sample::Sample(ros::NodeHandle nh) :
//   nh_(nh),laserProcessingPtr_(nullptr), ackermanPtr_(nullptr), marker_counter_(0),
  running_(false), 
//goalOK_(false), advGoals_(false)
    nh_(nh),laserProcessingPtr_(nullptr), imageProcessingPtr_(nullptr)
{
    // //Sets the default robotPose_ to 0
    // robotPose_.position.x = 0.0;
    // robotPose_.position.y = 0.0;
    // robotPose_.position.z = 0.0;
    // robotPose_.orientation.x = 0.0;
    // robotPose_.orientation.y = 0.0;
    // robotPose_.orientation.z = 0.0;
    // robotPose_.orientation.w = 0.0;

    // //Sets the default goal to (0,0)
    // goal_.x = 0.0;
    // goal_.y = 0.0;
    // goal_.z = 0.0;

    sub1_ = nh_.subscribe("/robot1/scan", 100, &Sample::laserCallback,this);
    sub2_ = nh_.subscribe("/robot1/camera/rgb/image_raw", 100, &Sample::imageCallback,this);

    // //Publishing markers
    // pubVis_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);
    // //Publishing brakes
    // pubBrake_ = nh_.advertise<std_msgs::Float64>("orange/brake_cmd",3,false);
    // //Publishing steering
    // pubSteer_ = nh_.advertise<std_msgs::Float64>("orange/steering_cmd",3,false);
    // //Publishing throttle
    // pubThrottle_ = nh_.advertise<std_msgs::Float64>("orange/throttle_cmd",3,false);
    // //Publishing cones
    // pubCones_ = nh_.advertise<geometry_msgs::PoseArray>("orange/cones",3,false);

    pubDrive_ = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel",3,false);

    //Service to enable the robot to start and stop from command line input
    service1_ = nh_.advertiseService("/mission", &Sample::request,this);
    // //Service to toggle advanced goals from laser data
    // service2_ = nh_.advertiseService("/toggle_advanced", &Sample::advanced,this);

}

// We delete anything that needs removing here specifically
Sample::~Sample(){
    // //deletes the laserprocessing object pointer
    // if(laserProcessingPtr_ != nullptr){
    //     delete laserProcessingPtr_;
    // }
    // //deletes the ackerman object pointer
    // if(ackermanPtr_ != nullptr){
    //     delete ackermanPtr_;
    // }
    if(laserProcessingPtr_ != nullptr){
        delete laserProcessingPtr_;
    }
    if(imageProcessingPtr_ != nullptr){
        delete imageProcessingPtr_;
    }
}

// //Creates markers to be displayed in the simulator
// visualization_msgs::Marker Sample::createMarker(geometry_msgs::Point point, markerType::obj type){
  
//   visualization_msgs::Marker marker;

//   //Set the reference frame ID and time stamp.
//   marker.header.frame_id = "world";
//   marker.header.stamp = ros::Time::now();

//   //Lifetime of the object, matches the Hz of the rate limiter
//   marker.lifetime = ros::Duration(0.2);
  
//   //Unique ID of the marker
//   marker.id = marker_counter_++; 

//   // The marker type, a road marker uses a cube
//   if(type == markerType::obj::road){
//     marker.type = visualization_msgs::Marker::CUBE;
//     // Set the scale of the marker
//     marker.scale.x = 0.5;
//     marker.scale.y = 0.5;
//     marker.scale.z = 0.5;
//     //Colour is r,g,b where each channel of colour is 0-1. This marker is green
//     marker.color.r = 0;
//     marker.color.g = 1;
//     marker.color.b = 0;
//     // Set the namespace of the marker
//     marker.ns = "road";
//   }
//   else if (type == markerType::obj::cones){
//     marker.type = visualization_msgs::Marker::CYLINDER;
//     // Set the scale of the marker
//     marker.scale.x = 0.4;
//     marker.scale.y = 0.4;
//     marker.scale.z = 0.5;
//     //Colour is r,g,b where each channel of colour is 0-1. This marker is orange
//     marker.color.r = 1;
//     marker.color.g = 150.0/255.0;
//     marker.color.b = 0;
//     // Set the namespace of the marker
//     marker.ns = "cones";
//   }

//   // Set the marker action. This will add it to the screen
//   marker.action = visualization_msgs::Marker::ADD;

//   // Sets the position of the marker
//   marker.pose.position.x = point.x;
//   marker.pose.position.y = point.y;
//   marker.pose.position.z = point.z;

//   //Sets the orientation, we are not going to orientate it
//   marker.pose.orientation.x = 0.0;
//   marker.pose.orientation.y = 0.0;
//   marker.pose.orientation.z = 0.0;
//   marker.pose.orientation.w = 1.0;

//   //Alpha is transparency (50% transparent)
//   marker.color.a = 0.5f;

//   return marker;
// }

// // Converts the data type of the cone from a geometry_msgs::Point to a geometry_msgs::Pose with orientation
// geometry_msgs::Pose Sample::createCone(geometry_msgs::Point cone){
//     geometry_msgs::Pose conePose;
//     // The position is the same
//     conePose.position.x = cone.x;
//     conePose.position.y = cone.y;
//     conePose.position.z = cone.z;
//     // The orientation is added
//     conePose.orientation.x = 0.0;
//     conePose.orientation.y = 0.0;
//     conePose.orientation.z = 0.0;
//     conePose.orientation.w = 1.0;
//     return conePose;
// }

// // A callback for odometry
// void Sample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
// {
//     geometry_msgs::Pose pose = msg->pose.pose;
//     std::unique_lock<std::mutex> lck1 (robotPoseMtx_); // Locks the data for the robotPose to be saved
//     robotPose_ = pose; // We copy the pose here
// }

// // A callback for laser scans
// void Sample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
// {
//     std::unique_lock<std::mutex> lck(laserDataMtx_); // Locks the data for the laserData to be saved
//     laserData_ = *msg; // We store a copy of the LaserScan in laserData_
// }

// // A callback for predetermined goals
// void Sample::goalCallback(const geometry_msgs::PoseArrayConstPtr& msg)
// {
//     std::unique_lock<std::mutex> lck(goalsMtx_); // Locks the data for the goals to be saved
//     goals_ = *msg; // We store a copy of the the PoseArray in Goals_
// }

void Sample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    std::unique_lock<std::mutex> lck(laserDataMtx_); // Locks the data for the laserData to be saved
    laserData_ = *msg; // We store a copy of the LaserScan in laserData_
}

void Sample::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::unique_lock<std::mutex> lck(imageDataMtx_); // Locks the data for the laserData to be saved
    imageData_ = *msg; // We store a copy of the LaserScan in laserData_
}

//The main thread for processing data and publishing the markers and car controls
void Sample::seperateThread() {
    // //waits for the robotPose orientation to be populated by odometry callback
    // while(robotPose_.orientation.w+
    //       robotPose_.orientation.x+
    //       robotPose_.orientation.y+
    //       robotPose_.orientation.z == 0.0);
    while(laserData_.range_min+laserData_.range_max == 0.0 && imageData_.height+imageData_.width == 0.0);
    //NEED TO WAIT FOR A DATA POINT TO BE POPULATED LIKE ABOVE

    //Limits the execution of this code to 5Hz
    ros::Rate rate_limiter(5.0);
    while (ros::ok()) {

        // //Locks all of the data with mutexes
        // std::unique_lock<std::mutex> lck1 (robotPoseMtx_);
        // std::unique_lock<std::mutex> lck2 (laserDataMtx_);
        // std::unique_lock<std::mutex> lck3 (goalsMtx_);
        std::unique_lock<std::mutex> lck1 (laserDataMtx_);
        std::unique_lock<std::mutex> lck2 (imageDataMtx_);

        // //Gets the yaw
        // double r_yaw =  tf::getYaw(robotPose_.orientation);

        // //Gets the laser data and feeds it into the laserprocessing library functions
        // LaserProcessing laserProcessing(laserData_);
        // //Initialises an Ackerman object
        // Ackerman ackerman;
        // //Finds the centre between the cones for advanced goals
        // geometry_msgs::Point coneCentre = laserProcessing.detectRoadCentre();
        
        LaserProcessing laserProcessing(laserData_);
        ImageProcessing imageProcessing(imageData_);

        // //Checks if advanced goals is being used
        // if(advGoals_)
        // {
        //     //Converts the cone's coordinates from the laser's reference to the global reference
        //     goal_ = ackerman.local2Global(coneCentre,robotPose_);
        // }
        // //Else, assume predetermined goals
        // else
        // {   
        //     //If there are any goals stored, feed the first element into the goal_ variable
        //     if(goals_.poses.size() > 0)
        //     {
        //         geometry_msgs::Point currentGoal;
        //         currentGoal.x = goals_.poses.front().position.x;
        //         currentGoal.y = goals_.poses.front().position.y;
        //         currentGoal.z = goals_.poses.front().position.z;
        //         goal_ = currentGoal;
                
        //         //If the distance to the goal is less than the stopping distance then the first element of the goal
        //         //is erased making the second goal is moved into the first element of the array.
        //         if(ackerman.DistanceToGoal(goal_, robotPose_) < STOP_DISTANCE_) goals_.poses.erase(goals_.poses.begin());
        //     }
        // }
        // //If the goal is invalid
        // if(goal_.z == 999)
        // {
        //     //goal is invalid
        //     goalOK_ = false;
        //     //stop running the mission
        //     running_ = false;
        // }
        // //Otherwise the goal is valid
        // else goalOK_ = true;
        
        //CREATES THE DATA TO BE PUBLISHED
        // //Initialises the float64s for the publishers
        // std_msgs::Float64 steering, brake, throttle;

        // //Gets the steering based on the goal and the robotPose
        // steering.data = ackerman.GetSteering(goal_, robotPose_);
        

        //Unlocks all mutexes
        // lck3.unlock();
        lck2.unlock();
        lck1.unlock();
            
        // //While the car is running, the throttle is being set to its default value 
        // if(running_){
        //     throttle.data = ackerman.GetThrottle();
        //     brake.data = 0.0;
        // }
        //While the car is not running, the brakes is applied 
        // else{
        //     throttle.data = 0.0;
        //     brake.data = ackerman.GetBrake();
        // }

        // //We create a marker for the goal
        // visualization_msgs::Marker marker = createMarker(goal_, markerType::obj::road);
        // visualization_msgs::MarkerArray marker_array; //creates the marker array for publishing
        // marker_array.markers.push_back(marker); //goal marker is pushed into marker array
        
        // geometry_msgs::PoseArray pose_array; // the pose array is made for publishing
        // for(unsigned int i = 0; i < laserProcessing.countSegments(); i++){
        //     geometry_msgs::Point coneidx = ackerman.local2Global(laserProcessing.getCones().at(i),robotPose_);
        //     visualization_msgs::Marker coneMark = createMarker(coneidx, markerType::obj::cones);
        //     marker_array.markers.push_back(coneMark); //cone marker is pushed into marker array
            
        //     geometry_msgs::Pose conePose = createCone(coneidx);
        //     pose_array.poses.push_back(conePose); //cone marker is pushed into cone array
        // }
        // We publish to the topics
        // if(running_){
        //     pubCones_.publish(pose_array);
        //     pubSteer_.publish(steering);
        //     pubThrottle_.publish(throttle);
        // }
        // // The markers and brakes are published whether the car is running or not
        // pubVis_.publish(marker_array);
        // pubBrake_.publish(brake);

        // unsigned int i = laserProcessing.countObjectReadings();
        // ROS_INFO("Laser readings: %u", i);

        imageProcessing.TemplateMatch();

        geometry_msgs::Twist drive;
        if(running_){
            drive.linear.x = 0.1; //sends it forward
            drive.linear.y = 0.0;
            drive.linear.z = 0.0;
            drive.angular.x = 0.0;
            drive.angular.y = 0.0;
            drive.angular.z = 0.0;
        }
        else{
            drive.linear.x = 0.0; //sends it forward
            drive.linear.y = 0.0;
            drive.linear.z = 0.0;
            drive.angular.x = 0.0;
            drive.angular.y = 0.0;
            drive.angular.z = 0.0;
        }

        pubDrive_.publish(drive);

        //We have a rate timer, this sleep here is needed to ensure it stops and sleeps 
        //it will do it for the exact amount of time needed to run at 5Hz
        rate_limiter.sleep();
    }
}

//Service that handles starting and stopping the missions based on command line input
//Communicate with this service using 
//rosservice call /mission "data: true"
bool Sample::request(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res)
{  
    //If the request is true, start the mission
    if(req.data)
    {
        ROS_INFO_STREAM("Requested: Start mission");
        //Checks if the goal is valid before proceeding
        // if(goalOK_){
            running_ = true; //start the car if there is a goal
            res.success = true;
            res.message = "The Turtlebot has started it's mission";
        // } 
        //If the goal is invalid, dont start the mission
        // else {
        //     res.success = false;
        //     res.message = "Couldn't find the goal!";
        //     return false;
        // }
    }
    //if the request is false, stop the mission
    else
    {
        ROS_INFO_STREAM("Requested: Stop mission");
        res.success = true;
        res.message = "Turtlebot stopping";
        running_ = false;
    }
    //return true when the service completes its request
    return true;
}

// //Service that handles toggling between advanced and basic goals, basic goals are on by default
// bool Sample::advanced(std_srvs::SetBool::Request  &req,
//              std_srvs::SetBool::Response &res)
// {
//     //if the data is set to true, advanced goals will be used and advGoals_ = true
//     if(req.data)
//     {
//         ROS_INFO_STREAM("Requested: Advanced goals");
//         res.success = true;
//         res.message = "Switched to advanced goals";
//         advGoals_ = true;
//     }
//     //otherwise basic goals is used, advGoals_ = false
//     else
//     {
//         ROS_INFO_STREAM("Requested: Basic goals");
//         res.success = true;
//         res.message = "Switched to basic goals";
//         advGoals_ = false;
//     }
//     //returns true every time since switching goals shouldn't be prevented for any obvious reason
//     return true;
// }