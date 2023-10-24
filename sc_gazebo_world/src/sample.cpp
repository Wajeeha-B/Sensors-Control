#include "sample.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

// #include "std_msgs/Float64.h"
// #include "std_msgs/Empty.h"


using std::cout;
using std::endl;

//Default constructor of the sample class
Sample::Sample(ros::NodeHandle nh) :
  nh_(nh), running_(false), real_(false),
  laserProcessingPtr_(nullptr), imageProcessingPtr_(nullptr)
{
    sub1_ = nh_.subscribe("/robot1/scan", 100, &Sample::laserCallback,this);
    sub2_ = nh_.subscribe("/robot1/camera/rgb/image_raw", 100, &Sample::imageCallback,this);

    sub3_ = nh_.subscribe("/scan", 100, &Sample::laserCallback,this);
    sub4_ = nh_.subscribe("/camera/rgb/image_raw", 100, &Sample::imageCallback,this);
    
    sub5_ = nh_.subscribe("/robot1/camera/rgb/camera_info", 10, &Sample::cameraInfoCallback,this);

    pubDrive_ = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel",3,false);

    pubRealDrive_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",3,false);

    //Service to enable the robot to start and stop from command line input
    service1_ = nh_.advertiseService("/mission", &Sample::request,this);
    //Service to toggle advanced goals from laser data
    service2_ = nh_.advertiseService("/real", &Sample::real,this);

}

// We delete anything that needs removing here specifically
Sample::~Sample(){
    if(laserProcessingPtr_ != nullptr){
        delete laserProcessingPtr_;
    }
    if(imageProcessingPtr_ != nullptr){
        delete imageProcessingPtr_;
    }
}

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

void Sample::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    std::unique_lock<std::mutex> lck(cameraInfoDataMtx_); // Locks the data for the laserData to be saved
    cameraInfoData_ = *msg; // We store a copy of the LaserScan in laserData_
}

//The main thread for processing data and publishing the markers and car controls
void Sample::seperateThread() {

    while(laserData_.range_min+laserData_.range_max == 0.0 || imageData_.encoding[0] == 0);

    // ROS_INFO("laser: %f\nimage: %u\nencoding: %d", laserData_.range_min+laserData_.range_max, imageData_.height+imageData_.width,
    // imageData_.encoding[0]);
    
    //NEED TO WAIT FOR A DATA POINT TO BE POPULATED LIKE ABOVE

    //Limits the execution of this code to 5Hz
    ros::Rate rate_limiter(5.0);
    while (ros::ok()) {

        // //Locks all of the data with mutexes
        std::unique_lock<std::mutex> lck1 (laserDataMtx_);
        std::unique_lock<std::mutex> lck2 (imageDataMtx_);
        std::unique_lock<std::mutex> lck3 (cameraInfoDataMtx_);
        
        LaserProcessing laserProcessing(laserData_);
        ImageProcessing imageProcessing(imageData_, cameraInfoData_);

        //Unlocks all mutexes
        lck3.unlock();
        lck2.unlock();
        lck1.unlock();

        xPixel_ = imageProcessing.TemplateMatch();
        laserProcessing.myFunction(myInt);

        double angle;
        angle = imageProcessing.LocalAngle(xPixel_);

        geometry_msgs::Twist drive;
        if(running_){
            drive.linear.x = 0.1; //sends it forward
            drive.linear.y = 0.0;
            drive.linear.z = 0.0;
            drive.angular.x = 0.0;
            drive.angular.y = 0.0;
            // if (turning_ != 0) drive.angular.z = turning_*turningSens_;
            if (angle > 0.001 || angle < -0.001) drive.angular.z = angle;
            else drive.angular.z = 0.0;
        }
        else{
            drive.linear.x = 0.0;
            drive.linear.y = 0.0;
            drive.linear.z = 0.0;
            drive.angular.x = 0.0;
            drive.angular.y = 0.0;
            drive.angular.z = 0.0;
        }

        if(!real_) pubDrive_.publish(drive);
        else pubRealDrive_.publish(drive);

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
        running_ = true; //start the car if there is a goal
        res.success = true;
        res.message = "The Turtlebot has started it's mission";

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

//Service that handles toggling between advanced and basic goals, basic goals are on by default
bool Sample::real(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res)
{
    //if the data is set to true, advanced goals will be used and advGoals_ = true
    if(req.data)
    {
        ROS_INFO_STREAM("Requested: Real topics");
        res.success = true;
        res.message = "Switched to real topics";
        real_ = true;
    }
    //otherwise basic goals is used, advGoals_ = false
    else
    {
        ROS_INFO_STREAM("Requested: Sim topics");
        res.success = true;
        res.message = "Switched to sim topics";
        real_ = false;
    }
    //returns true every time since switching goals shouldn't be prevented for any obvious reason
    return true;
}