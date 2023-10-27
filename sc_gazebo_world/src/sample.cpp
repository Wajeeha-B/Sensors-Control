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
  nh_(nh), running_(false), real_(false), tooClose_(false),
  laserProcessingPtr_(nullptr), imageProcessingPtr_(nullptr)
{
    sub1_ = nh_.subscribe("/robot1/scan", 100, &Sample::laserCallback,this);
    sub2_ = nh_.subscribe("/robot1/camera/rgb/image_raw", 100, &Sample::imageCallback,this);

    sub3_ = nh_.subscribe("/scan", 100, &Sample::laserCallback,this);
    sub4_ = nh_.subscribe("/camera/rgb/image_raw", 100, &Sample::imageCallback,this);
    
    sub5_ = nh_.subscribe("/robot1/camera/rgb/camera_info", 10, &Sample::cameraInfoCallback,this);
    //Subscribing to odometry of the robot
    sub6_ = nh_.subscribe("/robot1/odom", 100, &Sample::odomCallback,this);

    pubDrive_ = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel",3,false);

    pubRealDrive_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",3,false);

    //Service to enable the robot to start and stop from command line input
    service1_ = nh_.advertiseService("/mission", &Sample::request,this);
    //Service to toggle advanced goals from laser data
    service2_ = nh_.advertiseService("/real", &Sample::real,this);

    //Sets the default robotPose_ to 0
    robotPose_.position.x = 0.0;
    robotPose_.position.y = 0.0;
    robotPose_.position.z = 0.0;
    robotPose_.orientation.x = 0.0;
    robotPose_.orientation.y = 0.0;
    robotPose_.orientation.z = 0.0;
    robotPose_.orientation.w = 0.0;

    //Sets the default goal to (0,0)
    goal_.x = 0.0;
    goal_.y = 0.0;
    goal_.z = 0.0;
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

// A callback for odometry
void Sample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    std::unique_lock<std::mutex> lck(robotPoseMtx_); // Locks the data for the robotPose to be saved
    robotPose_ = pose; // We copy the pose here
}

//The main thread for processing data and publishing the markers and robot controls
void Sample::seperateThread() {

    while(laserData_.range_min+laserData_.range_max == 0.0 || imageData_.encoding[0] == 0 ||
          robotPose_.orientation.w+robotPose_.orientation.x+
          robotPose_.orientation.y+robotPose_.orientation.z == 0.0);

    // ROS_INFO("laser: %f\nimage: %u\nencoding: %d", laserData_.range_min+laserData_.range_max, imageData_.height+imageData_.width,
    // imageData_.encoding[0]);
    
    //NEED TO WAIT FOR A DATA POINT TO BE POPULATED LIKE ABOVE

    //Limits the execution of this code to 5Hz
    ros::Rate rate_limiter(5.0);
    while (ros::ok()) {

        //Locks all of the data with mutexes
        std::unique_lock<std::mutex> lck1 (laserDataMtx_);
        std::unique_lock<std::mutex> lck2 (imageDataMtx_);
        std::unique_lock<std::mutex> lck3 (cameraInfoDataMtx_);
        std::unique_lock<std::mutex> lck4 (robotPoseMtx_);
        
        LaserProcessing laserProcessing(laserData_);
        ImageProcessing imageProcessing(imageData_, cameraInfoData_);

        //Unlocks all mutexes
        lck4.unlock();
        lck3.unlock();
        lck2.unlock();
        lck1.unlock();

        int xPixel;
        xPixel = imageProcessing.TemplateMatch();
        // laserProcessing.myFunction(myInt);

        double angle;
        angle = imageProcessing.LocalAngle(xPixel);
        // ROS_INFO("angle: %f", angle);

        // ROS_INFO("AngleMin= %f\n AngleMax= %f\n AngleIncrement= %f", laserData_.angle_min, laserData_.angle_max, laserData_.angle_increment);
        
        
        double dist;
        dist = laserProcessing.FindDistance(angle);
        // ROS_INFO("Distance: %f", dist);

        if(dist < STOP_DISTANCE_ || dist > 2147483647) tooClose_ = true;
        else tooClose_ = false;

        geometry_msgs::Point localGoal_;
        localGoal_.x = dist*cos(angle);
        localGoal_.y = dist*sin(angle);
        localGoal_.z = 0;
        // ROS_INFO("Local Goal: [%f, %f, %f]", localGoal_.x, localGoal_.y, localGoal_.z);
        
        // ROS_INFO("Robot pose: [%f,%f,%f]", robotPose_.position.x, robotPose_.position.y, robotPose_.position.z);
        goal_ = local2Global(localGoal_, robotPose_);
        // ROS_INFO("Global Goal: [%f, %f, %f]", goal_.x, goal_.y, goal_.z);

        double steering = 0;
        steering = GetSteering(goal_, robotPose_);
        // ROS_INFO("steering: %f", steering);


        // if(goals_.size() == 0) goals_.push_back(goal_);
        // else if(DistanceBetweenGoals(goal_, goals_.back()) > GOAL_DISTANCE_) goals_.push_back(goal_);
        // ROS_INFO("distance between goals: %f", DistanceBetweenGoals(goal_, goals_.back()));
        // ROS_INFO("goal size: %ld", goals_.size());

        //If there are any goals stored, feed the first element into the goal_ variable
        // double steering = 0;
        // if(goals_.size() > 0)
        // {
        //     geometry_msgs::Point currentGoal;
        //     currentGoal.x = goals_.front().x;
        //     currentGoal.y = goals_.front().y;
        //     currentGoal.z = goals_.front().z;
            
        //     //If the distance to the goal is less than the stopping distance then the first element of the goal
        //     //is erased making the second goal is moved into the first element of the array.
        //     if(DistanceToGoal(currentGoal, robotPose_) < STOP_DISTANCE_) goals_.erase(goals_.begin());
        //     steering = GetSteering(currentGoal, robotPose_);
        //     // ROS_INFO("steering: %f", steering);
        // }

        geometry_msgs::Twist drive;
        if(running_ && !tooClose_){
            drive.linear.x = 0.1; //sends it forward
            drive.linear.y = 0.0;
            drive.linear.z = 0.0;
            drive.angular.x = 0.0;
            drive.angular.y = 0.0;
            // if (turning_ != 0) drive.angular.z = turning_*turningSens_;
            if (angle > 0.001 || angle < -0.001) drive.angular.z = angle;
            else drive.angular.z = 0.0;
            // drive.angular.z = steering*turningSens_;
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
        running_ = true; //start the robot if there is a goal
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

geometry_msgs::Point Sample::local2Global(geometry_msgs::Point goal, geometry_msgs::Pose robot)
{
    geometry_msgs::Point p;
    //transforms the location to a global reference accounting for the offset of the robot's location relative to the laser sensor's location
    p.x = (goal.x * cos(tf::getYaw(robot.orientation)) - goal.y * sin(tf::getYaw(robot.orientation))) + robot.position.x +SENSOR_OFFSET_*cos(tf::getYaw(robot.orientation));
    p.y = (goal.x * sin(tf::getYaw(robot.orientation)) + goal.y * cos(tf::getYaw(robot.orientation))) + robot.position.y +SENSOR_OFFSET_*sin(tf::getYaw(robot.orientation));
    return p;
}

double Sample::DistanceToGoal(geometry_msgs::Point goal, geometry_msgs::Pose robot)
{
    //finds the difference in x and y and get the hypotenuse between the two points
    double dist = sqrt(pow(goal.x-robot.position.x,2)+pow(goal.y-robot.position.y,2));
    return dist;
}

double Sample::DistanceBetweenGoals(geometry_msgs::Point goal1, geometry_msgs::Point goal2){
    double dist = sqrt(pow(goal1.x-goal2.x,2)+pow(goal1.y-goal2.y,2));
    return dist;
}

double Sample::GetSteering(geometry_msgs::Point goal, geometry_msgs::Pose robot)
{
    /*-----Chord Length-----*/
    double dx = goal.x-robot.position.x;    //difference in x between the robot's current position and the goal
    double dy = goal.y-robot.position.y;    //difference in y between the robot's current position and the goal
    double CL = sqrt(pow(dx,2)+pow(dy,2));  //Chord length is hypotenuse of perpendicular lengths of dx and dy

    /*-----Alpha Angle-----*/
    double AA = atan2(dy, dx)-tf::getYaw(robot.orientation);  //Alpha angle made with angle formed from chord from the yaw angle of the robot
    return AA;
}