#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

//ROS data types
// #include "std_msgs/Float64.h"
// #include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

//We include header of another class we are developing
#include "laserprocessing.h"
#include "imageprocessing.h"

/*!
 *  \brief     Sample Class
 *  \details
 *  This class is used for communicating with the simulator environment using ROS using the provided libraries.
 *  It is designed to use laserprocessing to intepret the data and ackerman to use the laser data and generate an input for the control or the UGV.
 *  It can control the UGV in 2 specific ways based on the goals it uses. It can use basic or advanced goals, basic being an array of predetermined goals
 *  that are save in a text document and separately executed and published into a topic that Sample is subscribed to.
 *  While advanced goals actively uses the laser data and its ability to identify a cone in the simulator to find the goal that lies in between the two closest cones.
 *  As it moves towards the goal, more cones are discovered and become the next goal so that the UGV can complete a lap of the course.
 *  Sample uses a service to switch between basic and advanced goals. It also uses a second service to initialise its movement by starting the mission.
 *  \author    Ashton Powell
 *  \version   1.00
 *  \date      2023-05-30
 */
class Sample
{
public:
  /// @brief Constructor of the Sample class.
  ///
  /// Sets the default values of variables such as the robot position, the goals, the running_ boolean, etc.
  /// Requires the NodeHandle input to communicate with ROS.
  Sample(ros::NodeHandle nh);

  /// @brief Destructor of the Sample class.
  ///
  /// Deletes the object pointers for laserprocessing and ackerman classes.
  ~Sample();
  
  /// @brief seperate thread.
  ///
  /// The main processing thread that will run continously and utilise the data.
  /// When data needs to be combined then running a thread seperate to callback will guarantee data is processed.
  /// The data is then used to publish markers and control the car, causing new data to be generated on its updated position and perspective.
  void seperateThread();

  /// @brief request service callback for starting and stopping the mission and car movement.
  ///
  /// @param [in] req The request, a boolean value where true means the mission is in progress and false stops the mission.
  /// @param [in] res The response, a boolean and string value indicating if starting the mission was successful.
  ///
  /// @return bool - Will return true to indicate the request succeeded.
  bool request(std_srvs::SetBool::Request  &req,
               std_srvs::SetBool::Response &res);
  
  /// @brief request service callback for toggling between advanced and basic goals.
  /// @param [in] req the request, if true advanced goals is used, if false basic is used.
  /// @param [in] res the response, shares the current level goals being used.
  /// @return a boolean value to indicate the request was successful.
  bool real(std_srvs::SetBool::Request  &req,
            std_srvs::SetBool::Response &res);

  geometry_msgs::Point local2Global(geometry_msgs::Point goal, geometry_msgs::Pose robot);
  double DistanceToGoal(geometry_msgs::Point goal, geometry_msgs::Pose robot);
  double GetSteering(geometry_msgs::Point goal, geometry_msgs::Pose robot);

private:
  
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  //! Node handle for communication
  ros::NodeHandle nh_;

  ros::Publisher pubDrive_;

  ros::Publisher pubRealDrive_;

  // PUBLISHERS CAN STAY THE SAME
  //! Robot odometry subscriber, uses OdomCallback
  ros::Subscriber sub1_;
  //! Laser scan subscriber, uses LaserCallback
  ros::Subscriber sub2_;
  //! Robot odometry subscriber, uses OdomCallback
  ros::Subscriber sub3_;
  //! Laser scan subscriber, uses LaserCallback
  ros::Subscriber sub4_;

  ros::Subscriber sub5_;

  ros::Subscriber sub6_;

  //! Mission service, starts and stops the mission
  ros::ServiceServer service1_;
  //! Advanced goals service, toggles between advanced and basic goals
  ros::ServiceServer service2_;

  //! Pointer to Laser Object
  LaserProcessing* laserProcessingPtr_;
  //! Pointer to Laser Object
  ImageProcessing* imageProcessingPtr_;

  sensor_msgs::LaserScan laserData_;
  std::mutex laserDataMtx_;

  sensor_msgs::Image imageData_;
  std::mutex imageDataMtx_;

  sensor_msgs::CameraInfo cameraInfoData_;
  std::mutex cameraInfoDataMtx_;

  //! Stores the position and orientation of the robot
  geometry_msgs::Pose robotPose_;
  //! Mutex to lock robotPose_
  std::mutex robotPoseMtx_;

  //! Flag for whether the car is moving and the mission is active
  std::atomic<bool> running_; 
  int myInt = 0;
  double turning_ = 0;
  double turningSens_ = 0.001;

  //! Flag for the toggle for using advanced goals
  std::atomic<bool> real_;

  geometry_msgs::Point goal_;
  std::vector<geometry_msgs::Point> goals_;
  double SENSOR_OFFSET_ = 0.12; //estimate
  double STOP_DISTANCE_ = 0.1;
  bool tooClose_;
};

#endif // SAMPLE_H