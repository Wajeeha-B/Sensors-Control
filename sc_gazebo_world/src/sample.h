#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

//ROS data types
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
 *  It is designed to use laserprocessing to interpret laser data and imageprocessing to interpret image data.
 *  This information is used to generate an input for the control or the Turtlebot to follow the AR tag.
 *  \author    Ashton Powell
 *  \version   1.00
 *  \date      2023-10-29
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
  /// Deletes the object pointers for laserprocessing and imageprocessing classes.
  ~Sample();
  
  /// @brief seperate thread.
  ///
  /// The main processing thread that will run continously and utilise the data.
  /// When data needs to be combined then running a thread seperate to callback will guarantee data is processed.
  /// The data is then used to publish input to move the TurtleBot, causing new data to be generated on its updated position and perspective.
  void seperateThread();

  /// @brief request service callback for starting and stopping the mission and Turtlebot's movement.
  ///
  /// @param [in] req The request, a boolean value where true means the mission is in progress and false stops the mission.
  /// @param [in] res The response, a boolean and string value indicating if starting the mission was successful.
  ///
  /// @return bool - Will return true to indicate the request succeeded.
  bool request(std_srvs::SetBool::Request  &req,
               std_srvs::SetBool::Response &res);
  
  /// @brief request service callback for toggling between publishing to simulated and real topics.
  ///
  /// @param [in] req the request, if true the program publishes to real topics.
  /// @param [in] res the response, shares the current set of topics being published to.
  ///
  /// @return a boolean value to indicate the request was successful.
  bool real(std_srvs::SetBool::Request  &req,
            std_srvs::SetBool::Response &res);

  /// @brief Converts the goal from the laser's reference to the global reference.
  ///
  /// @param [in] goal a geometry_msgs::Point that is the location in x,y,z for the goal.
  /// @param [in] robot a geometry_msgs::Pose that is the current position and orientation of the TurtleBot.
  ///
  /// @return a geometry_msgs::Point which is the x,y,z location of the goal.
  geometry_msgs::Point local2Global(geometry_msgs::Point goal, geometry_msgs::Pose robot);

  /// @brief Getter for distance to be travelled to reach goal, updates as the platform moves to current goal.
  ///
  /// @param [in] goal a geometry_msgs::Point that is the current goal in x,y,z for the Turtlebot.
  /// @param [in] robot a geometry_msgs::Pose that is the current position and orientation of the Turtlebot.
  ///
  /// @return distance of the goal to the Turtlebot in a straight line [m].
  double DistanceToGoal(geometry_msgs::Point goal, geometry_msgs::Pose robot);

  /// @brief Getter for distance between consecutive goals
  ///
  /// @param [in] goal1 a geometry_msgs::Point that is the first goal in x,y,z for the Turtlebot.
  /// @param [in] goal2 a geometry_msgs::Point that is the second goal in x,y,z for the Turtlebot.
  ///
  /// @return distance between consecutive goals in a straight line [m].
  double DistanceBetweenGoals(geometry_msgs::Point goal1, geometry_msgs::Point goal2);

  /// @brief Gets the angle of the required turn to follow a straight line towards the next goal.
  ///
  /// @param [in] goal a geometry_msgs::Point that is the current goal in x,y,z for the TurtleBot.
  /// @param [in] robot a geometry_msgs::Pose that is the current position and orientation of the TurtleBot.
  ///
  /// @return a double value for the angle of the required turn of the TurtleBot [rad].
  double GetSteering(geometry_msgs::Point goal, geometry_msgs::Pose robot);

private:
  /// @brief Laser Callback from the laser sensor's reference
  ///
  /// @param [in|out] msg sensor_msgs::LaserScanConstPtr - the laser scan data
  /// @note This function and the declaration are ROS specific
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /// @brief Image Callback from the camera's reference
  ///
  /// @param [in|out] msg sensor_msgs::ImageConstPtr - the image data
  /// @note This function and the declaration are ROS specific
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /// @brief Camera Info Callback
  ///
  /// @param [in|out] msg sensor_msgs::CameraInfoConstPtr - the camera info data
  /// @note This function and the declaration are ROS specific
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

  /// @brief Odometry Callback from the world reference of the TurtleBot
  ///
  /// @param [in|out] msg nav_msgs::OdometryConstPtr - The odometry message
  /// @note This function and the declaration are ROS specific
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  //! Node handle for communication
  ros::NodeHandle nh_;
  //! Driving command publisher
  ros::Publisher pubDrive_;
  //! Driving command for the real robot publisher
  ros::Publisher pubRealDrive_;
  //! Laser scan subscriber, uses LaserCallback
  ros::Subscriber sub1_;
  //! Image subscriber, uses ImageCallback
  ros::Subscriber sub2_;
  //! Laser scan subscriber for the real robot, uses LaserCallback
  ros::Subscriber sub3_;
  //! Image subscriber for the real robot, uses ImageCallback
  ros::Subscriber sub4_;
  //! Camera Info subscriber, uses CameraInfoCallback
  ros::Subscriber sub5_;
  //! Robot odometry subscriber, uses OdomCallback
  ros::Subscriber sub6_;

  //! Mission service, starts and stops the mission
  ros::ServiceServer service1_;
  //! real topics service, toggles between simulated and real topics
  ros::ServiceServer service2_;

  //! Pointer to Laser Object
  LaserProcessing* laserProcessingPtr_;
  //! Pointer to Image Object
  ImageProcessing* imageProcessingPtr_;

  //! Stores the laser data from the LIDAR scanner
  sensor_msgs::LaserScan laserData_;
  //! Mutex to lock laserData_
  std::mutex laserDataMtx_;

  //! Stores the image data from the camera sensor
  sensor_msgs::Image imageData_;
  //! Mutex to lock imageData_
  std::mutex imageDataMtx_;

  //! Stores the camera's info and parameters
  sensor_msgs::CameraInfo cameraInfoData_;
  //! Mutex to lock cameraInfoData_
  std::mutex cameraInfoDataMtx_;

  //! Stores the position and orientation of the robot
  geometry_msgs::Pose robotPose_;
  //! Mutex to lock robotPose_
  std::mutex robotPoseMtx_;

  //! Flag for whether the car is moving and the mission is active
  std::atomic<bool> running_;
  //! Stores the value of the amount of turning for the robot
  double turning_ = 0;
  //! Adjusts the sensitivity for the turning so the robot doesnt whip around violently when turning
  double turningSens_ = 1;
  //! Flag for the toggle for using advanced goals
  std::atomic<bool> real_;
  //! Stores a goal for the robot to move towards
  geometry_msgs::Point goal_;
  //! Stores a series of goals in the order they occur
  std::vector<geometry_msgs::Point> goals_;
  //! The offset between the reference of the TurtleBot and the reference of the laser scanner
  double SENSOR_OFFSET_ = 0.12;
  //! The stop distance to stop the following TurtleBot before it collides with the guiding TurtleBot
  double STOP_DISTANCE_ = 0.3;
  //! Boolean for stopping the TurtleBot when it becomes too close to the guiding TurtleBot
  bool tooClose_;
};

#endif // SAMPLE_H