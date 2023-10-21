#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

//ROS data types
// #include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
// #include "nav_msgs/Odometry.h"
// #include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"

//We include header of another class we are developing
#include "laserprocessing.h"
#include "imageprocessing.h"
// #include "ackerman.h"

//namespace for the types of markers being generated
// namespace markerType{
//     typedef enum {
//       road, /*!< road */
//       cones, /*!< cones */
//     } obj; /*!< object */
// }

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
  ///
  /// @brief Constructor of the Sample class.
  ///
  /// Sets the default values of variables such as the robot position, the goals, the running_ boolean, etc.
  /// Requires the NodeHandle input to communicate with ROS.
  Sample(ros::NodeHandle nh);

  /// @brief Destructor of the Sample class.
  ///
  /// Deletes the object pointers for laserprocessing and ackerman classes.
  ~Sample();

  /// @brief Creates markers based off their location and type, either being a road marker or cone marker.
  /// @param [in] point the x,y,z location of the marker.
  /// @param [in] type the size and colour of the marker being a green cube or an orange cylinder (cone).
  /// @return a marker variable to be publish in the simulator.
//   visualization_msgs::Marker createMarker(geometry_msgs::Point point, markerType::obj type);

  /// @brief converts a point variable to a pose variable, used specifically for cones.
  /// @param [in] cone the x,y,z location of a cone that was detected from @sa laserprocessing.
  /// @return pose variable with the same x,y,z coordinates of the point but with added orientation.
//   geometry_msgs::Pose createCone(geometry_msgs::Point cone);
  
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
//   bool advanced(std_srvs::SetBool::Request  &req,
//                 std_srvs::SetBool::Response &res);

private:

  /// @brief Odometry Callback from the world reference of the UGV
  ///
  /// @param [in|out] msg nav_msgs::OdometryConstPtr - The odometry message
  /// @note This function and the declaration are ROS specific
//   void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  /// @brief Laser Callback from the laser sensor's reference in front of the UGV
  /// @param [in|out] msg sensor_msgs::LaserScanConstPtr - the laser scan data
  /// @note This function and the declaration are ROS specific
//   void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);
  
  /// @brief Goal Callback from the world reference from the simulator environment
  /// @param [in|out] msg geometry_msgs::PoseArrayConstPtr - the array of goals
  /// @note This function and the declaration are ROS specific
//   void goalCallback(const geometry_msgs::PoseArrayConstPtr& msg);
  
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  

  //! Node handle for communication
  ros::NodeHandle nh_;
//   //! Visualisation Marker publisher
//   ros::Publisher pubVis_;
//   //! UGV brakes publisher
//   ros::Publisher pubBrake_;
//   //! UGV steering wheel angle publisher
//   ros::Publisher pubSteer_;
//   //! UGV throttle publisher
//   ros::Publisher pubThrottle_;
//   //! Cones location publisher
//   ros::Publisher pubCones_;


  ros::Publisher pubDrive_;

  // PUBLISHERS CAN STAY THE SAME
  //! Robot odometry subscriber, uses OdomCallback
  ros::Subscriber sub1_;
  //! Laser scan subscriber, uses LaserCallback
  ros::Subscriber sub2_;

  //! Mission service, starts and stops the mission
  ros::ServiceServer service1_;
//   //! Advanced goals service, toggles between advanced and basic goals
//   ros::ServiceServer service2_;

  //! Pointer to Laser Object
//   LaserProcessing* laserProcessingPtr_;
//   //! Pointer to Ackerman Object
//   Ackerman* ackermanPtr_;

  //! Pointer to Laser Object
  LaserProcessing* laserProcessingPtr_;
  //! Pointer to Laser Object
  ImageProcessing* imageProcessingPtr_;

//   //! Stores the position and orientation of the robot
//   geometry_msgs::Pose robotPose_;
//   //! Mutex to lock robotPose_
//   std::mutex robotPoseMtx_;

//   //! Stores the data of the laser sensor
//   sensor_msgs::LaserScan laserData_;
//   //! Mutex to lock Laser Data
//   std::mutex laserDataMtx_;

//   //! Stores the array of basic predetermined goals
//   geometry_msgs::PoseArray goals_;
//   //! Mutex to lock goal data
//   std::mutex goalsMtx_;

  sensor_msgs::LaserScan laserData_;
  std::mutex laserDataMtx_;

  sensor_msgs::Image imageData_;
  std::mutex imageDataMtx_;

//   //! Provides the unique ID of the markers
//   unsigned int marker_counter_;
//   //! Stores the current goal
//   geometry_msgs::Point goal_;
  //! Flag for whether the car is moving and the mission is active
  std::atomic<bool> running_; 
  int myInt = 0;
  double turning = 0;
  double turningSens = 0.001;
//   //! Flag for is the goal is invalid
//   std::atomic<bool> goalOK_;
//   //! Flag for the toggle for using advanced goals
//   std::atomic<bool> advGoals_;
//   //! The distance used to determined if a goal is reach and the next one can be used
//   float STOP_DISTANCE_ = 0.5;
};

#endif // SAMPLE_H