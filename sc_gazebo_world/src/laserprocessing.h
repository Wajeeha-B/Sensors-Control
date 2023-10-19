#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
// #include <geometry_msgs/Pose.h>
#include <math.h>
// #include "tf/transform_datatypes.h"
#include <iostream>
#include "ros/ros.h"

/*!
 *  \brief     Laser Processing Class
 *  \details
 *  This class is used for processing laser scan data and finding different things such as the locations of cones,
 *  the location of a goal, determining if a goal is between two cones, and converting laser data into different data types
 *  for other classes to understand. This class relies on the Sample class to input the laser data collected from a laser scanner
 *  inside a simulator that detects and measures object and their proximity to the sensor. It takes 640 readings in a 180 arc at
 *  the front of the car and has limited range.
 *  @sa Sample
 *  \author    Ashton Powell
 *  \version   1.00
 *  \date      2023-05-30
 */

class LaserProcessing
{
public:
  /// @brief Constructor for laser processing
  /// @param [in] laserScan - laserScan to be processed
  LaserProcessing(sensor_msgs::LaserScan laserScan);
  
  /// @brief Counts the number of valid readings from the laser scanner
  /// @return The number of readings
  unsigned int countObjectReadings();

  /// @brief Counts the number of segments of valid readings from the laser scanner
  /// @return The number of segments that are cones
  unsigned int countSegments();

  void myFunction(int myInt);

  /// @brief Finds the midpoint between the two closest cones that are detected by the laser scanner
  /// @return A geometry_msgs::Point variable with the x,y,z location of the midpoint
  // geometry_msgs::Point detectRoadCentre();

  /// @brief Checks if the predetermined goal is within the area between the two closest cones
  /// @param [in] goal the predetermined goal of interest
  /// @return a boolean value, true indicating the goal is between the cones
  // bool GoalInCones(geometry_msgs::Point goal);

  /// @brief Gets the cones store in cones_ and converts them from a vector pair to a vector of geometry_msgs::Point
  /// @return std::vector<geometry_msgs::Point> of all cones detected
  // std::vector<geometry_msgs::Point> getCones();

private:
    //! Stores the laser scan data
    sensor_msgs::LaserScan laserScan_;
    //! Defines the range to identify a cone in segment readings
    float CONE_RANGE_ = 0.3;
    //! Stores a vector of cones as x and y coordinate pairs
    std::vector<std::pair<float, float>> cones_;

    // //! Stores the two closest cones
    // geometry_msgs::Point cone1_, cone2_;
};

#endif // DETECTCABINET_H