#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

/*!
 *  \brief     Laser Processing Class
 *  \details
 *  This class is designed to process laser data obtained from an LDS-01 sensor mounted 
 *  on a Turtlebot3 Waffle robot. The sensor captures readings in 360-degree increments 
 *  with a precision of 1 degree. The class provides counting valid readings, identifying 
 *  cone segments, finding distances based on angles, calculating turns, and determining 
 *  the magnitude of turns.
 *  @sa Sample
 *  \author    Ashton Powell, Jacinta Kuessner
 *  \version   1.00
 *  \date      31/10/23
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

  /// @brief Takes in the angle of the laser pointer
  /// @returns returns a distance, given the angle
  double FindDistance(double angle);

  /// @brief Takes in an angle and a distance and returns true or false
  /// @returns returns 0 = left, 1 = do nothing, 2 = right
  unsigned int calculateTurn(double angle);

  /// @brief Takes in an angle and a distance
  /// @returns returns magnitude of the turn as a double with a range of 0-360
  double calculateMagnitude(double angle);

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
