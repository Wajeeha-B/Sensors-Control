#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
// #include <geometry_msgs/Pose.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <iostream>
#include "ros/ros.h"

/*!
 *  \brief     Image Processing Class
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

class ImageProcessing
{
public:
  /// @brief Constructor for laser processing
  /// @param [in] Image - laserScan to be processed
  ImageProcessing(sensor_msgs::Image image, sensor_msgs::CameraInfo cameraInfo);

  int TemplateMatch(void);
  
  double LocalAngle(int xPixel);

private:
    //! Stores the laser scan data
    sensor_msgs::Image image_;
    int width_;
    int height_;
    sensor_msgs::CameraInfo cameraInfo_;
    double fovX_;
};

#endif // DETECTCABINET_H