#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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
 *  This class is used for processing image data and finding different things such as the locations of the AR tag and the angle of pixels.
 *  This class relies on the Sample class to input the image data collected from a camera. It takes 640x480 images.
 *  @sa Sample
 *  \author    Ashton Powell
 *  \version   1.00
 *  \date      2023-10-29
 */

class ImageProcessing
{
public:
  /// @brief Constructor for laser processing
  ///
  /// @param [in] image - image data to be processed
  /// @param [in] cameraInfo - camera info for for processing
  ImageProcessing(sensor_msgs::Image image, sensor_msgs::CameraInfo cameraInfo);

  /// @brief Uses template matching to find the pixel location of the AR tag
  /// 
  /// @return the x value of the matching pixel location
  int TemplateMatch(void);
  
  /// @brief Gets the angle relative to the camera sensor based on the horizontal pixel location
  ///
  /// @param [in] xPixel - the pixel of the AR tag's horizontal location
  /// @return the angle that the pixel creates from the camera's reference [rad]
  double LocalAngle(int xPixel);

private:
    //! Stores the image data
    sensor_msgs::Image image_;
    //! Stores the image width
    int width_;
    //! Stores the image height
    int height_;
    //! Stores the camera info
    sensor_msgs::CameraInfo cameraInfo_;
    //! Stores the calulcated horizontal FOV of the camera
    double fovX_;
};

#endif //IMAGEPROCESSING_H