#include "imageprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;
// using namespace cv;

//Default constructor of ImageProcessing, needs an initial set of image data to be initialised
ImageProcessing::ImageProcessing(sensor_msgs::Image image, sensor_msgs::CameraInfo cameraInfo):
    image_(image), cameraInfo_(cameraInfo), fovX_(0.0){}

//Uses the template matching function from OpenCV and converts it into a horizontal value for the pixel
int ImageProcessing::TemplateMatch(){
    //Stores the image dimensions and encoding
    width_ = image_.width;
    height_ = image_.height;
    std::string encoding = image_.encoding;

    //Initialises the img variable from OpenCV
    cv::Mat img;

    //Configure the cv::Mat based on the encoding information
    if (encoding == "8UC1") img = cv::Mat(height_, width_, CV_8UC1);
    else if (encoding == "8UC3") img = cv::Mat(height_, width_, CV_8UC3);
    else if (encoding == "bgr8") img = cv::Mat(height_, width_, CV_8UC3);
    else if (encoding == "rgb8") img = cv::Mat(height_, width_, CV_8UC3);
    else{
        //Handle other encodings if needed
        ROS_ERROR("Unsupported image encoding: %s", encoding.c_str());
    }
    //Makes sure that the image has been successfully copied into the code
    assert(!img.empty() && "File could not be read, check with cv::imread()");
    memcpy(img.data, &image_.data[0], img.total() * img.elemSize());
    cv::Mat img2 = img.clone();

    //Copies the template image into the code
    cv::Mat templateImg = cv::imread("catkin_ws/src/Sensors-Control/sc_gazebo_world/src/tag36_11_00000rqt.jpg", cv::IMREAD_COLOR);
    //Makes sure that the image has been successfully copied into the code
    assert(!templateImg.empty() && "File could not be read, check with cv::imread()");

    //Stores the width and height of the template image
    int w = templateImg.cols;
    int h = templateImg.rows;
    //Sets the method for template matching, this being SQDIFF
    int method = 0;

    //Uses template matching and stores the result
    cv::Mat result;
    cv::matchTemplate(img, templateImg, result, method);

    //Finds the min and max value inside the resulting image of template matching
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

    //Sets the matching location to the minimum value of the resulting image and draws a rectangle around it
    cv::Point matchLoc = minLoc;
    cv::rectangle(img, matchLoc, cv::Point(matchLoc.x + w, matchLoc.y + h), cv::Scalar(255), 2);

    //Generates the image from the camera with the rectangle around the match
    cv::Mat displayResult;
    cv::normalize(result, displayResult, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
    // cv::imshow("Matching Result", displayResult);
    // cv::imshow("Detected Point", img);
    // cv::waitKey(0);

    return matchLoc.x; //Returns the x value of the matched location
}

//Finds the angle that the pixel forms with the camera from the local reference
double ImageProcessing::LocalAngle(int xPixel){
    //Sets the FOV if it has not been done already
    if (fovX_ == 0){
        double fX;
        fX = cameraInfo_.K[0];
        // ROS_INFO("CameraInfo: [%f,%f,%f,%f,%f,%f,%f,%f,%f]", cameraInfo_.K[0], cameraInfo_.K[1], cameraInfo_.K[2], cameraInfo_.K[3], cameraInfo_.K[4], cameraInfo_.K[5], cameraInfo_.K[6], cameraInfo_.K[7], cameraInfo_.K[8]);
        // ROS_INFO("K Value: %f", fX);

        //Calculates the FOV using the width of the image and the focal length
        double fovX = 2*atan2(width_, 2.0*fX);
        fovX_ = fovX*180/M_PI; //degrees
        fovX_ = fovX; //radians
        // ROS_INFO("FOV Value: %f", fovX);
    }
    double x = 275.0 - xPixel; //Centres the value to 275 (assuming 275 is the middle)
    double d = (width_/2)/(tan(fovX_/2));
    double theta = atan(x/d);
    // ROS_INFO("angle: %f", theta);
    return theta;
}