#include "imageprocessing.h"
#include <algorithm>
#include <numeric>
// #include "std_msgs/Float64.h"
using namespace std;
// using namespace cv;

//Default constructor of Laserprocessing, needs an initial set of laser data to be initialised
ImageProcessing::ImageProcessing(sensor_msgs::Image image, sensor_msgs::CameraInfo cameraInfo):
    image_(image), cameraInfo_(cameraInfo), fovX_(0.0){}

int ImageProcessing::TemplateMatch(){
    width_ = image_.width;
    height_ = image_.height;
    std::string encoding = image_.encoding;

    cv::Mat img;

    // Configure the cv::Mat based on the encoding information
    if (encoding == "8UC1") img = cv::Mat(height_, width_, CV_8UC1);
    else if (encoding == "8UC3") img = cv::Mat(height_, width_, CV_8UC3);
    else if (encoding == "bgr8") img = cv::Mat(height_, width_, CV_8UC3);
    else if (encoding == "rgb8") img = cv::Mat(height_, width_, CV_8UC3);
    else{
        // Handle other encodings if needed
        ROS_ERROR("Unsupported image encoding: %s", encoding.c_str());
    }

    assert(!img.empty() && "File could not be read, check with cv::imread()");
    memcpy(img.data, &image_.data[0], img.total() * img.elemSize());
    cv::Mat img2 = img.clone();
    //Starting from the username folder
    cv::Mat templateImg = cv::imread("catkin_ws/src/Sensors-Control/sc_gazebo_world/src/tag36_11_00000rqt.jpg", cv::IMREAD_COLOR);
    assert(!templateImg.empty() && "File could not be read, check with cv::imread()");

    int w = templateImg.cols;
    int h = templateImg.rows;
    int method = 0;

    cv::Mat result;
    cv::matchTemplate(img, templateImg, result, method);

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

    cv::Point matchLoc = minLoc;
    // ROS_INFO("\nmatchLoc.x = %d\nmatchLoc.y = %d", matchLoc.x, matchLoc.y);
    cv::rectangle(img, matchLoc, cv::Point(matchLoc.x + w, matchLoc.y + h), cv::Scalar(255), 2);

    cv::Mat displayResult;
    cv::normalize(result, displayResult, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
    // cv::imshow("Matching Result", displayResult);
    // cv::imshow("Detected Point", img);
    // cv::waitKey(0);
    // if(matchLoc.x != 275) return 275-matchLoc.x;
    // else return 0;
    return matchLoc.x;
}

double ImageProcessing::LocalAngle(int xPixel){
    if (fovX_ == 0){
        double fX;
        fX = cameraInfo_.K[0];
        // ROS_INFO("CameraInfo: [%f,%f,%f,%f,%f,%f,%f,%f,%f]", cameraInfo_.K[0], cameraInfo_.K[1], cameraInfo_.K[2], cameraInfo_.K[3], cameraInfo_.K[4], cameraInfo_.K[5], cameraInfo_.K[6], cameraInfo_.K[7], cameraInfo_.K[8]);
        // ROS_INFO("K Value: %f", fX);

        double fovX = 2*atan2(width_, 2.0*fX);
        fovX_ = fovX*180/M_PI; //degrees
        fovX_ = fovX; //radians
        // ROS_INFO("FOV Value: %f", fovX);
    }
    double x = 275.0 - xPixel; //Centres the value to 320 (assuming 275 is the middle)
    // xPixel = i - 275.0;
    double d = (width_/2)/(tan(fovX_/2));
    double theta = atan(x/d);
    // ROS_INFO("angle: %f", theta);
    return theta;
}