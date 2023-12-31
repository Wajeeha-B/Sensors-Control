#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

//Default constructor of Laserprocessing, needs an initial set of laser data to be initialised
LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan){}

//Counts the number of valid readings from the laser bouncing off an object
unsigned int LaserProcessing::countObjectReadings()
{
    unsigned int count = 0;
    //Increments the count variable if the reading is within the max and min range of the laser
    for (int i = 0; i<laserScan_.ranges.size(); i++){
        if(laserScan_.ranges.at(i)>laserScan_.range_min &&
           laserScan_.ranges.at(i)<laserScan_.range_max){
            count++;
           }
    }
    return count;
}

// This estimates the distance at a specific angle if there is no exact match
double LaserProcessing::FindDistance(double targetAngle){
    // Calculate the index corresponding to the target angle
    int index = (targetAngle - laserScan_.angle_min) / laserScan_.angle_increment;

    // Calculate the float index corresponding to the target angle
    float floatIndex = (targetAngle - laserScan_.angle_min) / laserScan_.angle_increment;

    // Get the indices of the surrounding angles
    int lowerIndex = static_cast<int>(floor(floatIndex));
    int upperIndex = static_cast<int>(ceil(floatIndex));

    // Check if the indices are within the bounds of the ranges array
    if (lowerIndex >= 0 && upperIndex < laserScan_.ranges.size()) {
        // Linearly interpolate the distance at the target angle
        float alpha = floatIndex - lowerIndex;
        double interpolatedDistance = (1.0 - alpha) * laserScan_.ranges[lowerIndex] + alpha * laserScan_.ranges[upperIndex];
        return interpolatedDistance;
    }

    // Eror handling for out of bounds
    if (index < 0) index = index+360;
    return laserScan_.ranges[index]; 
};

//Counts the number of segments of readings to indicate a single entity being detected, excludes the firetruck
unsigned int LaserProcessing::countSegments()
{
    //initialises the variables that monitors the amount of segments, number of readings in 1 segment,
    //the start of a segment and the x and y values of the reading
    unsigned int count = 0;
    unsigned int pointCount = 0;
    bool seg_start = false;
    float xVal = 0.0;
    float yVal = 0.0;
    for (int i = 1; i<laserScan_.ranges.size(); i++)
    {
        //if the previous reading is within the cone_range_ in metres of the current reading, it is part of the same entity being scanned
        if(fabs(laserScan_.ranges.at(i-1)-laserScan_.ranges.at(i)) < CONE_RANGE_)
        {
            //positively triggered if statement to indicate that 1 segment is detected
            if(!seg_start){
                seg_start = true;
                count++;
            }
            //Continuously adds the total x and y values from the segment readings
            xVal += laserScan_.ranges.at(i-1)*cos(laserScan_.angle_min + laserScan_.angle_increment*(i-1));
            yVal += laserScan_.ranges.at(i-1)*sin(laserScan_.angle_min + laserScan_.angle_increment*(i-1));
            //counts the number of readings in 1 segment
            pointCount++;
        }
        //if the reading is outside the cone_range_, the segment has ended and the data from that specific segment can be processed
        else
        {
            //Checks if the segment is larger than 1 read, otherwise its an invalid segment
            if(pointCount > 0)
            {
                //finds the average x and y values to determine the approximate location of the cone
                float xAvg = xVal/pointCount;
                float yAvg = yVal/pointCount;

                //finds the index of the start of the segment
                unsigned int Startidx = (i-1)-pointCount;
                //converts the laser data into x and y coordinates
                float xStart = laserScan_.ranges.at(Startidx)*cos(laserScan_.angle_min + laserScan_.angle_increment*(Startidx));
                float yStart = laserScan_.ranges.at(Startidx)*sin(laserScan_.angle_min + laserScan_.angle_increment*(Startidx));
                //converts the laser data of the last reading of the segment into x and y coordinates
                float xEnd = laserScan_.ranges.at(i-1)*cos(laserScan_.angle_min + laserScan_.angle_increment*(i-1));
                float yEnd = laserScan_.ranges.at(i-1)*sin(laserScan_.angle_min + laserScan_.angle_increment*(i-1));
                //finds the distance between the start and end of a segment
                float radius = sqrt(pow(xStart-xEnd,2)+pow(yStart-yEnd,2));
                //if the segment is detected and the radius is larger than the size of a cone it is assumed to be not a valid cone
                if(radius > CONE_RANGE_ && seg_start) count--;
                //otherwise the segment detected is a cone and the local x and y values are added to the vector pair
                else cones_.push_back({xAvg,yAvg});
                //counting variables are reset to 0 for the next segment
                pointCount = 0;
                xVal = 0.0;
                yVal = 0.0;
            }
            //a segment is no longer detected
            seg_start = false;
        } 
    }
    //returns the total amount of segments detected
    return count;
}

std::pair<double, double> LaserProcessing::MinDistAngle(double stopDistance){
    double min_distance = laserScan_.range_max;  // Initialize with a large value
    double angle_of_min_distance = 0.0;  // Initialize the angle at which the minimum distance occurs

    // Find the minimum valid distance in the laser scan data- 1st obstacle
    for (size_t i = 0; i < laserScan_.ranges.size(); ++i) {
        double angle = laserScan_.angle_min + i * laserScan_.angle_increment;  // Calculate the angle of the current reading
        angle = fmod(angle + 2*M_PI, 2*M_PI) * (180.0 / M_PI);  // Normalize angle to [0, 360) degrees

        if (laserScan_.ranges[i] < min_distance &&
            ((angle >= 0 && angle <= 90) || (angle >= 270 && angle <= 359)) &&
            laserScan_.ranges[i] < stopDistance) {
            min_distance = laserScan_.ranges[i];
            angle_of_min_distance = angle;  // Update the angle at which the minimum distance occurs
        }
    }
    return std::make_pair(min_distance,angle_of_min_distance);
}

//finds the midpoint between two of the closest cones detected, one from the left and one from the right
// geometry_msgs::Point LaserProcessing::detectRoadCentre(){
//     geometry_msgs::Point pt;
//     //Uses the countSegments function to populate the cones_ vector pair.
//     unsigned int count = countSegments();
//     //It is necessary to have at least 2 cones detected to find the midpoint so it will return an invalid goal if there is less than 2 cones
//     if (cones_.size() < 2){
//       ROS_INFO_STREAM("Not enough cones!");
//       pt.z = 999;
//       return pt;
//     }

//     //shortRange records the closest cone detected and starts at the maximum value of the laser sensor
//     float shortRange = laserScan_.range_max;
//     //stores the index of the 2 closest cones
//     unsigned int a = 0, b = 0;
    
//     //finds the closest cone to the right of the car
//     for(int i = 0; i < cones_.size(); i++)
//     {
//         //measures the range from the laser sensor and the location of the cone
//         float r = sqrt(pow(cones_.at(i).first,2)+pow(cones_.at(i).second,2));
//         //if the range is shorter than the value of shortRange, a new shortest range is set to shortRange and the index of the cone is saved
//         //it also ensures that this cone is to the right of the laser sensor/car by having a negative y value
//         if(r < shortRange && cones_.at(i).second < 0){
//           shortRange = r;
//           a = i;
//         }
//     }
//     //the shortRange variable is reset to the max range to find the next closest cone
//     shortRange = laserScan_.range_max;

//     //finds the closest cone to the left of the car
//     for(int j = 0; j < cones_.size(); j++)
//     {
//         //if the index j matches the index of the first closest cone found, continue to the next iteration
//         if(j == a) continue;
//         //measures the range from the laser sensor and the location of the cone
//         float r = sqrt(pow(cones_.at(j).first,2)+pow(cones_.at(j).second,2));
//         //if the range is shorter than the value of shortRange, a new shortest range is set to shortRange and the index of the cone is saved
//         //it also ensures that this cone is to the left of the laser sensor/car by having a positive y value
//         if(r < shortRange && cones_.at(j).second > 0){
//           shortRange = r;
//           b = j;
//         }
//     }
//     //the location of the 2 closest cones are stored for use later
//     cone1_.x = cones_.at(a).first;
//     cone1_.y = cones_.at(a).second;
//     cone2_.x = cones_.at(b).first;
//     cone2_.y = cones_.at(b).second;

//     //the x and y values of the 2 closest cone's is averaged using the index in the vector to find the midpoint which is the goal
//     pt.x = (cones_.at(a).first+cones_.at(b).first)/2;
//     pt.y = (cones_.at(a).second+cones_.at(b).second)/2;
//     pt.z = 0.0;
//     return pt;
// }

// //Used to check if the goal is within the cones, not actively used for navigation or generating goals
// bool LaserProcessing::GoalInCones(geometry_msgs::Point goal){
//     geometry_msgs::Point centre;
//     //finds the midpoint between the 2 cones
//     centre.x = (cone1_.x+cone2_.x)/2;
//     centre.y = (cone1_.y+cone2_.y)/2;
//     //finds the distance between the midpoint and the first cone
//     float radius = sqrt(pow(centre.x-cone1_.x,2)+pow(centre.y-cone1_.y,2));
//     //finds the distance between the midpoint and the goal
//     float centreGoalDiff = sqrt(pow(centre.x-goal.x,2)+pow(centre.y-goal.y,2));
//     //if the distance between the midpoint and the goal is greater than the midpoint and the goal, then it is considered not between the cones
//     if(centreGoalDiff > radius) return false;
//     //otherwise the goal is between the cones
//     return true;
// }

// //Allows other classes to access the array of points of the detected cones
// std::vector<geometry_msgs::Point> LaserProcessing::getCones(){
//     std::vector<geometry_msgs::Point> cones;
//     geometry_msgs::Point cone;
//     //iterates through the cones_ vector pair and converts them into a vector of geometry_msgs::Point variables with 0.0 z value
//     for(auto elem : cones_){
//         cone.x = elem.first;
//         cone.y = elem.second;
//         cone.z = 0.0;
//         cones.push_back(cone);
//     }
//     return cones;
// }
