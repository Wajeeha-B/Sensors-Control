Do you need me to calculate distance based on the current position of the robot and what the laser sees next?
lidar takes angle and find reading at that angle
if so, how do I get the current position of the robot
is the laser reading relative to the correct position of the robot, or aboslute on a co-ordinate plane-- orbot
is the laser pointer only workings on a 180 degree arc-- no
Can you help me use this logic in the sample.cpp?
note find angle min print in ros terminal
print angle min and max as well as increment
when calculating distance find matching angle and index of angle in float- make while/for loop of sorts and iterate through readings
- if angle is within ~0.5 deg its close enough
 this for loop may by similar enough to be helpful 
   unsigned int count = 0;
    //Increments the count variable if the reading is within the max and min range of the laser
    for (int i = 0; i<laserScan_.ranges.size(); i++){
        if(laserScan_.ranges.at(i)>laserScan_.range_min &&
           laserScan_.ranges.at(i)<laserScan_.range_max){
            count++;
           }
    }
    return count;

Turn robot based on scale of 0-100