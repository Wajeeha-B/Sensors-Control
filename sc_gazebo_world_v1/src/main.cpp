#include "ros/ros.h"
#include "sample.h"
#include <thread>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "sc_gazebo_world");

  //Decleares the node handle
  ros::NodeHandle nh;

  //Creates a new Sample object and inputs the node handle
  std::shared_ptr<Sample> sample(new Sample(nh));
  //Starts a thread using the seperateThread function which communicates with ROS
  std::thread t(&Sample::seperateThread,sample);

  ros::spin();

  ros::shutdown();

  //Joins the thread when the program has concluded
  t.join();

  return 0;
}
