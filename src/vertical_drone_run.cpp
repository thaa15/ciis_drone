#include "Copter_Ciis.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_node");
  ros::NodeHandle nh;
  ros::Rate rate(40);

  CopterCIIS copter(nh,rate);
  copter.arm();
  copter.goFlight();
  nh.shutdown();

  return 0;
}
