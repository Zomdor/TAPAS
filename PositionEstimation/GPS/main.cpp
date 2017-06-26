#include <sstream>
#include "GPS.h"

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "GPS_ROS");
    // Create GPS object
    GPS gps;
    // connect to the port
    gps.initController("/dev/ttyACM0", 9600);
    ros::spin();
  
  return 0;
}
