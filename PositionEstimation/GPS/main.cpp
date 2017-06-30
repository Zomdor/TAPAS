#include <sstream>
#include "GPS.h"

int main(int argc, char **argv)
{
	//ROS_GPS ROS initialization for GPS node	
  	ros::init(argc, argv, "GPS_ROS");
  	//ROS_GPS create GPS object
  	GPS gps;
    	//ROS_GPS connect to the port
  	gps.initController("/dev/robots/gps", 9600);
  	ros::spin();
  
  return 0;
}
