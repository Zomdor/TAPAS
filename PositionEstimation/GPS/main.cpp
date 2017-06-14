//#include "ros/ros.h"

//#include "TAPAS/GPSInt.h"
//#include "TAPAS/GPSFloat.h"
//#include "TAPAS/GPSBool.h"
//
//#include "TAPAS/GPSgetPosLatitude.h"
//#include "TAPAS/GPSgetPosLongitude.h"
//#include "TAPAS/GPSgetPosX.h"
//#include "TAPAS/GPSgetPosY.h"
//#include "TAPAS/GPSsetZeroXY.h"

#include <sstream>
#include "GPS.h"

//bool getPosLatitude(TAPAS::GPSgetPosLatitude::Request  &req, TAPAS::GPSgetPosLatitude::Response &res);
//bool getPosLongitude(TAPAS::GPSgetPosLongitude::Request  &req, TAPAS::GPSgetPosLongitude::Response &res);
//bool getPosX(TAPAS::GPSgetPosX::Request  &req, TAPAS::GPSgetPosX::Response &res);
//bool getPosY(TAPAS::GPSgetPosY::Request  &req, TAPAS::GPSgetPosY::Response &res);
//bool setZeroXY(TAPAS::GPSsetZeroXY::Request  &req, TAPAS::GPSsetZeroXY::Response &res);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GPS_ROS");
  
  GPS gps;
    
  gps.initController("/dev/ttyACM0", 9600);
  ros::spin();
  
  return 0;
}
