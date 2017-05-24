#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "std_msgs/Int16.h"
//msg:
#include "TAPAS/GPSInt.h"
#include "TAPAS/GPSFloat.h"
#include "TAPAS/GPSBool.h"
//srv:
#include "TAPAS/GPSgetPosLatitude.h"
#include "TAPAS/GPSgetPosLongitude.h"
#include "TAPAS/GPSgetPosX.h"
#include "TAPAS/GPSgetPosY.h"
#include "TAPAS/GPSsetZeroXY.h"

#include <sstream>
#include "GPS.h"
bool getPosLatitude(TAPAS::GPSgetPosLatitude::Request  &req, TAPAS::GPSgetPosLatitude::Response &res);
bool getPosLongitude(TAPAS::GPSgetPosLongitude::Request  &req, TAPAS::GPSgetPosLongitude::Response &res);
bool getPosX(TAPAS::GPSgetPosX::Request  &req, TAPAS::GPSgetPosX::Response &res);
bool getPosY(TAPAS::GPSgetPosY::Request  &req, TAPAS::GPSgetPosY::Response &res);
bool setZeroXY(TAPAS::GPSsetZeroXY::Request  &req, TAPAS::GPSsetZeroXY::Response &res);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GPS_ROS");
  ros::NodeHandle n;
  
  // messages:
  ros::Publisher GPSisOpen = n.advertise<TAPAS::GPSBool>("GPSisOpen", 1000);
  ros::Publisher GPSgetLat = n.advertise<TAPAS::GPSFloat>("GPSgetLat", 1000);
  ros::Publisher GPSgetFixStatus = n.advertise<TAPAS::GPSInt>("GPSgetFixStatus", 1000);
  ros::Publisher GPSgetLon = n.advertise<TAPAS::GPSFloat>("GPSgetLon", 1000);
  ros::Publisher GPSisDataValid = n.advertise<TAPAS::GPSBool>("GPSisDataValid", 1000);
  ros::Publisher GPSisSetZero = n.advertise<TAPAS::GPSBool>("GPSisSetZero", 1000);
  
  // services:
  ros::ServiceServer SRVgetPosLatitude = n.advertiseService("GPSgetPosLatitude", getPosLatitude);
  ros::ServiceServer SRVgetPosLongitude = n.advertiseService("GPSgetPosLongitude", getPosLongitude);
  ros::ServiceServer SRVgetPosX = n.advertiseService("GPSgetPosX", getPosX);
  ros::ServiceServer SRVgetPosY = n.advertiseService("GPSgetPosY", getPosY);
  ros::ServiceServer SRVsetZeroXY = n.advertiseService("GPSsetZeroXY", setZeroXY);
  
  ros::Rate loop_rate(10);
  

  while (ros::ok())
  {
    TAPAS::GPSBool msgIsOpen;
    TAPAS::GPSFloat msgGetLat;
    TAPAS::GPSInt msgGetFixStatus;
    TAPAS::GPSFloat msgGetLon;
    TAPAS::GPSBool msgIsDataValid;
    TAPAS::GPSBool msgIsSetZero;

    msgIsOpen.data = true;
    msgIsOpen.header.stamp = ros::Time::now();
    msgIsOpen.header.frame_id = "/GPSisOpen"; 
    
    msgGetLat.data = 3.1;
    msgGetLat.header.stamp = ros::Time::now();
    msgGetLat.header.frame_id = "/GPSgetLat"; 
    
    msgGetFixStatus.data = 4;
    msgGetFixStatus.header.stamp = ros::Time::now();
    msgGetFixStatus.header.frame_id = "/GPSgetFixStatus"; 
    
    msgGetLon.data = 3.2;
    msgGetLon.header.stamp = ros::Time::now();
    msgGetLon.header.frame_id = "/GPSgetLon"; 
    
    msgIsDataValid.data = true;
    msgIsDataValid.header.stamp = ros::Time::now();
    msgIsDataValid.header.frame_id = "/GPSisDataValid"; 
    
    msgIsSetZero.data = false;
    msgIsSetZero.header.stamp = ros::Time::now();
    msgIsSetZero.header.frame_id = "/GPSisSetZero"; 
    
    GPSisOpen.publish(msgIsOpen);
    GPSgetLat.publish(msgGetLat);
    GPSgetFixStatus.publish(msgGetFixStatus);
    GPSgetLon.publish(msgGetLon);
    GPSisDataValid.publish(msgIsDataValid);
    GPSisSetZero.publish(msgIsSetZero);
     
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
bool getPosLatitude(TAPAS::GPSgetPosLatitude::Request  &req, TAPAS::GPSgetPosLatitude::Response &res)
{
    //gps.getPosLatitude(req.X)
    //timestamp
    res.Lat = req.X;
    return true;
}
bool getPosLongitude(TAPAS::GPSgetPosLongitude::Request  &req, TAPAS::GPSgetPosLongitude::Response &res)
{   
    res.Lon = req.X;
    return true;
}
bool getPosX(TAPAS::GPSgetPosX::Request  &req, TAPAS::GPSgetPosX::Response &res)
{
    res.X = req.startLat;
    return true;
}
bool getPosY(TAPAS::GPSgetPosY::Request  &req, TAPAS::GPSgetPosY::Response &res)
{
    res.Y = req.startLon;
    return true;
}
bool setZeroXY(TAPAS::GPSsetZeroXY::Request  &req, TAPAS::GPSsetZeroXY::Response &res)
{
    // void
    return true;
}
