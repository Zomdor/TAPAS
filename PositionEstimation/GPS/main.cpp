#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "TAPAS/GPSInt.h"
#include "TAPAS/GPSFloat.h"
#include "TAPAS/GPSBool.h"
//#include "GPS.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher GPSisOpen = n.advertise<TAPAS::GPSBool>("GPSisOpen", 1000);
  ros::Publisher GPSgetLat = n.advertise<TAPAS::GPSFloat>("GPSgetLat", 1000);
  ros::Publisher GPSgetFixStatus = n.advertise<TAPAS::GPSInt>("GPSgetFixStatus", 1000);
  ros::Publisher GPSgetLon = n.advertise<TAPAS::GPSFloat>("GPSgetLon", 1000);
  ros::Publisher GPSisDataValid = n.advertise<TAPAS::GPSBool>("GPSisDataValid", 1000);
  ros::Publisher GPSisSetZero = n.advertise<TAPAS::GPSBool>("GPSisSetZero", 1000);
  
  ros::Rate loop_rate(10);

  int count = 0;
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
    ++count;
  }
  return 0;
}
