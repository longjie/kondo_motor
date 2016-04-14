/**
 * @brief configurator for kondo driver
 */
#include <stdio.h>

extern "C" {
  #include "kondo_driver/ics_serial.h"
}

int main (int argc, char **argv)
{
  // Init ROS node
  ros::init (argc, argv, "kondo_configurator");
  ros::NodeHandle nh;
  
}
