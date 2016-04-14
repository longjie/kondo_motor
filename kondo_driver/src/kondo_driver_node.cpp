/**
 * @file kondo_driver_node.cpp
 * 
 * Device driver node of KONDO ICS servo motors.
 */
#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "kondo_driver/kondo_driver.h"

/**
 * @brief Main function
 */
int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "kondo_driver");
  ros::NodeHandle nh;

  // Create hardware interface 
  KondoDriver robot(argc - 1, &argv[1]);

  // Connect to controller manager
  controller_manager::ControllerManager cm(&robot, nh);

  // Set spin ratge
  ros::Rate rate(1.0 / ros::Duration(0.050).toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok()) {
    cm.update(ros::Time::now(), ros::Duration(0.050));
    robot.update();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
