#include "ros/ros.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/actuator_command_interface.h"
#include "hardware_interface/actuator_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "kondo_msgs/setPower.h"
#include "kondo_msgs/getParameters.h"
extern "C" {
#include "kondo_driver/ics_serial.h"
}

#define FLIP_STEP (2)

/* Maximum motor num (32 is maximum on spec. sheet) */
const int MAX_MOTOR_NUM = 32;

const int MAX_PULSE = 11500; // maximum pulse width
const int MIN_PULSE = 3500; // minimum pulse width
const int CNT_PULSE = 7500; // center pulse width

/* Ratio to radian to pulse width */
const double RADIAN_PER_PULSE = 270.0 * M_PI / (MAX_PULSE - MIN_PULSE) / 180.0;

inline double pulse_to_radian (double pulse)
{
  return (pulse - CNT_PULSE) * RADIAN_PER_PULSE;
}

/**
 * @brief convert the angle radian to servo pulse value
 */
inline int radian_to_pulse (double radian)
{
  return CNT_PULSE + radian / RADIAN_PER_PULSE;
}

/**
 * @brief class for individual servo motor
 */
class KondoMotor
{
private:
  bool loopback; /// loopback mode
  bool motor_power; /// motor power (servo on/off)
  std::string motor_name;
  int id; /// ICS id
  int ics; /// fd of ICS device
  int stretch; /// stretch parameter
  int speed; /// speed parameter
  int curr_limit; /// current limit paramter
  int temp_limit; /// temperature limit parameter [deg. of cersius]
  int min_angle; /// joint minimum angle [deg]
  int max_angle; /// joint maximum angle [deg]
  uint8_t eeprom[64]; /// EEPROM buffer of the ICS servo motor

  ros::ServiceServer power_service; /// setPower service
  ros::ServiceServer get_param_service; /// getParam service

public:
  double cmd_pos; /// inteface value from joint position controller
  double cmd_vel; /// inteface value from joint velocity controller
  double pos, vel, eff; /// inteface values of robot hardware
  double pos_prev, pos_curr, pos_buff[FLIP_STEP];
  int round;
  int skip_count;
  std::string joint_name; /// name of joint

  int controller_type;

  KondoMotor (int ics, std::string actuator_name,
              hardware_interface::JointStateInterface & state_interface,
              hardware_interface::PositionJointInterface & pos_interface,
              hardware_interface::VelocityJointInterface & vel_interface,
              bool loopback);


  void set_stretch (unsigned char stretch);
  void set_current_limit (unsigned char curr);
  void set_temperature_limit (unsigned char temp);
  void set_speed (unsigned char speed);

  void get_param_from_eeprom(void);
  void set_param_to_eeprom(void);

  void update (void);
  void update_position (void);
  void update_velocity (void);

  // ROS service call
  bool set_power (kondo_msgs::setPower::Request & req, kondo_msgs::setPower::Response & res);
  bool get_param_from_eeprom(kondo_msgs::getParameters::Request & req, kondo_msgs::getParameters::Response & res);
};
