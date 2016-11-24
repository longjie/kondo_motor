/**
 * @file kondo_motor.cpp
 * @author Ryosuke Tajima
 * Kondo ICS motor driver
 */
#include "kondo_driver/kondo_motor.h"

/**
 * @brief constructor of KondoMotor
 * @param ics File descriptor of ICS device file
 * @param name name of the actuator
 * @param joint_state_interface State interface for the corresponding joint
 * @param position_joint_interface Position interface for the corresponding joint
 * @param velocity_joint_interface Velocity interface for the corresponding joint
 */
KondoMotor::KondoMotor(int ics,
                       std::string name,
		       hardware_interface::JointStateInterface & joint_state_interface,
		       hardware_interface::PositionJointInterface & position_joint_interface,
		       hardware_interface::VelocityJointInterface & velocity_joint_interface, bool loopback = false)
  :  ics(ics), round(0), pos_prev(0), pos_curr(0), cmd_vel(0), pos(0), vel(0), eff(0), skip_count(0), controller_type(0)
{
  this->loopback = loopback;
  this->motor_power = false;
  motor_name = name;
  ros::NodeHandle nh(std::string("~") + motor_name);
  if (nh.getParam("id", id)) {
    ROS_INFO("id: %d", id);
  }
  // Load paramters
  if (nh.getParam("joint_name", joint_name)) {
    ROS_INFO("joint_name: %s", joint_name.c_str());
  }
  if (nh.getParam("min_angle", min_angle)) {
    ROS_INFO("min_angle: %d", min_angle);
  }
  if (nh.getParam("max_angle", max_angle)) {
    ROS_INFO("max_angle: %d", max_angle);
  }
  if (nh.getParam("stretch", stretch)) {
    ROS_INFO("stretch: %d", stretch);
    set_stretch(stretch);
  }
  if (nh.getParam("speed", speed)) {
    ROS_INFO("speed: %d", speed);
    set_speed(speed);
  }
  if (nh.getParam("current_limit", curr_limit)) {
    ROS_INFO("current_limit: %d", curr_limit);
    set_current_limit(curr_limit);
  }
  if (nh.getParam("temperature_limit", temp_limit)) {
    ROS_INFO("temperature_limit: %d", temp_limit);
    set_temperature_limit(temp_limit);
  }
  // Check motor existence
  if (!loopback) {
    if (ics_set_pulse(ics, id, 0, NULL) < 0) {
      ROS_WARN("Cannot connect to servo[%d]. Going to loopback mode.", id);
      loopback = true;
    }
  }
  hardware_interface::JointStateHandle state_handle(joint_name, &pos, &vel, &eff);
  joint_state_interface.registerHandle(state_handle);

  hardware_interface::JointHandle pos_handle(joint_state_interface.getHandle(joint_name), &cmd_pos);
  position_joint_interface.registerHandle(pos_handle);

  hardware_interface::JointHandle vel_handle(joint_state_interface.getHandle(joint_name), &cmd_vel);
  velocity_joint_interface.registerHandle(vel_handle);

  power_service = nh.advertiseService("set_power", &KondoMotor::set_power, this);
  get_param_service = nh.advertiseService("get_parameters", &KondoMotor::get_param_from_eeprom, this);
}

void KondoMotor::update_position(void)
{
  static int pulse_cmd = 0;
  if (cmd_pos < min_angle * 3.14 / 180) {
    cmd_pos = min_angle * 3.14 / 180;
  }
  if (cmd_pos > max_angle * 3.14 / 180) {
    cmd_pos = max_angle * 3.14 / 180;
  }
  if (motor_power == true) {
    pulse_cmd = radian_to_pulse(cmd_pos);
  } else {
    pulse_cmd = 0;
  }
  if (loopback) {
    pos = cmd_pos;
    eff = 0;
  } else {
    int r,
	pulse_ret = 0;
    if (ics_set_pulse(ics, id, pulse_cmd, &pulse_ret) == 0) {
      pos_prev = pos_curr;
      pos = pos_curr = pulse_to_radian(pulse_ret);
    }
    /*
     * how can I get speed ? 
     */
    vel = (pos_curr - pos_prev) / 0.01;
    /*
     * get servo current 
     */
    int current = ics_get_current(ics, id);
    if (current > 0) {
      if (current < 64) {
	eff = current;
      } else {
	eff = -(current - 64);
      }
    }
  }
}

void KondoMotor::update_velocity(void)
{
  static int pulse_cmd = 0;
  if (motor_power == true) {
    pulse_cmd = radian_to_pulse(cmd_vel);
  } else {
    pulse_cmd = 0;
  }
  if (loopback) {
    vel = cmd_vel;
    eff = 0;
    // Simulated motion
    pos += cmd_vel * 0.01;
  } else {
    int r, pulse_ret = 0;
    if (ics_set_pulse(ics, id, pulse_cmd, &pulse_ret) == 0) {
      int n;
      n = skip_count % FLIP_STEP;
      pos_prev = pos_curr;
      pos = pos_curr = pulse_to_radian(pulse_ret);
    }
    /*
     * how can I get speed ? 
     */
    vel = (pos_curr - pos_prev) / 0.01;
    /*
     * get servo current 
     */
    int current = ics_get_current(ics, id);
    if (current > 0) {
      if (current < 64) {
	eff = current;
      } else {
	eff = -(current - 64);
      }
    }
  }
}

void KondoMotor::update(void)
{
  if (controller_type == 0) {
    update_position();
  } else if (controller_type == 1) {
    update_velocity();
  } else {
    ROS_INFO("invalid control type %d", controller_type);
  }
}

bool KondoMotor::set_power(kondo_msgs::setPower::Request & req, kondo_msgs::setPower::Response & res)
{
  motor_power = req.request;
  res.result = req.request;
  if (req.request) {
    ROS_INFO("KondoMotor[%d]: set_power: ON", id);
  }
  else {
    ROS_INFO("KondoMotor[%d]: set_power: OFF", id);
  }
  return true;
}

bool KondoMotor::get_param_from_eeprom(kondo_msgs::getParameters::Request & req, kondo_msgs::getParameters::Response & res)
{
  ROS_INFO("%s: id %d, request: %d", __func__, this->id, req.request);
  get_param_from_eeprom();
  res.result = true;
  return true;
}

/**
 * @brief Get paramter from the EEPROM of the motor
 */
void KondoMotor::get_param_from_eeprom(void)
{
  ros::NodeHandle nh("~" + motor_name + "/eeprom");
  if (ics_get_eeprom(ics, 0, eeprom) < 0) {
    ROS_ERROR("Cannot get param from EEPROM");
    return;
  }
  nh.setParam("stretch", ics_eeprom_stretch (eeprom));
  nh.setParam("sppeed", ics_eeprom_speed (eeprom));
  nh.setParam("punch", ics_eeprom_punch (eeprom));
  nh.setParam("dead_band", ics_eeprom_dead_band (eeprom));
  nh.setParam("dumping", ics_eeprom_dumping (eeprom));
  nh.setParam("safe_timer", ics_eeprom_safe_timer (eeprom));
  nh.setParam("infrev", bool(ics_eeprom_flag (eeprom) & ICS_FLAG_INFREV));
  nh.setParam("max_pulse", ics_eeprom_max_pulse (eeprom));
  nh.setParam("min_pulse", ics_eeprom_min_pulse (eeprom));
  nh.setParam("baud_rate", ics_eeprom_baud_rate (eeprom));
  nh.setParam("max_temperature", ics_eeprom_max_temperature (eeprom));
  nh.setParam("max_current", ics_eeprom_max_current (eeprom));
  nh.setParam("response", ics_eeprom_response (eeprom));
  nh.setParam("user_offset", ics_eeprom_user_offset (eeprom));
}

/**
 * @brief Set paramters to the EEPROM of the motor
 */
void KondoMotor::set_param_to_eeprom(void)
{
}

// Set speed parameter
void KondoMotor::set_speed(unsigned char speed)
{
  if (loopback) {
    this->speed = speed;
  } else {
    this->speed = ics_set_max_speed(ics, id, speed);
    ROS_INFO("%s: %d", __func__, this->speed);
  }
}
// Set strech parameter
void KondoMotor::set_stretch(unsigned char stretch)
{
  if (loopback) {
    this->stretch = stretch;
  } else {
    this->stretch = ics_set_stretch(ics, id, stretch);
    ROS_INFO("%s: %d", __func__, this->stretch);
  }
}
// Set current limit 
void KondoMotor::set_current_limit(unsigned char curr)
{
  if (loopback) {
    curr_limit = curr;
  } else {
    this->curr_limit = ics_set_stretch(ics, id, curr);
    ROS_INFO("%s: %d", __func__, this->curr_limit);
  }
}

// Set temperature limit
void KondoMotor::set_temperature_limit(unsigned char temp)
{
  if (loopback) {
    temp_limit = temp;
  } else {
    this->temp_limit = ics_set_max_temperature(ics, id, temp);
    ROS_INFO("%s: %d", __func__, this->temp_limit);
  }
}
