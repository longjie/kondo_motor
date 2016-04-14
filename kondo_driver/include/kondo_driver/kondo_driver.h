/**
 * Kondo ICS motor driver
 */
#include "kondo_driver/kondo_motor.h"
#include "kondo_msgs/setPower.h"

extern "C" {
#include "kondo_driver/ics_serial.h"
}

/**
 * @brief KondoDriver class derived from the hardware_interface class
 */
class KondoDriver : public hardware_interface::RobotHW
{
private:
  bool loopback; /// loopback mode (no use of hardware)
  int ics; /// file descriptor of ICS device file

  // Hardware interface
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface position_joint_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  // Vector of motors
  std::vector < boost::shared_ptr < KondoMotor > >actuator_vector;

public:
  KondoDriver(int num, char **actuators);
  ~KondoDriver ();
  void update (void);
  bool canSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, 
    const std::list<hardware_interface::ControllerInfo>& stop_list);
#if 0
  bool prepareSwitch(const std::list< hardware_interface::ControllerInfo > & start_list,
                     const std::list< hardware_interface::ControllerInfo > & stop_list);
#endif
  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                        const std::list<hardware_interface::ControllerInfo> &stop_list);
};
