# kondo_motor: Kondo Kagaku's ICS servo motor driver packages

This ROS package contains hardware interface to Kondo Kagaku's ICS
motor system.

 ICS (Interactive Communication System) is a kind of
serial bus to control RC servo motors. Basically it can control
measure its position. It has additional features such as setting gain,
getting current.

kondo_motor is a ROS meta-package, contains the following two
packages:

- kondo_driver: driver node for ICS servo motors
- kondo_msgs: message files for kondo_motor

# Supported Hardware

## ICS servo motors

![KRS-4031HV ICS](http://kondo-robot.com/w/wp-content/uploads/03048_11-400x400.jpg)

- [KRS-4031HV ICS](http://kondo-robot.com/product/krs-4031hv-ics)
- [KRS-4032HV ICS](http://kondo-robot.com/product/krs-4032hv-ics)

The other KRS series servo motors are not checked with this package.

- [Other KRS Series servo
  motors(ICS)](http://kondo-robot.com/product-category/servomotor/krs)

## USB ICS Adapters

Two USB-ICS adapters are available on the Kondo's on-line store. Both
adapters use the FTD232RL chip to interact with servo motors.

![Dual USB adapter HS](http://kondo-robot.com/w/wp-content/uploads/021161-400x400.jpg)

* [ICS USB Adapter HS](http://kondo-robot.com/product/02116)
* [Dual USB Adapter HS](http://www.kopropo.co.jp/sys/archives/4315)

# Quick start

## install device driver

You need to install ftdi_sio. Currently the KONDO's usb ICS adapters
are not registered in the driver source code. You can setup the device
as follow. You need to enter sudo password.

```
$ rosrun kondo_driver setup_device.sh
```

## Setting servo ID.  

Connect each servo and the adapter one to one.
For setting servo ID, you can use ics_set_id.

```
$ rosrun ics_set_id <-d /dev/ttyUSB0> <-i id>
```

After setting the id of each servo motor, you can connect your motors
on the same bus.

## Change config/driver_sample.yaml

The driver setting is described in a yaml file as ROS
parameters. 'config/driver_sample.yaml' shows the example in which two motors
(ID0, ID1) connected.

The servo_0 and servo_1 fields are the servo settings. Each field
means:

* id: Servo's ICS id. The ID on the same bus must not be identical.
* joint_name: Name of the joint attached to the servo.

The following parameters are the interanal characteristics of each
servo motor.

- min_anlge: Minimum angle of servo [deg]
- max_anlge: Maximum angle of servo [deg]
- stretch: stretch parameter
- speed: speed paramter
- current_limit: current limit parameter 
- temperature_limit: temperature limit parameter

You can see more about each parameter from the ICS command reference.

- [ICS 3.5 command reference(in Japanese)](http://kondo-robot.com/w/wp-content/uploads/ICS3.5_SoftwareManual_1_1.pdf)

## Change config/controller_sample.yaml

The kondo_driver is fit for using ros_controllers. Check
config/controller_sample.yaml for ros_control configuration. This
example uses three type of controllers.

- joint_state_controller/JointStateController 
  This controller publish the joint angle and velocity to the /joint_states.

- position_controllers/JointPositionController 
  This controller receive the position command for the servo motor.

- velocity_controllers/JointVelocityController
  This controller receive the velocity command for the servo motor.

See the [ros_control](http://wiki.ros.org/ros_control) for more
details.

## Launch kondo_driver.launch to bring up driver.

The following sample launch file brings up the controllers. 

```
$ roslaunch kondo_driver kondo_driver.launch
```

## Command the servo

Initially all servo motors are free (not powered at all). You can use the service call to to power on the motor.

```
$ rosservce call /joint_0_position_controller/power_on "request: true"
```

Check if you can command the sevo position. Publish
/joint_0_position_controller/command (std_msgs/Float64) to control
joint angle. Unit is in radian.

```
$ rostopic pub -1 /joint_0_position_controller/command std_msgs/Float64 "data: 0.0"
```

# TODO

- to command to set EEPROM parameters
- to swith the controller from/to position/vecocity controller
