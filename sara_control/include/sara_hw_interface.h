#ifndef SARA_HW_INTERFACE
#define SARA_HW_INTERFACE
/********Librerias*************/
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>

#include "std_msgs/String.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

#include "sensor_msgs/JointState.h"

/**************Definiciones***************/
#define LEFT 0
#define RIGHT 1
#define LOOP_RATE 10 /* 10*F_feedback*/
#define LOOP_RATE_THRESHOLD 0.1
#define LOOP_RATE_CORRECTION 1 //Hz
#define LOOP_RATE_CORRECTION_RATE 20 //Correct the delay each 10 control loop

//#define DEBUG
#define ADJUST_RATE

/************Estructuras********/
struct LOW_LEVEL_TIMING {
  ros::Time low_level_time[2];
  ros::Time last_low_level_time[2];
  double low_level_period[2];
} ;

struct TIMING {
  ros::Time feedback_time[2];
  ros::Time last_feedback_time[2];
  double feedback_delay[2];
  double feedback_period[2];
};

struct PERIOD_CORRECTION{
  double acumulated_delay[2];
  double average_delay[2];
  unsigned int loop_counter;
};

struct CONTROL_DATA{
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

  double temp_pos[2];
  double temp_vel[2];
};
/************Definicion de clases********/

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(); //Constructor

  void read(void);   // Read data from hardware here. joint_state
  void write(void);  // Write data to hardware here. joint_command Publication
  int compute_period(void);
  void vel_Callback(const sensor_msgs::JointState::ConstPtr& msg);//The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you
  void setup(MyRobot*);


private:
  hardware_interface::JointStateInterface jnt_state_interface; //Always
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  ros::Publisher cmd_pub;
  ros::NodeHandle n;


};

#endif
