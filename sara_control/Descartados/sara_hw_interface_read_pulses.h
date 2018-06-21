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
#include "sara_control/enc_msg.h"

/**************Definiciones***************/
#define left 0
#define right 1
#define loop_rate 10 /* 10*F_feedback*/
#define loop_rate_threshold 0.1 /*10%*/
#define steps_per_revolution 64000
#define pi 3.14159265358979323846


/************Definicion de clases********/

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(); //Constructor

  void read(void);   // Read data from hardware here. joint_state
  void write(void);  // Write data to hardware here. joint_command Publication

  void vel_Callback(const sara_control::enc_msg::ConstPtr& msg);//The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you

  unsigned int get_flag_feedback(void);
  void reset_flag_feedback(void);


private:
  hardware_interface::JointStateInterface jnt_state_interface; //Always
  hardware_interface::VelocityJointInterface jnt_vel_interface;



  ros::Publisher cmd_pub;
  ros::NodeHandle n;


};
