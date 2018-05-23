#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include "std_msgs/String.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

#include "sensor_msgs/JointState.h"



class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() //Constructor
 {
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("wheel_left_joint", &pos[0], &vel[0], &eff[0]); //Controller Read from this variables, position, velocity and eff
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("wheel_right_joint", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("wheel_left_joint"), &cmd[0]); //Desired command variable
   jnt_vel_interface.registerHandle(vel_handle_a);

   hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("wheel_right_joint"), &cmd[1]);
   jnt_vel_interface.registerHandle(vel_handle_b);

   registerInterface(&jnt_vel_interface);

   cmd_pub = n.advertise<sensor_msgs::JointState>("cmd_pub", 1000); //Publish data

  }




   void read(void)   // Read data from hardware here. joint_state
  {
    pos[0]=temp_pos[0];
    pos[1]=temp_pos[1];
    vel[0]=temp_vel[0];
    vel[1]=temp_vel[1];




  }
  void write(void)  // Write data to hardware here. joint_command Publication
  {
    sensor_msgs::JointState data; //Declaro data con el formato JointState
    data.name.resize(2);
    data.position.resize(2);
    data.velocity.resize(2);
    data.effort.resize(2);

		data.name[0]="D";
		data.position[0]=0;
		data.velocity[0]=cmd[0];
		data.effort[0]=0;


		data.name[1]="I";
		data.position[1]=0;
		data.velocity[1]=cmd[1];
		data.effort[1]=0;



		cmd_pub.publish(data);


  }

  void vel_Callback(const sensor_msgs::JointState::ConstPtr& msg)//The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you
  {
    //Lecturas temporales
    temp_pos[0]=msg->position[0];
    temp_pos[1]=msg->position[1];
    temp_vel[0]=msg->velocity[0];
    temp_vel[1]=msg->velocity[1];
    ROS_INFO("%s", "Callback de velocidades");


  }


private:
  hardware_interface::JointStateInterface jnt_state_interface; //Always
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

  double temp_pos[2];
  double temp_vel[2];

  ros::Publisher cmd_pub;
  ros::NodeHandle n;






};





int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw_interface");
  ros::NodeHandle n;
  ROS_INFO("%s", "Nodo iniciado");

  MyRobot robot; //Init the objet
  controller_manager::ControllerManager cm(&robot); //Error
  ros::Subscriber vel_sub = n.subscribe("odom_joint_state", 1000,&MyRobot::vel_Callback ,&robot);
  //ros::Publisher cmd_pub = n.advertise<sensor_msgs::JointState>("cmd_pub", 1000); //Publish data

  ros::Time last_time=ros::Time::now();
  ros::Duration elapsed_time;
  ros::Rate loop_rate(10);
  ROS_INFO("%s", "Entrada al bucle");

  while (ros::ok())
  {
    elapsed_time=ros::Time::now()-last_time;
     last_time=ros::Time::now();
     robot.read(); //Make
     cm.update(ros::Time::now(), elapsed_time);
     robot.write(); //Make

     ros::spinOnce();
     loop_rate.sleep();
     //sleep();
  }
}
