/********Librerias*************/
#include "sara_hw_interface.h"


void control_loop(ros::Rate rate, MyRobot* robot ,controller_manager::ControllerManager* cm)

{
  ros::Time last_time=ros::Time::now();
  ros::Duration elapsed_time;
  while(1)
  {

    elapsed_time=ros::Time::now()-last_time;
    last_time=ros::Time::now();
    robot->read(); //Make
    cm->update(ros::Time::now(), elapsed_time);
    robot->write(); //Make
    rate.sleep();
  }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw_interface");
  ros::NodeHandle n;
  ROS_INFO("%s", "Nodo control_iterface iniciado");

  MyRobot robot; //Init the objet
  controller_manager::ControllerManager cm(&robot); //Error
  ros::Subscriber vel_sub = n.subscribe("odom_joint_state", 1000,&MyRobot::vel_Callback ,&robot);
  //ros::Publisher cmd_pub = n.advertise<sensor_msgs::JointState>("cmd_pub", 1000); //Publish data

  boost::thread(control_loop, ros::Rate(loop_rate),&robot,&cm);

  ros::spin(); //Permite que se ejecuten Callback



}
