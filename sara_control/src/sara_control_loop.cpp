/********Librerias*************/
#include "sara_hw_interface.h"


void control_loop(ros::Rate rate, MyRobot* robot ,controller_manager::ControllerManager* cm)

{
  const ros::Duration desired_update_freq = ros::Duration((float)1/(float)loop_rate);
  ROS_INFO("%s :%f","upate_frec", desired_update_freq.toSec());
  const double cycle_time_error_threshold = desired_update_freq.toSec()*(double)loop_rate_threshold;
  ROS_INFO("%s :%f","threshold_frec", cycle_time_error_threshold);

  ros::Time last_time=ros::Time::now();
  ros::Duration elapsed_time;

  while(1)
  {

    elapsed_time=ros::Time::now()-last_time;
    last_time=ros::Time::now();

    // Error check cycle time
    const double cycle_time_error = (elapsed_time - desired_update_freq).toSec();
    if (cycle_time_error > cycle_time_error_threshold)
    {
      ROS_WARN_STREAM_NAMED("bucle_control", "Cycle time exceeded error threshold by: "
                                       << cycle_time_error << ", cycle time: " << elapsed_time
                                       << ", threshold: " << cycle_time_error_threshold);
    }
    robot->read();
    cm->update(ros::Time::now(), elapsed_time);
    robot->write();
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


  boost::thread(control_loop, ros::Rate(loop_rate),&robot,&cm);

  ros::spin(); //Permite que se ejecuten Callback



}
