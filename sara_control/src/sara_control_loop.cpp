/*********************************
************ Includes ************
**********************************/
#include "sara_hw_interface.h"

//*********************************
// Name:        main
// Returns:     none
// Parameters:  Objets of the hardware interface and controller manager
// Description: thread of the controller
//*********************************

void control_loop(ros::Rate rate, MyRobot* robot ,controller_manager::ControllerManager* cm)

{
  const ros::Duration desired_update_freq = ros::Duration((float)1/(float)LOOP_RATE);
  ROS_INFO("%s :%f","upate_frec", desired_update_freq.toSec());
  const double cycle_time_error_threshold = desired_update_freq.toSec()*(double)LOOP_RATE_THRESHOLD;
  ROS_INFO("%s :%f","threshold_frec", cycle_time_error_threshold);


  ros::Time last_time=ros::Time::now();
  ros::Duration elapsed_time;

  while(1)
  {
    //Save time data
    elapsed_time=ros::Time::now()-last_time;
    last_time=ros::Time::now();

    //Check cycle time error
    const double cycle_time_error = (elapsed_time - desired_update_freq).toSec();
    if (cycle_time_error > cycle_time_error_threshold)
    {
      ROS_WARN_STREAM_NAMED("bucle_control", "Cycle time exceeded error threshold by: "
                                       << cycle_time_error << ", cycle time: " << elapsed_time
                                       << ", threshold: " << cycle_time_error_threshold);
    }

    //Compute control
    robot->read();
    cm->update(ros::Time::now(), elapsed_time);
    robot->write();

    //Sleep until next cicle
#ifdef ADJUST_RATE
    ros::Rate fixed_rate(robot->compute_period());
    fixed_rate.sleep();
#else
    rate.sleep();
#endif
  } //End of while

}//End of fn

//*********************************
// Name:        main
// Returns:     none
// Parameters:  system arguments
// Description: entry point
//*********************************

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw_interface");
  ros::NodeHandle n;
  ROS_INFO("%s", "Nodo control_iterface iniciado");

  MyRobot robot; //Hardware interface objet
  controller_manager::ControllerManager cm(&robot); //Controller manager objet

  robot.setup(&robot); //Init variables

  ros::Subscriber vel_sub = n.subscribe("wheel_state", 100,&MyRobot::vel_Callback ,&robot);

  boost::thread(control_loop, ros::Rate(LOOP_RATE),&robot,&cm);//thread of the controller

  ros::spin(); //Infinite loop. Allows to run Callbacks

}
