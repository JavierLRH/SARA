#include "sensor_msgs/JointState.h"

main()
{
  MyRobot robot; //Init the objet
  controller_manager::ControllerManager cm(&robot);

  ros::Publisher cmd_pub = n.advertise<sensor_msgs::JointState>("cmd_pub", 1000); //Publish data
  ros::Subscriber vel_sub = n.subscribe("vel_sub", 1000, robot.vel_Callback);

  while (true)
  {
     robot.read(); //Make
     cm.update(robot.get_time(), robot.get_period());
     robot.write(); //Make
     sleep();
  }
}
