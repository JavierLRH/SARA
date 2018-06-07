/********Librerias*************/
#include "sara_hw_interface.h"

/***Variables globales*********/
double cmd[2];
double pos[2];
double vel[2];
double eff[2];

double temp_pos[2];
double temp_vel[2];


MyRobot::MyRobot() //Constructor
 {
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("wheel_left_joint", &pos[left], &vel[left], &eff[left]); //Controller Read from this variables, position, velocity and eff
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("wheel_right_joint", &pos[right], &vel[right], &eff[right]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("wheel_left_joint"), &cmd[left]); //Desired command variable
   jnt_vel_interface.registerHandle(vel_handle_a);

   hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("wheel_right_joint"), &cmd[right]);
   jnt_vel_interface.registerHandle(vel_handle_b);

   registerInterface(&jnt_vel_interface);

   cmd_pub = n.advertise<sensor_msgs::JointState>("cmd_pub", 1000); //Publish data

   //Inicio las variables
   pos[left]=0.0;
   temp_pos[left]=0.0;
   pos[right]=0.0;
   temp_pos[right]=0.0;
   vel[left]=0.0;
   temp_vel[left]=0.0;
   vel[right]=0.0;
   temp_vel[right]=0.0;
   eff[left]=0.0;
   eff[right]=0.0;

  }




void MyRobot::read(void)   // Read data from hardware here. joint_state
  {
    pos[left]=temp_pos[left];
    pos[right]=temp_pos[right];
    vel[left]=temp_vel[left];
    vel[right]=temp_vel[right];


    ROS_INFO("%s (I:%lf D:%lf)", "Leo",pos[left], pos[right]);




  }

void MyRobot::write(void)  // Write data to hardware here. joint_command Publication
  {
    sensor_msgs::JointState data; //Declaro data con el formato JointState
    data.name.resize(2);
    data.position.resize(2);
    data.velocity.resize(2);
    data.effort.resize(2);

		data.name[left]="LEFT";
		data.position[left]=0;
		data.velocity[left]=cmd[left];
		data.effort[left]=0;


		data.name[right]="RIGHT";
		data.position[right]=0;
		data.velocity[right]=cmd[right];
		data.effort[right]=0;

    ROS_INFO("%s (D:%lf I:%lf)", "Publico",cmd[right], cmd[left]);

		cmd_pub.publish(data);



  }

void MyRobot::vel_Callback(const sensor_msgs::JointState::ConstPtr& msg)//The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you
  {
    //Lecturas temporales
    temp_pos[right]=msg->position[right];
    temp_pos[left]=msg->position[left];
    temp_vel[right]=msg->velocity[right];
    temp_vel[left]=msg->velocity[left];
    //ROS_INFO("%s", "Callback de velocidades");


  }
