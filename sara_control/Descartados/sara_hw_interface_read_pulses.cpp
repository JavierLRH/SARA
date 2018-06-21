/********Librerias*************/
#include "sara_hw_interface.h"

/***Variables globales*********/
double cmd[2];
double pos[2];
double vel[2];
double eff[2];

double dt_left;
int dsteps_left;
int time_enc_left_last;
int steps_enc_left_last;

double dt_right;
int dsteps_right;
int time_enc_right_last;
int steps_enc_right_last;



unsigned int flag_feedback;


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
   pos[right]=0.0;
   vel[left]=0.0;
   vel[right]=0.0;
   eff[left]=0.0;
   eff[right]=0.0;

   dt_left=0.0:
   dsteps_left=0:
   time_enc_left_last=0;
   steps_enc_left_last=0;

   dt_right=0.0;
   dsteps_right=0;
   time_enc_right_last=0;
   steps_enc_right_last=0;

   flag_feedback=0;

  }




void MyRobot::read(void)   // Read data from hardware here. joint_state
  {
    /*Buffer for the RT thread*/

    pos[left]=((double)steps_enc_left_last/(double)steps_per_revolution)*2*pi;
    pos[right]=((double)steps_enc_right_last/(double)steps_per_revolution)*2*pi;
    vel[left]=(((double)dsteps_left/(double)steps_per_revolution)*2*pi)/(double)dt_left;
    vel[right]=(((double)dsteps_right/(double)steps_per_revolution)*2*pi)/(double)dt_right;


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

void MyRobot::odom_Callback(const sara_control::enc_msg::ConstPtr& msg)//The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you
  {
    //ROS_INFO("%s", "Callback de velocidades");
    //temp_pos[right]=msg->position[right];
    int time_enc_right=0;
    int steps_enc_right=0;

    int time_enc_left=0;
    int steps_enc_left=0;

    if(msg->encID==0): //EncoderA (RIGHT)

      time_enc_right=msg->time
      steps_enc_right=msg->data

      //Calcular incrementos
      dt_right=(double)(time_enc_right-time_enc_right_last)*(10**-4) //100us
      dsteps_right=steps_enc_right-steps_enc_right_last

      //Guardiar variables actuales
      time_enc_right_last=time_enc_right
      steps_enc_right_last=steps_enc_right


    if(msg->encID==1): //EncoderB (LEFT)

      time_enc_left=msg->time
      steps_enc_left=msg->data

      //Calcular incrementos
      dt_left=(double)(time_enc_left-time_enc_left_last)*(10**-4) //100us
      dsteps_left=steps_enc_left-steps_enc_left_last

      //Guardiar variables actuales
      time_enc_left_last=time_enc_left
      steps_enc_left_last=steps_enc_left




    flag_feedback=1;



  }

  unsigned int MyRobot::get_flag_feedback(void)
  {
    return flag_feedback;

  }

  void MyRobot::reset_flag_feedback(void)
  {
    flag_feedback=0;
  }
