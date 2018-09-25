/********Librerias*************/
#include "sara_hw_interface.h"

/***Variables globales*********/
CONTROL_DATA control_data;
LOW_LEVEL_TIMING time_debug;
TIMING timing;
PERIOD_CORRECTION period_correction;



MyRobot::MyRobot() //Constructor
 {
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("wheel_left_joint", &control_data.pos[LEFT], &control_data.vel[LEFT], &control_data.eff[LEFT]); //Controller Read from this variables, position, velocity and eff
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("wheel_right_joint", &control_data.pos[RIGHT], &control_data.vel[RIGHT], &control_data.eff[RIGHT]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("wheel_left_joint"), &control_data.cmd[LEFT]); //Desired command variable
   jnt_vel_interface.registerHandle(vel_handle_a);

   hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("wheel_right_joint"), &control_data.cmd[RIGHT]);
   jnt_vel_interface.registerHandle(vel_handle_b);

   registerInterface(&jnt_vel_interface);


  }

void MyRobot::setup(MyRobot* robot)
{

  //Inicio las variables
  memset(&control_data,0x0,sizeof(CONTROL_DATA));
  memset(&period_correction,0x0,sizeof(PERIOD_CORRECTION));
  memset(&time_debug,0x0,sizeof(LOW_LEVEL_TIMING));
  memset(&timing,0x0,sizeof(TIMING));

  cmd_pub = n.advertise<sensor_msgs::JointState>("cmd_wheel", 1000); //Publish data

  //ros::Subscriber vel_sub = n.subscribe("odom_joint_state", 1000,&MyRobot::vel_Callback);


}


void MyRobot::read(void)   // Read data from hardware here. joint_state
  {

    /*Buffer for the RT thread*/
    memcpy(control_data.pos,control_data.temp_pos,2*sizeof(double));
    memcpy(control_data.vel,control_data.temp_vel,2*sizeof(double));

    //Delay of the feedback
    timing.feedback_delay[LEFT]=(ros::Time::now()-timing.feedback_time[LEFT]).toSec();
    timing.feedback_delay[RIGHT]=(ros::Time::now()-timing.feedback_time[RIGHT]).toSec();


#ifdef DEBUG
    //Period of the feedback
    timing.feedback_period[LEFT]=(timing.feedback_time[LEFT]-timing.last_feedback_time[LEFT]).toSec();
    timing.feedback_period[RIGHT]=(timing.feedback_time[RIGHT]-timing.last_feedback_time[RIGHT]).toSec();;

    //Period of the low level time
    time_debug.low_level_period[LEFT]=(time_debug.low_level_time[LEFT]-time_debug.last_low_level_time[LEFT]).toSec();
    time_debug.low_level_period[RIGHT]=(time_debug.low_level_time[RIGHT]-time_debug.last_low_level_time[RIGHT]).toSec();

    //Save values
    memcpy(timing.last_feedback_time,timing.feedback_time,2*sizeof(double));
    memcpy(time_debug.last_low_level_time,time_debug.low_level_time,2*sizeof(double));


    //ROS_INFO("%s (I:%lf D:%lf)", "Leo",pos[LEFT], pos[RIGHT]);
    ROS_INFO("%s Ti %lf Td %lf DelayI %lf DelayD %lf lowI %lf lowD %lf",
            "Loop Time",
            timing.feedback_period[LEFT],timing.feedback_period[RIGHT],
            timing.feedback_delay[LEFT],timing.feedback_delay[RIGHT],
            time_debug.low_level_period[LEFT],time_debug.low_level_period[RIGHT]);
#endif



  }

void MyRobot::write(void)  // Write data to hardware here. joint_command Publication
  {
    sensor_msgs::JointState data; //Declaro data con el formato JointState
    data.name.resize(2);
    data.position.resize(2);
    data.velocity.resize(2);
    data.effort.resize(2);

    data.header.stamp=ros::Time::now();

    data.name[LEFT]="LEFT";
		data.position[LEFT]=0;
		data.velocity[LEFT]=control_data.cmd[LEFT];
		data.effort[LEFT]=0;


		data.name[RIGHT]="RIGHT";
		data.position[RIGHT]=0;
		data.velocity[RIGHT]=control_data.cmd[RIGHT];
		data.effort[RIGHT]=0;

    //ROS_INFO("%s (D:%lf I:%lf)", "Publico",cmd[RIGHT], cmd[LEFT]);

		cmd_pub.publish(data);



  }
int MyRobot::compute_period(void)
{
  int return_rate=0;
  period_correction.acumulated_delay[LEFT]+=timing.feedback_delay[LEFT];
  period_correction.acumulated_delay[RIGHT]+=timing.feedback_delay[RIGHT];

  if (period_correction.loop_counter>LOOP_RATE_CORRECTION_RATE)
  {
    period_correction.average_delay[LEFT]=period_correction.acumulated_delay[LEFT]/LOOP_RATE_CORRECTION_RATE;
    period_correction.average_delay[RIGHT]=period_correction.acumulated_delay[RIGHT]/LOOP_RATE_CORRECTION_RATE;

    if(period_correction.average_delay[LEFT]<0.02 || period_correction.average_delay[RIGHT]<0.02) //Feedback too close to the control loop
          return_rate= (LOOP_RATE-LOOP_RATE_CORRECTION);

    else if(period_correction.average_delay[LEFT]>0.08 || period_correction.average_delay[RIGHT]>0.08) //Feedback too far to the control loop
          return_rate= (LOOP_RATE+LOOP_RATE_CORRECTION);
    else
          return_rate= LOOP_RATE;

    memset(&period_correction,0x0,sizeof(PERIOD_CORRECTION)); //Clear de correction values
  }
  else
  {
    period_correction.loop_counter++;
    return_rate= LOOP_RATE;
  }
  return return_rate;
}

  //The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you

void MyRobot::vel_Callback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    //ROS_INFO("%s", "Callback de velocidades");
    if(msg->name[0] == "RIGHT")
    {
    control_data.temp_pos[RIGHT]=msg->position[0];
    control_data.temp_vel[RIGHT]=msg->velocity[0];
    time_debug.low_level_time[RIGHT]=msg->header.stamp;
    timing.feedback_time[RIGHT]=ros::Time::now();

    }

    else if(msg->name[0] == "LEFT")
    {
    control_data.temp_pos[LEFT]=msg->position[0];
    control_data.temp_vel[LEFT]=msg->velocity[0];
    time_debug.low_level_time[LEFT]=msg->header.stamp;
    timing.feedback_time[LEFT]=ros::Time::now();
    }
  }
