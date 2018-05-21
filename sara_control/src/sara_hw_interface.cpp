#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
 {
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]); //Controller Read from this variables, position, velocity and eff
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]); //Desired command variable
   jnt_vel_interface.registerHandle(vel_handle_a);

   hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
   jnt_vel_interface.registerHandle(vel_handle_b);

   registerInterface(&jnt_vel_interface);

   void read()   // Read data from hardware here. joint_state
  {
    pos[0]=temp_pos[0];
    pos[1]=temp_pos[1];
    vel[0]=temp_vel[0];
    vel[1]=temp_vel[1];


  }
  void write()  // Write data to hardware here. joint_command Publication
  {
    sensor_msgs::JointState data;
    data.name.resize(2);
    data.position.resize(2);
    data.velocity.resize(2);
    data.effort.resize(2);

		data.name[0]="D"
		data.position[0]=0;
		data.velocity[0]=cmd[0];
		data.effort[0]=0;


		dataI.name[1]="I"
		dataI.position[1]=0;
		dataI.velocit[1]=cmd[1];
		dataI.effort[1]=0;

    

		cmd_pub.publish(data);


  }

  void velCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    //Lecturas temporales
    temp_pos[0]=msg.position[0];
    temp_pos[1]=msg.position[1];
    temp_vel[0]=msg.velocity[0];
    temp_vel[1]=msg.velocity[1];


  }
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface; //Always
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

  double temp_pos[0]=0;
  double temp_pos[1]=0;
  double temp_pos[2]=0;
  double temp_pos[3]=0;

};
