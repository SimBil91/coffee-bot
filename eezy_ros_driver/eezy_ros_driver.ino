// ROS DRIVER FOR EEZY BOT MK2
// WHEN running rosserial package with the correct port, the robot arm will appear as normal rossnode

#include <SPI.h>
#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;
sensor_msgs::JointState robot_state;
char *a[] = {"X", "Y", "Z", "G"}; // Z-Axis, Y-Axis, X-Axis, G-ripper
float pos[4]={0},vel[4]={0},eff[4]={0}; /// stores arduino time
Servo x,y,z,g;
ros::Publisher js_publisher("joint_state", &robot_state);

// Callbacks for setting the servors with their subscribers
void servo_cbx( const std_msgs::UInt16& cmd_msg)
{
  x.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo_cby( const std_msgs::UInt16& cmd_msg)
{
  y.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo_cbz( const std_msgs::UInt16& cmd_msg)
{
  z.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo_cbg( const std_msgs::UInt16& cmd_msg)
{
  g.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
ros::Subscriber<std_msgs::UInt16> subx("x", servo_cbx);
ros::Subscriber<std_msgs::UInt16> suby("y", servo_cby);
ros::Subscriber<std_msgs::UInt16> subz("z", servo_cbz);
ros::Subscriber<std_msgs::UInt16> subg("g", servo_cbg);


void setup() 
{
    // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(js_publisher);
  nh.subscribe(subx);
  nh.subscribe(suby);
  nh.subscribe(subz);
  nh.subscribe(subg);

  x.attach(9);
  y.attach(6);
  z.attach(10);
  g.attach(5);

  robot_state.name_length = 4;
  robot_state.velocity_length = 4;
  robot_state.position_length = 4; /// here used for arduino time
  robot_state.effort_length = 4; /// here used for arduino time
  
  robot_state.name = a;
  robot_state.position = pos;
  robot_state.velocity = vel;
  robot_state.effort = eff;
}

void update_vals()
{
  pos[0]=x.read();
  pos[1]=y.read();
  pos[2]=z.read();
  pos[3]=g.read();
  robot_state.header.stamp=nh.now();
  robot_state.name = a;
  robot_state.position = pos;
  robot_state.velocity = vel;
  robot_state.effort = eff;

}

void loop()
{
  update_vals();
  // Publish robot jointstate
  js_publisher.publish( &robot_state );
  nh.spinOnce();
  delay(100);
}


