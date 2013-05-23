#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "MOCAPSocket.h"


using namespace std;

//////////////////////////////////////////////////////////////////////////////////
/*
 * This tutorial demonstrates simple sending of NatNet messages over the ROS system.
 */
int main(int argc, char **argv)
{
//Standard ROS initialization functions
  ros::init(argc, argv, "NatNet_Talker");

  ros::NodeHandle n;
/////////////////////////////////////////////////////////////////////////////////
  /**
   * Here we define topics necessary to transmit the information about the rigid
   * bodies through ROS. The number of topics depends on the number of rigid bodies
   * you would like to track. MAX_RIGID_BODY, defined in MOCAPSocekt.h allows up to
   * 32 rigid bodies. In reality, you will track much less rigid bodies.
   */

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("/Optitrack/RB0", 1000);
  ros::Publisher chatter_pub1 = n.advertise<geometry_msgs::Pose>("/Optitrack1/RB1", 1000);
  ros::Rate loop_rate(100);
/////////////////////////////////////////////////////////////////////////////////
 /**
  * This is the object defined in "MOCAPSocket.h". This object opens the socket
  * and connects to the Optitrack PC. Every time you read the socket MOCAPSocket
  * object it will check the available data and refresh its rigid bodies.
  */

  MOCAPSocket Socket;
/////////////////////////////////////////////////////////////////////////////////

  while (ros::ok())
  {
	  if(Socket.Read()>0)
	  {
			/**
			 * This is a message object. Define as many message objects as you would like
			 * to track. One for each topic related to that particular rigid body.
			 */
				geometry_msgs::Pose msg;
				geometry_msgs::Pose msg1;

			if (Socket.rigidBody[0].pitch!=0 && Socket.rigidBody[0].roll!=0 && Socket.rigidBody[0].yaw!=0)
			{
			/**
			 * The if statement checks if the rigid body is tracked with the MOCAP computer. If not,
			 * the MOCAP computer sends all zeros for that particular rigid body.
			 */
			  msg.orientation.x=Socket.rigidBody[0].pitch*3.14/180;
			  msg.orientation.y=Socket.rigidBody[0].roll*3.14/180;
			  msg.orientation.z=Socket.rigidBody[0].yaw*3.14/180;
			  msg.position.x= Socket.rigidBody[0].z;
			  msg.position.y= Socket.rigidBody[0].x;
			  msg.position.z= Socket.rigidBody[0].y;
			  chatter_pub.publish(msg);

			}
			if (Socket.rigidBody[1].pitch!=0.0 && Socket.rigidBody[1].roll!=0.0 && Socket.rigidBody[1].yaw!=0.0)
			{
			  msg1.orientation.x=Socket.rigidBody[1].pitch*3.14/180;
			  msg1.orientation.y=Socket.rigidBody[1].roll*3.14/180;
			  msg1.orientation.z=Socket.rigidBody[1].yaw*3.14/180;
			  msg1.position.x= Socket.rigidBody[1].z;
			  msg1.position.y= Socket.rigidBody[1].x;
			  msg1.position.z= Socket.rigidBody[1].y;
			  chatter_pub1.publish(msg1);

			}
	  }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

