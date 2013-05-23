#include "ros/ros.h"
//#include "mocap/RigidBody.h"
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
//#include "mocap/pc2quadcopter.h"
#include "MOCAPSocket.h"
#include <sstream>
/////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <stdio.h>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <arpa/inet.h>
#include "SDLinput.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////
/*
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	//
  /*
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "NatNet_Talker");
  /*
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  //ros::Publisher chatter_pub = n.advertise<geometry_msgs::Transform>("servo", 1000);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("/Optitrack/RB0", 1000);
  ros::Publisher chatter_pub1 = n.advertise<geometry_msgs::Pose>("/Optitrack1/RB1", 1000);
  ros::Rate loop_rate(100);


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
///////////////////////////////////////////////////////////////////////////
  MOCAPSocket Socket;
///////////////////////////////////////////////////////////////////////////
  int count = 0;
  while (ros::ok())
  {
	  //mocap::pc2quadcopter msg;
   if(Socket.Read()>0)
   {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
		geometry_msgs::Pose msg;
    geometry_msgs::Pose msg1;
		//mocap::pc2quadcopter msg;
    	//msg.rotation.x=Socket.rigidBody[0].qx;
    	//msg.rotation.y=Socket.rigidBody[0].qy;
    	//msg.rotation.z=Socket.rigidBody[0].qz;
    	//msg.rotation.w=Socket.rigidBody[0].qw;
    	//msg.translation.x
		//joy.Refresh();
    if (Socket.rigidBody[0].pitch!=0 && Socket.rigidBody[0].roll!=0 && Socket.rigidBody[0].yaw!=0)
    {
      msg.orientation.x=Socket.rigidBody[0].pitch*3.14/180;
        //msg.translation.y=Socket.rigidBody[0].y;
      msg.orientation.y=Socket.rigidBody[0].roll*3.14/180;
      //msg.translation.z=Socket.rigidBody[0].z;
      msg.orientation.z=Socket.rigidBody[0].yaw*3.14/180;
      //msg.orientation.w=(Socket.rigidBody[0].x-Socket.rigidBody[0].x_old)/20;
        //msg.position.x= (joy.axis1_-500)*3.14/4500;
      //msg.position.y= -(joy.axis2_-500)*3.14/4500;
      msg.position.x= Socket.rigidBody[0].z;
      msg.position.y= Socket.rigidBody[0].x;
      msg.position.z= Socket.rigidBody[0].y;
      //msg.speed.x=-(Socket.rigidBody[0].z-Socket.rigidBody[0].z_old)/20;
      //msg.speed.y=(Socket.rigidBody[0].x-Socket.rigidBody[0].x_old)/20;
      //msg.speed.z=(Socket.rigidBody[0].y-Socket.rigidBody[0].y_old)/20;
      chatter_pub.publish(msg);

    }
    if (Socket.rigidBody[1].pitch!=0.0 && Socket.rigidBody[1].roll!=0.0 && Socket.rigidBody[1].yaw!=0.0)
    {
      msg1.orientation.x=Socket.rigidBody[1].pitch*3.14/180;
        //msg.translation.y=Socket.rigidBody[0].y;
      msg1.orientation.y=Socket.rigidBody[1].roll*3.14/180;
      //msg.translation.z=Socket.rigidBody[0].z;
      msg1.orientation.z=Socket.rigidBody[1].yaw*3.14/180;
      //msg.orientation.w=(Socket.rigidBody[0].x-Socket.rigidBody[0].x_old)/20;
        //msg.position.x= (joy.axis1_-500)*3.14/4500;
      //msg.position.y= -(joy.axis2_-500)*3.14/4500;
      msg1.position.x= Socket.rigidBody[1].z;
      msg1.position.y= Socket.rigidBody[1].x;
      msg1.position.z= Socket.rigidBody[1].y;
      chatter_pub1.publish(msg1);

    }

    
    }
    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

