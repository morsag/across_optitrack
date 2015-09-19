#include "ros/ros.h"
//#include "mocap/RigidBody.h"
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
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
#include <sys/time.h>
#include <iostream>
#include <arpa/inet.h>
#include <time.h> 
//#include "SDLinput.h"
#include <iostream>
#include <stdlib.h>
#include <string.h>

using namespace std;

float LowPassFilter(float u, float y, float r)
{
    return (y*(1-r)+u*r);
}

//////////////////////////////////////////////////////////////////////////////////
/*
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  char * trackable = NULL;
  FILE * trackables;
  char a[15], drones[10][26],drones_vel[10][30];
  char default_name[50] = "/Optitrack";
  int DronesID[10];
  char default_name_vel[50] = "/Optitrack_vel";
  int i = 0, k = 0, j = 0, index = -1;
  int rate = 20;
  float r = 1;
  int counter = 0;
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

  argc--; argv++;
  while( argc && *argv[0] == '-' )
  {
    if( !strcmp(*argv, "-trackables") && ( argc > 1 ) )
    {
      trackable = *(argv+1);
      printf("Using custom drone names from file - %s\n",trackable);
      argc--; argv++;
    }
    argc--; argv++;
  }

  trackables=fopen(trackable, "r");
  if ((trackables==NULL) and (trackable!=NULL)) printf("Can't open file %s\n", trackable);
  if (trackable==NULL) printf("Using default topic name.\n");
  printf("----------------------------------------------\n");

  ros::init(argc, argv, "NatNet_Talker");

  /*
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;

  if (trackables!=NULL)
  {
    while ((fscanf(trackables, "%s",a)==1) and i<=10)
    {
      if (k % 2 == 0)
      {
        strcpy(drones[i], a);
        strcat(drones[i], default_name);
        printf("Creating topic %s\n",drones[i]);
        strcpy(drones_vel[i], a);
        strcat(drones_vel[i], default_name_vel);
        printf("Creating topic %s\n",drones_vel[i]);
      }
      else
      {
        DronesID[i] = atoi(a);
        ++i;
      }
      k++;
    }
  }
  else 
  {
    strcpy(drones[i], default_name);
    strcpy(drones_vel[i], default_name_vel);
    i = 1;
  }

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

  ros::Publisher *chatter_pub = new ros::Publisher[i];
  ros::Publisher *chatter_pub_vel = new ros::Publisher[i];
  geometry_msgs::PoseStamped *msg = new geometry_msgs::PoseStamped[i];
  geometry_msgs::TwistStamped *msg_old = new geometry_msgs::TwistStamped[i];
  geometry_msgs::TwistStamped *msg_old2 = new geometry_msgs::TwistStamped[i];
  geometry_msgs::TwistStamped *msg_vel = new geometry_msgs::TwistStamped[i];

  for (k=0;k<i;k++)
  {
    chatter_pub[k] = n.advertise<geometry_msgs::PoseStamped>(drones[k], 1);
    chatter_pub_vel[k] = n.advertise<geometry_msgs::TwistStamped>(drones_vel[k], 1);
  }

///////////////////////////////////////////////////////////////////////////
  MOCAPSocket Socket;
///////////////////////////////////////////////////////////////////////////


  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    //mocap::pc2quadcopter msg;
    if(Socket.Read()>0)
    {
      /**
      * This is a message object. You stuff it with data, and then publish it.
      */
      //mocap::pc2quadcopter msg;
      //msg.rotation.x=Socket.rigidBody[0].qx;
      //msg.rotation.y=Socket.rigidBody[0].qy;
      //msg.rotation.z=Socket.rigidBody[0].qz;
      //msg.rotation.w=Socket.rigidBody[0].qw;
      //msg.translation.x
      //joy.Refresh();

      for (k=0;k<i;k++)
      {

        index = -1;
        for (j=0;j<Socket.NmbrOfRigidBody;j++)
        {
          if (Socket.rigidBodyID[j] == DronesID[k])
          {
            index = j;
            break;
          }
        }

        if (index!=-1)
        {
          if (Socket.rigidBody[index].pitch!=0 && Socket.rigidBody[index].roll!=0 && Socket.rigidBody[index].yaw!=0 && Socket.rigidBody[index].z!=0)
          {
            msg_old2[k] = msg_old[k];
            msg_old[k] = msg_vel[k];
            msg_vel[k].twist.linear.x = (Socket.rigidBody[index].z - msg[k].pose.position.x)*rate;
            msg_vel[k].twist.linear.y = (Socket.rigidBody[index].x - msg[k].pose.position.y)*rate;
            msg_vel[k].twist.linear.z = (Socket.rigidBody[index].y - msg[k].pose.position.z)*rate;
            msg_vel[k].twist.angular.z = (Socket.rigidBody[index].yaw*3.14/180 - msg[k].pose.orientation.z)*rate;

            //msg[k].vel.position.x = LowPassFilter((Socket.rigidBody[k].z - msg[k].pose.pose.position.x)*rate, msg[k].vel.position.x, r);
            //msg[k].vel.position.y = LowPassFilter((Socket.rigidBody[k].x - msg[k].pose.pose.position.y)*rate, msg[k].vel.position.y, r);
            //msg[k].vel.position.z = LowPassFilter((Socket.rigidBody[k].y - msg[k].pose.pose.position.z)*rate, msg[k].vel.position.z, r);
            //msg[k].vel.orientation.z = LowPassFilter((Socket.rigidBody[k].yaw*3.14/180 - msg[k].pose.pose.orientation.z)*rate, msg[k].vel.orientation.z, r);

            msg_vel[k].twist.linear.x = msg_vel[k].twist.linear.x *0.4 + msg_old[k].twist.linear.x*0.3+ msg_old2[k].twist.linear.x*0.3;
            msg_vel[k].twist.linear.y = msg_vel[k].twist.linear.y*0.4 + msg_old[k].twist.linear.y*0.3+ msg_old2[k].twist.linear.y*0.3;
            msg_vel[k].twist.linear.z = msg_vel[k].twist.linear.z*0.4 + msg_old[k].twist.linear.z*0.3+ msg_old2[k].twist.linear.z*0.3;
            msg_vel[k].twist.angular.z = msg_vel[k].twist.angular.z*0.4 + msg_old[k].twist.angular.z*0.3 +msg_old2[k].twist.angular.z*0.3;

            if (abs(msg_old[k].twist.linear.x - msg_vel[k].twist.linear.x) > 0.3) msg_vel[k].twist.linear.x = 0;
            if (abs(msg_old[k].twist.linear.y - msg_vel[k].twist.linear.y) > 0.3) msg_vel[k].twist.linear.y = 0;
            if (abs(msg_old[k].twist.linear.z - msg_vel[k].twist.linear.z) > 0.3) msg_vel[k].twist.linear.z = 0;
            if (abs(msg_old[k].twist.angular.z - msg_vel[k].twist.angular.z) > 0.3) msg_vel[k].twist.angular.z = 0;

            msg[k].pose.orientation.x=Socket.rigidBody[index].pitch*3.14/180;
            //msg.translation.y=Socket.rigidBody[0].y;
            msg[k].pose.orientation.y=Socket.rigidBody[index].roll*3.14/180;
            //msg.translation.z=Socket.rigidBody[0].z;
            msg[k].pose.orientation.z=Socket.rigidBody[index].yaw*3.14/180;
            //msg.orientation.w=(Socket.rigidBody[0].x-Socket.rigidBody[0].x_old)/20;
            //msg.position.x= (joy.axis1_-500)*3.14/4500;
            //msg.position.y= -(joy.axis2_-500)*3.14/4500;
            msg[k].pose.position.x= Socket.rigidBody[index].z;
            msg[k].pose.position.y= Socket.rigidBody[index].x;
            msg[k].pose.position.z= Socket.rigidBody[index].y;
            msg[k].header.stamp =ros::Time::now();
            //msg.speed.x=-(Socket.rigidBody[0].z-Socket.rigidBody[0].z_old)/20;
            //msg.speed.y=(Socket.rigidBody[0].x-Socket.rigidBody[0].x_old)/20;
            //msg.speed.z=(Socket.rigidBody[0].y-Socket.rigidBody[0].y_old)/20;
          }
          chatter_pub[k].publish(msg[k]);
          chatter_pub_vel[k].publish(msg_vel[k]);
        }
      }        
    }
    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();
  }

  delete [] chatter_pub;
  delete [] msg;

  if (trackables!=NULL) fclose(trackables);

  return 0;
}
