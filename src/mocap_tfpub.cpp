#include "ros/ros.h"
//#include "mocap/RigidBody.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "MOCAPSocket.h"
#include <vector>
//include <sstream>
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
  char a[15], drones[10][26];
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
    strcpy(drones[i], "default");
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

  tf2_ros::TransformBroadcaster br;
  std::vector<geometry_msgs::TransformStamped> transforms(i);

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
              transforms[k].header.stamp = ros::Time::now();
              transforms[k].header.frame_id = "optitrack";
              transforms[k].child_frame_id = drones[k];
              // Axes are inverted between Optitrack and "environment"
              transforms[k].transform.translation.x = Socket.rigidBody[index].z;
              transforms[k].transform.translation.y = Socket.rigidBody[index].x;
              transforms[k].transform.translation.z = Socket.rigidBody[index].y;
              tf2::Quaternion q(Socket.rigidBody[index].qx, Socket.rigidBody[index].qy,
                                Socket.rigidBody[index].qz, Socket.rigidBody[index].qw);
              transforms[k].transform.rotation.x = q.z();
              transforms[k].transform.rotation.y = q.x();
              transforms[k].transform.rotation.z = q.y();
              transforms[k].transform.rotation.w = q.w();
          }
        }
      }    
      br.sendTransform(transforms);    
    }
    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();
  }

  if (trackables!=NULL) fclose(trackables);

  return 0;
}
