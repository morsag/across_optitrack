#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <controller/OptitrackPoseVel.h>
#include <ros/ros.h>
#include "roscopter/Attitude.h"
#define PI 3.141592653
#include <math.h>
#include <tf/transform_datatypes.h>

using namespace std;

class merge_data{
public:
    merge_data();
    float pose_z, pose_x, pose_y, roll, pitch, yaw;
    controller::OptitrackPoseVel pose, pose_old, pose_old2;
    double cam_time_old, opti_time_old;
    void optitrack_callback(const controller::OptitrackPoseVel& msg);
    void camera_callback (const geometry_msgs::Pose &msg);
    void attitude_callback (const roscopter::Attitude &msg);
    void velocity_callback (const geometry_msgs::Twist &msg);
};

merge_data::merge_data()
{
    // pose_z = 0;
    // pose_x = 0;
    // pose_y = 0;
    cam_time_old = ros::Time::now().toSec();
    opti_time_old = ros::Time::now().toSec();

    pose = controller::OptitrackPoseVel();
    pose_old = controller::OptitrackPoseVel();
    pose_old2 = controller::OptitrackPoseVel();

}

void merge_data::optitrack_callback(const controller::OptitrackPoseVel& msg)
{
    /*double time_t = ros::Time::now().toSec();
    double rate = 1.0 / (time_t- opti_time_old);
    opti_time_old = time_t;
    //ROS_INFO("Optitrack callback rate = %.1f", rate);

    pose.pose.pose.position.z = msg.pose.pose.position.z;
    pose.vel.position.z = (pose.pose.pose.position.z - pose_old.pose.pose.position.z)*rate;

    if (abs(pose_old.vel.position.z - pose.vel.position.z) > 0.3) pose.vel.position.z = pose_old.vel.position.z;
    
    // fir
    pose.vel.position.z = pose.vel.position.z*0.4 + pose_old.vel.position.z*0.3 +pose_old2.vel.position.z*0.3;
    pose_old2.pose.pose.position.z = pose_old.pose.pose.position.z;
    pose_old.pose.pose.position.z = pose.pose.pose.position.z;

    roll = msg.pose.pose.orientation.x;
    pitch = msg.pose.pose.orientation.y;
    //yaw = msg.pose.pose.orientation.z;*/
}

void merge_data::camera_callback (const geometry_msgs::Pose &msg)
{
    double time_t = ros::Time::now().toSec();
    double rate = 1.0 / (time_t- cam_time_old);
    tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);

    cam_time_old = time_t;
    //ROS_INFO("Camera callback rate = %.1f", rate);
    pose.pose.pose.position.x = msg.position.x;
    pose.pose.pose.position.y = -msg.position.y;
    pose.pose.pose.position.z = msg.position.z;
    

    //Edits for Kalman 03052014
    //pose.vel.position.x = (pose.pose.pose.position.x - pose_old.pose.pose.position.x)*rate;
    //pose.vel.position.y = (pose.pose.pose.position.y - pose_old.pose.pose.position.y)*rate;

    //if (abs(pose_old.vel.position.x - pose.vel.position.x) > 0.3) pose.vel.position.x = pose_old.vel.position.x;
    //if (abs(pose_old.vel.position.y - pose.vel.position.y) > 0.3) pose.vel.position.y = pose_old.vel.position.y;

    // fir filter
    //pose.vel.position.x = pose.vel.position.x*0.4 + pose_old.vel.position.x*0.3 +pose_old2.vel.position.x*0.3;
    //pose.vel.position.y = pose.vel.position.y*0.4 + pose_old.vel.position.y*0.3 +pose_old2.vel.position.y*0.3;
    //pose.pose.pose.position.x = pose.pose.pose.position.x*0.85+ pose_old.pose.pose.position.x*0.15 +pose_old2.pose.pose.position.x*0.0;
    //pose.pose.pose.position.y = pose.pose.pose.position.y*0.85 + pose_old.pose.pose.position.y*0.15 +pose_old2.pose.pose.position.y*0.0;
    pose_old2.pose.pose.position.x = pose_old.pose.pose.position.x;
    pose_old2.pose.pose.position.y = pose_old.pose.pose.position.y;
    pose_old.pose.pose.position.x = pose.pose.pose.position.x;
    pose_old.pose.pose.position.y = pose.pose.pose.position.y;

}
//Kalman edits
void merge_data::velocity_callback (const geometry_msgs::Twist &msg)
{
    pose.vel.position.x = -msg.linear.x;
    pose.vel.position.y = msg.linear.y;
}

void merge_data::attitude_callback (const roscopter::Attitude &msg)
{
    //roll = -msg.roll;
    //pitch = -msg.pitch;
    //yaw = 0;//msg.yaw + PI/2;
}

int main(int argc, char **argv)
{
    int rate = 20;
    ros::init(argc, argv, "merge_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(rate);
    merge_data data;
    
    //data = merge_data();    


    ros::Subscriber optitrack_subscriber = n.subscribe("Optitrack", 1, &merge_data::optitrack_callback, &data);
    ros::Subscriber velocity_subscriber = n.subscribe("usv_vel", 1, &merge_data::velocity_callback, &data);
    //ros::Subscriber attitude_subscriber = n.subscribe("attitude", 1, &merge_data::attitude_callback, &data);
    ros::Subscriber camera_subscriber = n.subscribe("usv_pose", 1, &merge_data::camera_callback, &data);
    ros::Publisher data_publisher = n.advertise<controller::OptitrackPoseVel>("Optitrack_vision",1);
    //Kalman edits

    while(ros::ok())
    {
        ros::spinOnce();
        data.pose.pose.header.stamp =ros::Time::now();
        // data.pose.pose.pose.orientation.x = data.roll;
        //data.pose.pose.pose.orientation.y = data.pitch;
        data.pose.pose.pose.orientation.z = 0; //data.yaw;

        data_publisher.publish(data.pose);
        loop_rate.sleep();
    }

    return 0;
}
