#include "kalman_filter_data.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <math.h> 

kalman_filter_data::kalman_filter_data()
{
	xacc = 0;
	yacc = 0;
	zacc = 0;
	pose_x = 0;
	pose_y = 0;
	pose_z = 0;
	roll = 0;
	pitch = 0;
	yaw = 0;
	imuRoll = 0;
	imuPitch = 0;
	imuYaw = 0;
	rollspeed = 0;
	yawspeed = 0;
	pitchspeed = 0;
}

void kalman_filter_data::imu_callback(const roscopter::Mavlink_RAW_IMU& msg)
{
	xacc = msg.xacc;
	yacc = msg.yacc;
	zacc = msg.zacc;
}

void kalman_filter_data::optitrack_callback(const controller::OptitrackPoseVel& msg)
{
	time_optitrack = msg.pose.header.stamp;
	pose_x = msg.pose.pose.position.x;
	pose_y = msg.pose.pose.position.y;
	pose_z = msg.pose.pose.position.z;
	roll = msg.pose.pose.orientation.x;
	pitch = msg.pose.pose.orientation.y;
	yaw = msg.pose.pose.orientation.z;

}

void kalman_filter_data::attitude_callback(const roscopter::Attitude& msg)
{
	time_imu = ros::Time::now();
	imuRoll = msg.roll;
	imuPitch = msg.pitch;
	imuYaw = msg.yaw;
	rollspeed = msg.rollspeed;
	yawspeed = msg.yawspeed;
	pitchspeed = msg.pitchspeed;
}

int main(int argc, char **argv)
{
	int rate = 60;
	int i=0;
	int j=0;
	int same_data = 0;
	float rollOver2, sinRollOver2, cosRollOver2, pitchOver2, sinPitchOver2, cosPitchOver2, yawOver2, sinYawOver2, cosYawOver2;
	ros::init(argc, argv, "kalman_filter_data");
	ros::NodeHandle n;
	ros::Rate loop_rate(rate);

	nav_msgs::Odometry vo_data;
	sensor_msgs::Imu raw_imu_data;


	kalman_filter_data data;

	ros::Subscriber optitrack_subscriber = n.subscribe("Optitrack", 1, &kalman_filter_data::optitrack_callback, &data);
	ros::Subscriber imu_subscriber = n.subscribe("raw_imu",1, &kalman_filter_data::imu_callback, &data);
	ros::Subscriber attitude_subscriber = n.subscribe("attitude",1, &kalman_filter_data::attitude_callback, &data);

	ros::Publisher vo_pub = n.advertise<nav_msgs::Odometry>("/vo",1);
	ros::Publisher imu_data_pub = n.advertise<sensor_msgs::Imu>("/imu_data",1);

	while(ros::ok())
	{
		if (data.pose_x == vo_data.pose.pose.position.x && data.pose_y == vo_data.pose.pose.position.y && data.pose_z == vo_data.pose.pose.position.z) same_data += 1;
		else same_data = 0;

		vo_data.header.stamp = data.time_optitrack;
		vo_data.header.frame_id = "base_footprint";
		vo_data.pose.pose.position.x = data.pose_x;
		vo_data.pose.pose.position.y = data.pose_y;
		vo_data.pose.pose.position.z = data.pose_z;
		vo_data.pose.pose.orientation.x = data.roll;//cos(data.roll/2.)*cos(data.pitch/2.)*cos(data.yaw/2.) + sin(data.roll/2.)*sin(data.pitch/2.)*sin(data.yaw/2.);
		vo_data.pose.pose.orientation.y = data.pitch;//sin(data.roll/2.)*cos(data.pitch/2.)*cos(data.yaw/2.) - cos(data.roll/2.)*sin(data.pitch/2.)*sin(data.yaw/2.);
		vo_data.pose.pose.orientation.z = data.yaw;//cos(data.roll/2.)*sin(data.pitch/2.)*cos(data.yaw/2.) + sin(data.roll/2.)*cos(data.pitch/2.)*sin(data.yaw/2.);
		vo_data.pose.pose.orientation.w = 0;//cos(data.roll/2.)*cos(data.pitch/2.)*sin(data.yaw/2.) - sin(data.roll/2.)*sin(data.pitch/2.)*cos(data.yaw/2.);
		if (same_data <3)
		{
			for (i=0, j=0 ;i<6 || j<6;i++, j++) vo_data.pose.covariance[6*i+j] = 0.01;
			for (i=0, j=0 ;i<3 || j<3;i++, j++) raw_imu_data.orientation_covariance[3*i+j] = 99999;
			for (i=0, j=0 ;i<3 || j<3;i++, j++) raw_imu_data.angular_velocity_covariance[3*i+j] = 99999;
			for (i=0, j=0 ;i<3 || j<3;i++, j++) raw_imu_data.linear_acceleration_covariance[3*i+j] = 99999;
		}
		else if (same_data == 3)
		{
			for (i=0, j=0 ;i<6 || j<6;i++, j++) vo_data.pose.covariance[6*i+j] = 99999;
			for (i=0, j=0 ;i<3 || j<3;i++, j++) raw_imu_data.orientation_covariance[3*i+j] = 0.01;
			for (i=0, j=0 ;i<3 || j<3;i++, j++) raw_imu_data.angular_velocity_covariance[3*i+j] = 0.01;
			for (i=0, j=0 ;i<3 || j<3;i++, j++) raw_imu_data.linear_acceleration_covariance[3*i+j] = 0.01;
		}

		for (i=0, j=0 ;i<6 || j<6;i++, j++) vo_data.twist.covariance[6*i+j] = 99999;

		raw_imu_data.header.stamp = data.time_imu;
		raw_imu_data.header.frame_id = "base_footprint";
		raw_imu_data.orientation.x = cos(data.imuRoll/2.)*cos(data.imuPitch/2.)*cos(data.imuYaw/2.) + sin(data.imuRoll/2.)*sin(data.imuPitch/2.)*sin(data.imuYaw/2.);
		raw_imu_data.orientation.y = sin(data.imuRoll/2.)*cos(data.imuPitch/2.)*cos(data.imuYaw/2.) - cos(data.imuRoll/2.)*sin(data.imuPitch/2.)*sin(data.imuYaw/2.);
		raw_imu_data.orientation.z = cos(data.imuRoll/2.)*sin(data.imuPitch/2.)*cos(data.imuYaw/2.) + sin(data.imuRoll/2.)*cos(data.imuPitch/2.)*sin(data.imuYaw/2.);
		raw_imu_data.orientation.w = cos(data.imuRoll/2.)*cos(data.imuPitch/2.)*sin(data.imuYaw/2.) - sin(data.imuRoll/2.)*sin(data.imuPitch/2.)*cos(data.imuYaw/2.);
		

		raw_imu_data.angular_velocity.x = data.rollspeed;
		raw_imu_data.angular_velocity.y = data.pitchspeed;
		raw_imu_data.angular_velocity.z = data.yawspeed;
		

		raw_imu_data.linear_acceleration.x = data.xacc/100.;
		raw_imu_data.linear_acceleration.y = data.yacc/100.;
		raw_imu_data.linear_acceleration.z = data.zacc/100.;
		

		vo_pub.publish(vo_data);
		imu_data_pub.publish(raw_imu_data);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
