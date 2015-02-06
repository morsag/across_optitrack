#include <nav_msgs/Odometry.h>
#include <controller/OptitrackPoseVel.h>
#include <roscopter/Mavlink_RAW_IMU.h>
#include <roscopter/Attitude.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

using namespace std;

class kalman_filter_data{
public:
	kalman_filter_data();
	void optitrack_callback(const controller::OptitrackPoseVel& msg);
	void imu_callback(const roscopter::Mavlink_RAW_IMU& msg);
	void attitude_callback(const roscopter::Attitude& msg);

	float 	xacc, yacc, zacc, pose_x, pose_y, pose_z, roll, pitch, yaw, imuRoll, imuPitch, imuYaw, rollspeed, yawspeed,	pitchspeed;
	ros::Time time_imu, time_optitrack;
};
