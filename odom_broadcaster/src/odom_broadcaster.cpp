#include "ros/ros.h"
//#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
 

// Initial pose
double orientSum=0, posXSum=0, posYSum=0;
const double PI = 3.141592;
 
// Robot physical constants
const double WHEEL_BASE = 0.38; // Center of left tire to center of right tire
 
// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;
 
ros::Time l_lasttime, l_currtime;
void Calc_Left(const std_msgs::Float32& leftSpeed) 
{
	l_currtime=ros::Time::now();
	ros::Duration dt=l_currtime-l_lasttime;
	l_lasttime=l_currtime;
	distanceLeft += leftSpeed.data * dt.toSec();
}
 
ros::Time r_lasttime, r_currtime;
void Calc_Right(const std_msgs::Float32& rightSpeed)
{   
	r_currtime=ros::Time::now();
	ros::Duration dt=r_currtime-r_lasttime;
	r_lasttime=r_currtime;
	distanceRight += rightSpeed.data * dt.toSec();
}
  
int main(int argc, char **argv) 
{
	// Launch ROS and create a node
	ros::init(argc, argv, "odometry_publisher");
	ros::NodeHandle node;
	
	ros::Publisher odom_pub;
	tf::TransformBroadcaster odom_broadcaster;	

	// Subscribe to ROS topics
	ros::Subscriber subForRightCounts = node.subscribe("right_wheel_speed", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
	ros::Subscriber subForLeftCounts = node.subscribe("left_wheel_speed", 100, Calc_Left, ros::TransportHints().tcpNoDelay());

	odom_pub = node.advertise<nav_msgs::Odometry>("odom", 100);

	double lastDist=0, lastDiffDist=0;
	ros::Time lastTime, currTime;

	ros::Rate loop_rate(30); 
	std::cout<<"odom_broadcaster!!"<<std::endl;
	 
	while(node.ok()) 
	{
		ros::spinOnce();

		currTime=ros::Time::now();
		ros::Duration dt=currTime-lastTime;
		float deltat=dt.toSec();
		lastTime=currTime;

		if(deltat<0.00001) deltat=1/30;

		// Calculate the average distance
		double totDist=(distanceRight + distanceLeft) / 2;
		double cycleDistance = totDist-lastDist;
		lastDist=totDist;

		// Calculate the number of radians the robot has turned since the last cycle
		double totDiffDist= (distanceRight - distanceLeft);
		double cycleDiffDistance = totDiffDist-lastDiffDist;
		lastDiffDist=totDiffDist;
		double cycleAngle = asin((cycleDiffDistance)/WHEEL_BASE);

		// Calculate the new pose (x, y, and theta)
		orientSum += cycleAngle;
		posXSum += cos(orientSum)*cycleDistance;
		posYSum += sin(orientSum)*cycleDistance; 

		// Compute the velocity
		double linearSpd = cycleDistance/deltat;
		double angularSpd = cycleAngle/deltat;
		   
		if (orientSum > PI) orientSum -= 2*PI;
		if (orientSum < -PI) orientSum += 2*PI;
		 
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(orientSum);

		nav_msgs::Odometry odometry;
		odometry.header.stamp = ros::Time::now();
		odometry.header.frame_id = "odom";
		odometry.child_frame_id = "base_link";
		odometry.pose.pose.position.x = posXSum;
		odometry.pose.pose.position.y = posYSum;
		odometry.pose.pose.position.z = 0;
		odometry.pose.pose.orientation=odom_quat;
		odometry.twist.twist.linear.x = linearSpd;
		odometry.twist.twist.linear.y = 0;
		odometry.twist.twist.linear.z = 0;
		odometry.twist.twist.angular.x = 0;
		odometry.twist.twist.angular.y = 0;
		odometry.twist.twist.angular.z = angularSpd;

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = odometry.header.stamp;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = posXSum;
		odom_trans.transform.translation.y = posYSum;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation=odom_quat;

		odom_broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odometry);

	
		loop_rate.sleep();
	}
	std::cout<<"odom_broadcaster DEAD!!"<<std::endl;
	return 0;
}