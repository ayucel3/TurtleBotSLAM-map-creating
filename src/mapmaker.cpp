#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>

bool turn = false;
double distance;

//Taking the vision info
void callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
	::distance = laser_msg->ranges[0];

	//front
	double front = laser_msg->ranges[0];

	//right
	double right = laser_msg->ranges[89];

	//back
	double back = laser_msg->ranges[179];

	//left
	double left = laser_msg->ranges[269];

	if(front < 1){
		::turn = true;
	}
	else{
		::turn = false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapmaker");

	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	ros::Subscriber sub_laser = nh.subscribe("/scan", 100, callback_laser);
	
	
	ros::Rate rate(4);
	rate.sleep();

	while (ros::ok()) {
		//ROS_INFO_STREAM(::distance);
		geometry_msgs::Twist msg;
		if(::turn == true){
			msg.angular.z = 1.9;
			msg.linear.x = -0.1;//if it is stuck it might help
		}
		else{
			msg.linear.x = 0.2;
		}
        	pub.publish(msg);   //This line is for publishing. It publishes to '/cmd_vel' 

		ros::spinOnce();
	}
}	
