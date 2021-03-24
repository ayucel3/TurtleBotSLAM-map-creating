#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include <iostream>

#define DEBUG 1

enum State { START, TURN_LEFT, ALONG_WALL, TURN_RIGHT };

const double PI = 3.41592653589793238;
const float HUG_DIST = 0.5;
const int comp_angle = 5;
const float threshold_parallel = 0.005;
const float linear_speed = 0.2;
const double angular_speed = PI/8;
float front = 0.0, back = 0.0, center = 0.0;

sensor_msgs::LaserScan data;

//I DID NOT USE THIS since the sensors are not giving exactly what we want instead I used a threshold
bool equal(float a, float b) {
  float ratio = a/b;
  float threshold = 0.995;
  return (ratio > threshold) && (ratio < 1/threshold);
}

//Taking the vision info
void callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
  data.header = laser_msg->header;
  data.angle_min = laser_msg->angle_min;
  data.angle_max = laser_msg->angle_max;
  data.angle_increment = laser_msg->angle_increment;
  data.scan_time = laser_msg->scan_time;
  data.range_min = laser_msg->range_min;
  data.range_max = laser_msg->range_max;
  data.ranges = laser_msg->ranges;
  data.intensities = laser_msg->intensities;
  front = data.ranges[270 + comp_angle];
  back = data.ranges[270 - comp_angle];
  center = data.ranges[270];
}

bool parallel_to_wall() {
  return equal(front, back);
}

bool wall_getting_closer() {
  return !equal(front, back) && (front < back);
}

bool wall_getting_further() {
  return !equal(front, back) && (front > back);
}

// bool keep_going(){
// 	if (isinf(center) or data.ranges[0] < HUG_DIST){
// 		return false;
// 	}
// 	return true;
// }


float which_direction() {
  //parallel: return 0
  if(parallel_to_wall()) return 0.0;
  if(isinf(front) && isinf(center) && isinf(back)) return 0.0;
  if(isinf(front)) return 1.0;
  //wall turns left: return <0
  //wall turns right: return >0
  return front - back;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapmaker");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Subscriber sub_laser = nh.subscribe("/scan", 1, callback_laser);
	
	
  ros::Rate rate(4);
  rate.sleep();
  State state = START;
  geometry_msgs::Twist msg;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  float which_way = 0.0;
  bool should_publish = true;
  while (ros::ok()) {
    ros::spinOnce();
    switch(state) {
    	case START:
	  //ROS_INFO("START");
      		if(data.ranges[0] <= HUG_DIST) {
			state = TURN_LEFT;
			should_publish = false;
#if DEBUG
			std::cout << "STATE_CHANGE Found a wall. Turning left" << std::endl;
#endif
      		}
      		else {
			msg.angular.z = 0.0;
			msg.linear.x = linear_speed;
			should_publish = true;
      		}
      		break;
    	case TURN_LEFT:
	  //ROS_INFO("TURN LEFT");
      		if(parallel_to_wall()) {
			state = ALONG_WALL;
			should_publish = false;
#if DEBUG
			std::cout << "STATE_CHANGE Now parallel to wall" << std::endl;
#endif
      		}
      		else {
			msg.angular.z = angular_speed;
			msg.linear.x = 0.0;
			should_publish = true;
      		}
      		break;
    	case ALONG_WALL:
	  //ROS_INFO("ALONG WALL");
      		which_way = which_direction();
      		if(wall_getting_closer()) {
#if DEBUG
		  std::cout << "STATE_CHANGE Wall is getting closer. Turning left" << std::endl;
#endif
			state = TURN_LEFT;
			should_publish = false;
      		}
      		else if(wall_getting_further()) {
#if DEBUG
		  std::cout << "STATE_CHANGE Wall is getting further. Turning right" << std::endl;
#endif
			state = TURN_RIGHT;
			should_publish = false;
      		}
      		else {
			msg.linear.x = linear_speed;
			msg.angular.z = 0.0;
			should_publish = true;
      		}
      		break;
    	case TURN_RIGHT:
	  //ROS_INFO("TURN RIGHT");
      		if(parallel_to_wall()) {
			state = ALONG_WALL;
			should_publish = false;
#if DEBUG
			std::cout << "STATE_CHANGE Parallel to wall. About to move forward" << std::endl;
#endif
      		}
      		else {
			msg.angular.z = -angular_speed;
			msg.linear.x = 0.0;
			should_publish = true;
			
      		}
      		break;
    	default:
      		std::cerr << "Error: unknown state " << state << std::endl;
      		exit(state);
    }

    if(should_publish) {
      pub.publish(msg);   //This line is for publishing. It publishes to '/cmd_vel'
    }
    
  }
}	
