#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include <iostream>

#define DEBUG 1

enum State { START, TURN_LEFT, ALONG_WALL, TURN_RIGHT };

const double PI = 3.41592653589793238;
const float HUG_DIST = 0.5;
const int comp_angle = 10;

sensor_msgs::LaserScan data;

bool equal(float a, float b) {
  float ratio = a/b;
  return (ratio > 0.9999) && (ratio < 1.0001);
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
}

bool parallel_to_wall() {
  return equal(data.ranges[90 - comp_angle], data.ranges[90 + comp_angle]);
}

float which_direction() {
  //parallel: return 0
  if(parallel_to_wall()) {
    return 0.0;
  }
  //wall turns left: return <0
  //wall turns right: return >0
  return data.ranges[90 - comp_angle] - data.ranges[90 + comp_angle];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapmaker");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Subscriber sub_laser = nh.subscribe("/scan", 100, callback_laser);
	
	
  ros::Rate rate(4);
  rate.sleep();

  State state = START;
  geometry_msgs::Twist msg;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  float which_way = 0.0;
  while (ros::ok()) {
    switch(state) {
    case START:
      if(data.ranges[0] <= HUG_DIST) {
	state = TURN_LEFT;
      }
      else {
	msg.angular.z = 0.0;
	msg.linear.x = 1.0;
      }
      break;
    case TURN_LEFT:
      if(parallel_to_wall()) {
	state = ALONG_WALL;
      }
      else {
	msg.angular.z = PI/2;
	msg.linear.x = 0.0;
      }
      break;
    case ALONG_WALL:
      which_way = which_direction();
      if(which_way < 0.0) {
	state = TURN_LEFT;
      }
      else if(which_way > 0.0) {
	state = TURN_RIGHT;
      }
      else {
	msg.linear.x = 1.0;
	msg.angular.z = 0.0;
      }
      break;
    case TURN_RIGHT:
      if(parallel_to_wall()) {
	state = ALONG_WALL;
      }
      else {
	msg.angular.z = -PI/2;
	msg.linear.x = 0.0;
      }
      break;
    default:
      std::cerr << "Error: unknown state " << state << std::endl;
      exit(state);
    }
    
    pub.publish(msg);   //This line is for publishing. It publishes to '/cmd_vel' 

    ros::spinOnce();
  }
}	
