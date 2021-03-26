#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/duration.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include <iostream>
#include <cmath>

#define DEBUG 1
#define RAD_FROM_DEG(x) (x * PI/180)

#if DEBUG
#define CHANGE_STATE(newState) do { \
  state = newState; \
  std::cout << "STATE_CHANGE Changed state to " << #newState << std::endl; \
  } while(0)
#else
#define CHANGE_STATE(newState) state = newState
#endif

enum State { START, WALL_IN_FRONT, WALL_ON_RIGHT };

const double PI = 3.1415926535897932384626433832795;
const float HUG_DIST = 0.5;
const int comp_angle = 20;
const float threshold_parallel = 0.005;
const float linear_speed = 0.2;
const double angular_speed = PI/2;
float front = 0.0, back = 0.0, center = 0.0;

sensor_msgs::LaserScan data;
nav_msgs::Odometry odom;


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
  for(int i = 0; i < 360; i++) {
    data.ranges[i] = laser_msg->ranges[i];
    data.intensities[i] = laser_msg->intensities[i];
  }
  front = data.ranges[270 + comp_angle];
  back = data.ranges[270 - comp_angle];
  center = data.ranges[270];
}

void callback_odometry(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  odom.pose = odom_msg->pose;
  odom.twist = odom_msg->twist;
}

float front_turn_angle() {
  float a = data.ranges[0], b = data.ranges[comp_angle], C = RAD_FROM_DEG(comp_angle);
  float wall_dist = sqrt(pow(a,2) + pow(b,2) - 2*a*b*cos(C));
  float wall_angle = acos((pow(a,2) + pow(wall_dist,2) - pow(b,2))/(2*a*wall_dist));
  return PI - wall_angle;
}

float right_turn_angle() {
  float C = RAD_FROM_DEG(comp_angle);
  float wall_dist = sqrt(pow(center,2) + pow(front,2) - 2*center*front*cos(C));
  float wall_angle = acos((pow(center,2) + pow(wall_dist,2) - pow(front,2))/(2*center*wall_dist));
  return PI/2 - wall_angle;
}

bool there_is_wall(){
  return !isinf(data.ranges[0]);
}

bool wall_in_front(float factor = 1.0){
  return data.ranges[0] <= factor*HUG_DIST + 0.05;
}

bool wall_on_right(float factor = 1.0){
  return !isinf(center) && (center <= factor*HUG_DIST + 0.05);
}

void turn(float radians, ros::Publisher pub, float linvel = 0.0) {
  float duration = abs(radians)/angular_speed;
  geometry_msgs::Twist msg;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.linear.x = linvel;
  msg.angular.z = (radians < 0.0)?(-angular_speed):(angular_speed);
#if DEBUG
  std::cout << "Should take " << duration << " seconds to turn " << radians << std::endl;
#endif
  pub.publish(msg);
  
  ros::Duration(abs(radians)/angular_speed).sleep();
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapmaker");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, callback_laser);
	
#if DEBUG
  std::cout << "initialized publisher and subscriber" << std::endl;
#endif
  ros::Rate rate(4);
  rate.sleep();
#if DEBUG
  std::cout << "Initialized rate and slept" << std::endl;
#endif
  ros::spinOnce();
#if DEBUG
  std::cout << "Spun once" << std::endl;
#endif
  State state = START;
  geometry_msgs::Twist msg;
#if DEBUG
  std::cout << "initialized state and msg" << std::endl;
#endif
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  float which_way = 0.0;
  bool searching_state = false;
  bool corner = false;
  bool turning = false;

#if DEBUG
  std::cout << "Starting" << std::endl;
#endif
  
  float turn_angle;
  while (ros::ok()) {
    switch(state) {

    case START:
      if(wall_on_right()) {
	CHANGE_STATE(WALL_ON_RIGHT);
      }
      else if(wall_in_front()) {
	CHANGE_STATE(WALL_IN_FRONT);
      }
      else {
	msg.linear.x = linear_speed;
	msg.angular.z = 0.0;
	pub.publish(msg);
	rate.sleep();
      }
      break;
    case WALL_IN_FRONT:
      turn_angle = front_turn_angle();
      turn(turn_angle, pub);
      CHANGE_STATE(WALL_ON_RIGHT);
#if DEBUG
      std::cout << "Turned " << turn_angle << " rad to be || to wall" << std::endl;
#endif
    case WALL_ON_RIGHT:
      msg.linear.x = linear_speed;
      msg.angular.z = 0.0;
      pub.publish(msg);

    default: //This should never happen
      std::cerr << "ERROR: unknown state " << state << std::endl;
      exit(state);
    }
   ros::spinOnce();
  }
}	
