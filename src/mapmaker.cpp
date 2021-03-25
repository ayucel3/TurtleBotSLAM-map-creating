#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include <iostream>
#include <cmath>

#define DEBUG 1
#define RAD_FROM_DEG(x) (x * PI/180)

enum State { WALL_SEARCH, FOUND_WALL, FOLLOW_WALL, CORNER };

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
  float threshold = 0.995;
  if(0.0 != b) {
    float ratio = a/b;
    return (ratio > threshold) && (ratio < 1/threshold);
  }
  return abs(a) < 1/threshold - 1;
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
  return equal(front, back) || equal(cos(RAD_FROM_DEG(comp_angle)), center/front) ||
    equal(cos(RAD_FROM_DEG(comp_angle)), center/back);
}

bool wall_getting_closer() {
  return !equal(front, back) && (front < back);
}

bool wall_getting_further() {
  return !equal(front, back) && (front > back);
}

bool wall_in_front() {
#if DEBUG
  std::cout << "Distance to wall in front is " << data.ranges[0] << std::endl;
#endif
  if(isinf(data.ranges[0])) return false;
  return equal(HUG_DIST, data.ranges[0]) || (data.ranges[0] < HUG_DIST);
}

bool wall_on_right() {
  return !isinf(center) && (equal(HUG_DIST, center) || (center < HUG_DIST));
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

//gets the angle the robot needs to turn to be parallel to the wall in front of it
float get_front_angle() {
  if(isinf(data.ranges[0])) return 0;
  if(isinf(data.ranges[comp_angle])) return RAD_FROM_DEG(comp_angle);
  float wall_dist = sqrt(pow(data.ranges[0],2) + pow(data.ranges[comp_angle],2) - 2*data.ranges[0]*data.ranges[comp_angle]*cos(RAD_FROM_DEG(comp_angle)));
  float turn_angle = acos((pow(wall_dist, 2) + pow(data.ranges[0],2) - pow(data.ranges[comp_angle],2)) / (2*wall_dist*data.ranges[0]));
  return PI - turn_angle;
}

//gets the angle the robot needs to turn to be parallel to the wall on its right
float get_right_angle() {
  if(isinf(data.ranges[0])) return RAD_FROM_DEG(270);
  if(isinf(data.ranges[270+comp_angle])) return RAD_FROM_DEG(270+comp_angle);
  float wall_dist = sqrt(pow(front,2) + pow(center,2) - 2*front*center*cos(RAD_FROM_DEG(comp_angle)));
  float wall_angle = acos((pow(wall_dist,2) + pow(center,2) - pow(front,2))/(2*wall_dist*center));
  return PI/2 - wall_angle;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapmaker");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, callback_laser);
	
	
  ros::Rate rate(4);
  rate.sleep();
  State state = WALL_SEARCH;
  geometry_msgs::Twist msg;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  float which_way = 0.0;
  bool should_publish = true;
#if DEBUG
  std::cout << "Starting" << std::endl;
#endif
  while (ros::ok()) {
    ros::spinOnce();
    switch(state) {
      
    case WALL_SEARCH: //Robot is lost! Where's the wall?
      if(wall_on_right()) {
	state = FOLLOW_WALL;
	should_publish = false;
#if DEBUG
	std::cout << "STATE_CHANGE Wall on right. Following" << std::endl;
#endif
      }
      if(wall_in_front()) {
	state = FOUND_WALL;
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
      
    case FOUND_WALL: //There's a wall in front, so prepare to follow it
      if(wall_on_right()) {
	state = FOLLOW_WALL;
	should_publish = false;
#if DEBUG
	std::cout << "STATE_CHANGE Wall on right. Following" << std::endl;
#endif
      }
      else if(center < 2*HUG_DIST) {
	msg.angular.z = get_right_angle()/2;
	msg.linear.x = 0.0;
	should_publish = true;
      }
      else {
	msg.angular.z = get_front_angle()/2;
	msg.linear.x - 0.0;
	should_publish = true;
      }
      break;
      
    case FOLLOW_WALL: //There's a wall on the right, so follow it
      msg.linear.x = linear_speed;
      should_publish = true;
      if(wall_in_front() && (get_front_angle() > 4*PI/9)) {
	state = CORNER;
	should_publish = false;
#if DEBUG
	std::cout << "STATE_CHANGE Found a corner." << std::endl;
#endif
      }
      else if(isinf(front)) {
	if(isinf(center)) {
	  state = WALL_SEARCH;
	  should_publish = false;
#if DEBUG
	  std::cout << "STATE_CHANGE Lost the wall on the right. Now searching for another" << std::endl;
#endif
	}
	else {
	  msg.angular.z = RAD_FROM_DEG(comp_angle-90);
	}
      }
      else if(wall_on_right() || (center < 1.5*HUG_DIST)) {
	msg.angular.z = get_right_angle()/2;
      }
      else {
	msg.angular.z = PI/2;
      }
      break;
      
    case CORNER: //There's a barrier, so the robot must turn
      if(wall_on_right()) {
	state = FOLLOW_WALL;
	should_publish = false;
#if DEBUG
	std::cout << "STATE_CHANGE Turned the corner. Following new wall" << std::endl;
#endif
      }
      else if(isinf(data.ranges[comp_angle])) {
	msg.angular.z = get_right_angle();
	msg.linear.x = 0.0;
	should_publish = true;
      }
      else {
	msg.angular.z = get_front_angle();
	msg.linear.x = 0.0;
	should_publish = true;
      }
      break;
      
    default: //This should never happen
      std::cerr << "ERROR: unknown state " << state << std::endl;
      exit(state);
    }

    if(should_publish) {
      pub.publish(msg);   //This line is for publishing. It publishes to '/cmd_vel'
    }
#if DEBUG
    else {
      std::cout << "STATE_CHANGE Changed state to " << state << std::endl;
    }
#endif
  }
}	
