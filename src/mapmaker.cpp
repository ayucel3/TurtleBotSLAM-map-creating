#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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

enum State { FOLLOW_WALL, CHECK_CORNER, TURN_RIGHT ,START , TURN_LEFT};

const double PI = 3.41592653589793238;
const float HUG_DIST = 0.5;
const int comp_angle = 5;
const float threshold_parallel = 0.005;
const float linear_speed = 0.2;
const double angular_speed = PI/8;
float front = 0.0, back = 0.0, center = 0.0;

sensor_msgs::LaserScan data;


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

bool there_is_wall(){
	if(!isinf(data.ranges[0])){return true;}
	return false;
}

bool wall_found(){
	if(data.ranges[0] < HUG_DIST){
		return true;
	}
	return false;
}

bool wall_on_right(){
 if(isinf(center) or (center > HUG_DIST + 0.05) ){//to have some upper limit biger than hug dist to give some capability to bot
	return false;
 }
 return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapmaker");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, callback_laser);
	
	
  ros::Rate rate(4);
  rate.sleep();

  ros::spinOnce();

  State state = START;
  geometry_msgs::Twist msg;

  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  float which_way = 0.0;
  bool searching_state = false;
  bool corner = false;

#if DEBUG
  std::cout << "Starting" << std::endl;
#endif

  while (ros::ok()) {
    switch(state) {

    case START: //JUST TO FIND THE FIRST WALL THERE IS GAP ON THE RIGHT SIDE WHICH thats why needed to seperate inital state after that we ll hopefully have the wall on our right side 
	if(wall_found()){
	  CHANGE_STATE(FOLLOW_WALL);
		searching_state = false;
	}
	else{
		msg.angular.z = 0;
		msg.linear.x = linear_speed;
	}

        break;
      
    case FOLLOW_WALL: //MAIN STATEMENT robot always trys to move forward and if there is wall stops to turn and if the right side is not a wall calls turn right
	if(wall_found()){
		msg.angular.z = angular_speed;
		msg.linear.x = 0;
		CHANGE_STATE(CHECK_CORNER);
	}
	else if(!wall_on_right()){//This is to be able to turn right when there is no wall but we can not do this when we start because chairs on the right gives inf at the start so agent goes crazy thats why we needed start state to just move the agent to wall directly.
	  CHANGE_STATE(TURN_RIGHT);
		
	}
	else{
		msg.angular.z = 0;
		msg.linear.x = linear_speed;
	}

        break;

      
    case CHECK_CORNER: //THIS Statement is just for getting sleep time and detecting corners when FOLLOW_WALL detects there is a wall a head
	if(wall_on_right() and wall_found()){ //for corner
		sleep(1);
		CHANGE_STATE(TURN_LEFT);
		corner = true;
	}
	else if(wall_on_right()){ //for just a side
	  CHANGE_STATE(FOLLOW_WALL);
	}
	else{
		sleep(1);
	}
      break;
      
    case TURN_RIGHT: //IF we are in searching state which means the right wall is lost or too far it will enter the first if and searching state will be set to true and also it will start turning to right
	if(!wall_on_right()){
		if(searching_state){// This code only runs once when it is in search state sleep is used to be able to turn the agent 
			sleep(1);
			CHANGE_STATE(START);
			
		}
		else {
		  msg.angular.z = -angular_speed*2;
		  msg.linear.x = 0;
		  searching_state = true;
		}
	}
	else{
		sleep(1);
		CHANGE_STATE(FOLLOW_WALL);//THis part is to set agent to normal state which is going forward and finding walls
	}
      break;

    case TURN_LEFT: //This is to handle corners
	if(corner){
		msg.angular.z = angular_speed;
		msg.linear.x = 0;
		corner = false;
	}
	else{
		sleep(1);
		CHANGE_STATE(FOLLOW_WALL);
	}
      break;

    default: //This should never happen
      std::cerr << "ERROR: unknown state " << state << std::endl;
      exit(state);
    }
   rate.sleep();
   ros::spinOnce();
   pub.publish(msg);
  }
}	
