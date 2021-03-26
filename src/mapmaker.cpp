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

enum State { FOLLOW_WALL, TURN_RIGHT ,START};

const double PI = 3.41592653589793238;
const float HUG_DIST = 1;
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

bool there_is_wall_behind(){
   int i = 180;
   while (i <= 270){
	if(data.ranges[i] < HUG_DIST + 0.01){
		return true;
	}
   	i += 10;
   }
	
  return (false);
}
void turn_right(ros::Publisher pub){
  double right = data.ranges[270];
  geometry_msgs::Twist msg;
  while (true){
  	if(data.ranges[0] - right  < 0.001){
		break;
	}
	msg.linear.x = 0;
	msg.angular.z = -PI/8;
	pub.publish(msg);
	ros::spinOnce();
  }
}
void turn_left(ros::Publisher pub){
  double front = data.ranges[0];
  geometry_msgs::Twist msg;
  if(data.ranges[270] - front < 0.001){//to handle 45 degrees...
   	msg.linear.x = 0;
	msg.angular.z = PI/8;
	pub.publish(msg);
	ros::spinOnce();
  }
  while (true){
  	if(data.ranges[270] - front < 0.001){
		break;
	}
	msg.linear.x = 0;
	msg.angular.z = PI/8;
	pub.publish(msg);
	ros::spinOnce();
  }
}

void go_to_inf(ros::Publisher pub){
  geometry_msgs::Twist msg;
  msg.angular.z = -PI/2;
  msg.linear.x = 0;
  pub.publish(msg);
  ros::Duration(1).sleep();
  while (true){
  	if((data.ranges[0] < 2) and !isinf(data.ranges[350]) and !isinf(data.ranges[10])){
		break;
	}
	msg.linear.x = linear_speed;
	msg.angular.z = 0;
	pub.publish(msg);
	ros::spinOnce();
  }
}

void move_towards_wall(ros::Publisher pub){
  double front = data.ranges[0];
  geometry_msgs::Twist msg;
  while (true){
  	if((data.ranges[0] - front  <= HUG_DIST + 0.1) or (data.ranges[5] < HUG_DIST) or (data.ranges[355] < HUG_DIST)){ // WHEN IT Gets close to hug dist in breaks or these angles become closer to avoid wall collisions it ll break
		break;
	}
	msg.linear.x = linear_speed;
	msg.angular.z = 0;
	pub.publish(msg);
	ros::spinOnce();
  }
}

bool wall_found(float factor = 1.0){
  return data.ranges[0] <= factor*HUG_DIST;
}
bool wall_on_right_inf(){
  return isinf(center);
}

bool wall_on_right_close(float factor = 1.0){
  return (center <= factor*HUG_DIST + 0.05);
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
	}
	else{
		msg.angular.z = 0;
		msg.linear.x = linear_speed;
	}

        break;
      
    case FOLLOW_WALL: //MAIN STATEMENT robot always trys to move forward and if there is wall stops to turn and if the right side is not a wall calls turn right
	if(wall_found()){
		turn_left(pub);
                msg.angular.z = 0;
		msg.linear.x = 0;
	}
		
	else if(!wall_on_right_close() or wall_on_right_inf()){
	  	CHANGE_STATE(TURN_RIGHT);
	}
	else{
		msg.angular.z = 0;
		msg.linear.x = linear_speed;
	}

        break;
      
    case TURN_RIGHT: //IF we are in searching state which means the right wall is lost or too far it will enter the first if and searching state will be set to true and also it will start turning to right
	if (wall_on_right_inf()){
		go_to_inf(pub);//if the right side is inf we want to go there to examine
		CHANGE_STATE(FOLLOW_WALL);
		msg.angular.z = 0;
	}	
	else if(!wall_on_right_close()){
		turn_right(pub);
		move_towards_wall(pub);
		msg.angular.z = 0;
                CHANGE_STATE(FOLLOW_WALL);
	}
	else{
		msg.angular.z = 0;
		CHANGE_STATE(FOLLOW_WALL);//THis part is to set agent to normal state which is going forward and finding walls
	}
	msg.linear.x = linear_speed;//this is what we want if something bad happens it needs to keep moving to get some new sensors if its not stuck
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
