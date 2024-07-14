
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <geometry_msgs/Twist.h>

extern int8_t sensor_buff[2];
int right_x;
double left_x, left_y;
int comma=0;
ros::NodeHandle nh;
std_msgs::Int8MultiArray Sensor_data;
//std_msgs::String color_data;


//void coordinate(const std_msgs::Int16& msg){
//	reset_encoder = msg.data;
//	reset_flag =1;
//}
//extern char color_buffer[]='';
void call_back(const geometry_msgs::Twist& cmd_vel){
	right_x = cmd_vel.angular.z;
	left_x = cmd_vel.linear.x;
	left_y = -cmd_vel.linear.y;
}
void seed(const std_msgs::Int8& seed){
	 comma = seed.data;
}
ros::Publisher sensor("sensor", &Sensor_data);
ros::Subscriber <geometry_msgs::Twist> joy("cmd_vel", &call_back);
ros::Subscriber<std_msgs::Int8>seedl("seed", &seed);
//ros::Subscriber<std_msgs::Int16> reset_en("coordinate", &coordinate);
//ros::Publisher color_sensor("color", &color_data);
void setup(void){
	nh.initNode();
	nh.advertise(sensor);
	nh.subscribe(joy);
	nh.subscribe(seedl);
//	nh.subscribe(reset_en);
//	nh.advertise(color_sensor);
}

void loop(void){
//		color_data.data= color_buffer;
		Sensor_data.data_length =2;
		Sensor_data.data= sensor_buff;
		sensor.publish(&Sensor_data);
//		color_sensor.publish(&color_data);
		nh.spinOnce();
}
