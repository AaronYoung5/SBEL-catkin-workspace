#include "ros/ros.h"
#include "common_msgs/Control.h"
int main(int argc,char**argv){
	ros::init(argc,argv,"figure_8_control");

	ros::NodeHandle n;
	ros::Duration(5).sleep();
	ros::Publisher chatter_pub = n.advertise<common_msgs::Control>("control",1);
	ros::Rate loop_rate(10);

	float steering = -0.05;
	bool toggleSpin = true;
	int count = 0;
	while(ros::ok()){
		common_msgs::Control msg;
		msg.throttle = 0.12;
		if(toggleSpin)
			steering = 0.45;
		else
			steering = -0.55;
		if(count%10==0)
			toggleSpin = !toggleSpin;
		msg.steering = steering;
		chatter_pub.publish(msg);
		loop_rate.sleep();
		++count;
		
	}
	return 0;
}
