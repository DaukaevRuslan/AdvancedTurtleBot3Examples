#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>

bool data = false;
bool flag = false;

int temp1 = 9999;
int temp2 = 0;

void callbackLine(std_msgs::Bool bol) {
	data = bol.data;
}

void callbackColor(std_msgs::Int32MultiArray arr){
	temp1 = arr.data[0];
	temp2 = arr.data[1];
		
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "turtle_line_drive");
	ros::NodeHandle nh;

	ros::Publisher myPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber mySub = nh.subscribe("line_sensor_data", 1000, callbackLine);
	ros::Subscriber mySub1 = nh.subscribe("color_data", 1000, callbackColor);
	ros::Rate loop_rate(10);

	geometry_msgs::Twist twist;

	while (ros::ok()) {
		
		
		if (!(temp1 < 3700)) {
			if (data) {
				twist.linear.x = 0.05;
				twist.angular.z = 0.5;
			}
			else{
				twist.linear.x = 0.05;
				twist.angular.z = -0.5;
			}	
		} else {
			flag = true;
			ROS_INFO("[%i]", temp1);
		}
		
		if (flag) {

			twist.linear.x = 0;
			twist.angular.z = 0;
		}

		myPub.publish(twist);
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
}