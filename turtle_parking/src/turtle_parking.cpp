#include <ros/ros.h>
#include "std_msgs/String.h"
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <math.h>

using namespace ros;
using namespace aruco_msgs;
using namespace geometry_msgs;

double distance = 0;
double angle = 0;
double offset = 0;
double X, Y, Z, W;

int state = 0;
int state2 = 0;

bool flag = false;
tf2::Vector3 rpy;

tf2::Vector3 convertQuaternion2RPY(const tf2::Quaternion quaternion) {
  tf2::Matrix3x3 matrix(quaternion);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);
  tf2::Vector3 rpy(roll, pitch, yaw);
  return rpy;
}

void chatterCallback(MarkerArray msg) {

  flag = true;
  
    distance = msg.markers[0].pose.pose.position.z;
    offset = msg.markers[0].pose.pose.position.x;
  
    X = msg.markers[0].pose.pose.orientation.x;
    Y = msg.markers[0].pose.pose.orientation.y;
    Z = msg.markers[0].pose.pose.orientation.z;
    W = msg.markers[0].pose.pose.orientation.w;  

  
}

int main(int argc, char* argv[]) {

  init(argc, argv, "turlte_parking");
  NodeHandle nh;
 
  Publisher myPub = nh.advertise<Twist>("cmd_vel", 1000);
  Subscriber mySub = nh.subscribe("markers", 1000, chatterCallback);  
  Rate loop_rate(10);
  Twist twist;

  while(ok()) 
  {

//     while(1){
//   // quaterToEuler(X, Y, Z, W, roll, pitch, yaw);

  

//  // ROS_INFO("[%f]", rpy.getX() * 180 / 3.14);
//   tf2::Quaternion quaternion(X, Y, Z, W);
//   rpy = convertQuaternion2RPY(quaternion);
//  // ROS_INFO("[%f]", x);
//   ROS_INFO("[%f]", rpy.getY() * 180 / 3.14);

//       spinOnce();
//       loop_rate.sleep();
// }

    
    if (flag) {
      flag = false;

      tf2::Quaternion quaternion(X, Y, Z, W);
      rpy = convertQuaternion2RPY(quaternion);

      if (state == 0) {
        if (state2 == 0){
            ROS_INFO("0 [%f]", offset);
          twist.linear.x = 0;
          if (offset < -0.01) 
            twist.angular.z = 0.15;
          if (offset > 0.01) 
            twist.angular.z = -0.15;
          if (offset < 0.01 and offset > -0.01) {
            twist.angular.z = 0;
            state = 1;
          }  
        }

        if (state2 == 1){
            ROS_INFO("0 [%f]", offset);
          twist.linear.x = 0;
          if (offset < -0.17) 
            twist.angular.z = 0.15;
          if (offset > -0.15) 
            twist.angular.z = -0.15;
          if (offset < -0.15 and offset > -0.17) {
            twist.angular.z = 0;
            state = 1;
          }  
        }
      }

        if (state == 1) {
          twist.angular.z = 0;
         // state2 = 1;
          ROS_INFO("1 [%f]", distance); 
          if (distance > 0.31) 
            twist.linear.x = 0.15;
          if (distance < 0.29) 
            twist.linear.x = -0.15;
          if (distance > 0.29 and distance < 0.31) {
            twist.linear.x = 0;
            state = 2;
          }
        }

        if (state == 2) {
          ROS_INFO("2 [%f]", rpy.getY());
          twist.linear.x = 0;
          if (rpy.getY() < -0.025) 
            twist.angular.z = 0.15;
          if (rpy.getY() > 0.025) 
            twist.angular.z = -0.15;
          if (rpy.getY() > -0.025 and rpy.getY() < 0.025) {
            twist.angular.z = 0;
            state = 3;
          }
        }

        if (state == 3) {
          ROS_INFO("3 [%f]", distance);
          twist.angular.z = 0; 
          if (distance > 0.61) 
            twist.linear.x = 0.15;
          if (distance < 0.59) 
            twist.linear.x = -0.15;
          if (distance > 0.59 and distance < 0.61) {
            twist.linear.x = 0;
            state = 0;
          }
        }

        if (distance > 0.59 and distance < 0.61 and offset > -0.02 and offset < 0.02 and rpy.getY() < 0.075 and rpy.getY() > -0.075 ) {
          state = -1;
          twist.linear.x = 0;
          twist.angular.z = 0;
          ROS_INFO("Done!");
        }
    }
    else {
      twist.angular.z = 0;
      twist.linear.x = 0;
    }

    myPub.publish(twist);
    spinOnce();
    loop_rate.sleep();
  }

  spin();
}