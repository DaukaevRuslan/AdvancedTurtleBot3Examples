#include <ros/ros.h>
#include "std_msgs/String.h"
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>

using namespace ros;
using namespace aruco_msgs;
using namespace geometry_msgs;

double z = 0;
double orZ = 0;

//MarkerArray msg2;

float _err_measure = 0.005;  // примерный шум измерений
float _q = 0.9;   // скорость изменения значений 0.001-1, варьировать самому

double simpleKalman(double newVal) {
  double _kalman_gain, _current_estimate;
  static double _err_estimate = _err_measure;
  static double _last_estimate;
  _kalman_gain = (double)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (double)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}

double computePID(double input, double setpoint, double kp, double ki, double kd, double dt) {
  double err = setpoint - input;
  static double integral = 0, prevErr = 0;
  integral = integral + (double)err * dt * ki;
  double D = (err - prevErr) / dt;
  prevErr = err;
  return (err * kp + integral + D * kd);
}

void chatterCallback(MarkerArray msg)
{

  
  //ROS_INFO("[%di]", z);
  
  z = msg.markers[0].pose.pose.position.z;
  z = simpleKalman(z);
  ROS_INFO("[%f]", z);

}

int main(int argc, char* argv[])
{
  init(argc, argv, "turlteRus_move123");
  NodeHandle nh;
  
  ROS_INFO("Hello, World!");
  //ROS_INFO("[%di]", msg->pose.pose.position.z);

  //Publisher pub = nh.advertise<Marker>("ALLO", 1000);
  Publisher pub2 = nh.advertise<Twist>("cmd_vel", 1000);
  Subscriber sub = nh.subscribe("markers", 1000, chatterCallback);
  Rate loop_rate(10);

  while(ok())
  {

    double vel = 0.0;
    double predVel = 0.0;
    vel = -computePID(z, 0.75, 0.5, 0.01, 0.1, 0.1);
  // if(z > 0.75)
  //   vel = 0.02;
  // if(z < 0.70)
  //   vel = -0.02;
    //vel = 0.5*(z - 0.75) + ((z - 0.75)/1 * 0.01);
    predVel = vel;
    //Marker marker;
    Twist twist;
    //marker.pose.pose.position.x = x;
    twist.linear.x = vel;
    //pub.publish(marker);
    pub2.publish(twist);
    spinOnce();
    loop_rate.sleep();
    
    //x += 1;
  }

  spin();
}