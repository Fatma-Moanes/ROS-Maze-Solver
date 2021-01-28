#include "../include/pid.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

ros::Publisher motor_pub;
double yaw = 0;
double setpoint = 0;
double linear_speed = 0;
double offset = 0;

void angle_callback(const geometry_msgs::Vector3StampedConstPtr &);
void angle_callback2(const nav_msgs::OdometryConstPtr &);

void stop_motors();

void setpoint_callback(const geometry_msgs::Vector3ConstPtr &);

double calibrate();
///////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rotate_angle");
    ros::NodeHandle node_handle;
    motor_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    //ros::Subscriber angle_sub = node_handle.subscribe("/imu/rpy/filtered", 1, angle_callback);
    ros::Subscriber angle_sub = node_handle.subscribe("/odom", 1000, angle_callback2);
    ros::Subscriber setpoint_sub = node_handle.subscribe("pid_setpoint", 1, setpoint_callback);
    ros::Rate r(50);
    //offset = calibrate();
    double kp = 1;
    double kd = 1;
    double ki = 0;

    PID pid(20, 100, -100, kp, kd, ki);
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    while (ros::ok()) {
        ros::spinOnce();
        double angular_vel = pid.calculate(setpoint, yaw);
        vel_msg.angular.z = angular_vel;
        vel_msg.linear.x = linear_speed;
        motor_pub.publish(vel_msg);
        ros::spinOnce();
        r.sleep();
    }
}

///////////////////////////////////////////////////////////////////////////////////////

void angle_callback(const geometry_msgs::Vector3StampedConstPtr &msg) {
    yaw = msg->vector.z - offset;
}

void angle_callback2(const nav_msgs::OdometryConstPtr &msg) {
    tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);

    //std::cout << yaw*180/3.14 << std::endl;
}

void stop_motors() {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.z = 0;
    motor_pub.publish(vel_msg);
}

void setpoint_callback(const geometry_msgs::Vector3ConstPtr &msg) {
    setpoint = msg->x;
    linear_speed = msg->y;
}

double calibrate() {
    ros::Rate r(50);
    for (int i = 0; i < 20; i++) {
        ros::spinOnce();
        r.sleep();
    }

    //std::cout << "offset = " << yaw << std::endl;
    return yaw;
}
