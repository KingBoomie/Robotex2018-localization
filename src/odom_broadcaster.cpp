#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <serial/WheelSpeed.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;

double inv_coup_data[3][4] = {{ -6.27484877e-05,  -6.27485161e-05,   6.27484592e-05, 6.27485446e-05},
                         { -6.27485446e-05,   6.27484877e-05,   6.27485446e-05,-6.27484877e-05},
                         {  4.43699063e-05,   4.43698862e-05,   4.43698862e-05, 4.43699063e-05}};

Mat inv_coup = Mat(3, 4, CV_64F, inv_coup_data);

ros::Time current_time, last_time;

void wheel_cb(const serial::WheelSpeed::ConstPtr& msg, const ros::Publisher& odom_pub) {
    std::cout << "Called odom callback\n";
    current_time = ros::Time::now();

    double motor_speeds_data[4][1] = { msg->wheel1, msg->wheel2, msg->wheel3, msg->wheel4 };
    Mat motor_speeds = Mat(4,1, CV_64F, motor_speeds_data);

    Mat euclidean_speeds = inv_coup * motor_speeds;

    double x = euclidean_speeds.at<double>(0,0);
    double y = euclidean_speeds.at<double>(0,1);
    double phi = -euclidean_speeds.at<double>(0,2);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(phi);


    nav_msgs::Odometry odom;
    // header data
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x += x;
    odom.pose.pose.position.y += y;
    odom.pose.pose.orientation = odom_quat;

    // velocity
    odom.twist.twist.linear.x = x;
    odom.twist.twist.linear.y = y;
    odom.twist.twist.angular.z = phi;

    // publish the msg
    odom_pub.publish(odom);

    last_time = current_time;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber wheel_sub = n.subscribe<serial::WheelSpeed>("wheelspeed", 50, boost::bind(wheel_cb, _1, boost::ref(odom_pub)));
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(10);
    while(n.ok()){

        ros::spinOnce();               // check for incoming messages
        /*current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;*/
        r.sleep();
    }
}