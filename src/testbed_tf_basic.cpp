/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 9, 2017, 12:44 PM
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
ros::Publisher attitude_pub;

/******************************************************************************
encodersCallback: generate a trasnform from the encoder data
******************************************************************************/
void encodersCallback(const geometry_msgs::Vector3StampedConstPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 1.0) );
    transform.setRotation(tf::createQuaternionFromRPY(msg->vector.x, msg->vector.y, msg->vector.z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "testbed"));
}

/******************************************************************************
imuCallback: generate a trasnform from the imu data
******************************************************************************/
void imuCallback(const sensor_msgs::Imu& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 1.0));
    transform.setRotation(tf::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "testbed_IMU"));

//    geometry_msgs::Vector3Stamped attitude_msg;
//    attitude_msg.header = msg.header;
//    tf::Matrix3x3(transform.getRotation()).getRPY(attitude_msg.vector.x,attitude_msg.vector.y,attitude_msg.vector.z);
//    attitude_pub.publish(attitude_msg);
}

/******************************************************************************
main: Run main function
******************************************************************************/
int main(int argc, char** argv){
    ros::init(argc, argv, "testbed_tf_basic");

    ros::NodeHandle node;
    ros::Subscriber sub_enc = node.subscribe("testbed/sensors/row/encoders", 10, &encodersCallback);
    ros::Subscriber sub_imu = node.subscribe("testbed/sensors/row/imu", 10, &imuCallback);
//    attitude_pub = node.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/attitude", 1000);

    ros::spin();
    return 0;
};
