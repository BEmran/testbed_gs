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
#include <math.h>

//ros::Publisher attitude_pub;

/******************************************************************************
encodersCallback: generate a trasnform from the encoder data
******************************************************************************/
void encodersCallback(const geometry_msgs::Vector3StampedConstPtr& msg){

  // Define parameters --------------------------------------------------------
  static tf::TransformBroadcaster br;     // brodcast
  tf::Transform tf_testbed;               // testbed tf
  // quaternion to adjust frames from NWU to NED
  tf::Quaternion quat_nwu2ned = tf::createQuaternionFromRPY(M_PI, 0.0, 0.0);
  // quaternion represents rotation from encoders data
  tf::Quaternion quat_encoderes = tf::createQuaternionFromRPY(msg->vector.x,
                                                              msg->vector.y,
                                                              msg->vector.z);

  // Apply rotation and translation for testbed -------------------------------
  tf_testbed.setOrigin(tf::Vector3(0.0, 0.0, 1.0));       // move tf 1m in z-axis
  tf_testbed.setRotation(quat_nwu2ned * quat_encoderes);  // apply rotation

  // Publish broadcast --------------------------------------------------------
  br.sendTransform(tf::StampedTransform(tf_testbed, ros::Time::now(), "world",
                                        "testbed"));
}

/******************************************************************************
imuCallback: generate a trasnform from the imu data
******************************************************************************/
void imuCallback(const sensor_msgs::Imu& msg){

  // Define parameters --------------------------------------------------------
  static tf::TransformBroadcaster br;     // brodcast
  tf::Transform tf_imu;                   // testbed tf
  // quaternion represents rotation from encoders data
  tf::Quaternion quat_imu(msg.orientation.x, msg.orientation.y,
                          msg.orientation.z, msg.orientation.w);

  // Apply rotation and translation for testbed -------------------------------
  tf_imu.setOrigin(tf::Vector3(0.0, 0.0, 1.0));     // move tf 1m in z-axis
  tf_imu.setRotation(quat_imu);                     // apply rotation

  // Publish broadcast --------------------------------------------------------
  br.sendTransform(tf::StampedTransform(tf_imu, ros::Time::now(), "world",
                                        "testbed_IMU"));
}

/******************************************************************************
main: Run main function
******************************************************************************/
int main(int argc, char** argv){

  // Initilaize ros node ------------------------------------------------------
  std::string name = "testbed_tf_basic";              // define ros node name
  ros::init(argc, argv, name);                        // inialize ros node
  ros::NodeHandle nh;                                 // define ros handle
  ros::Rate loop_rate(50);                            // define ros frequency

  // Initilaize topics --------------------------------------------------------
  ros::Subscriber sub_enc = nh.subscribe("testbed/sensors/row/encoders", 10,
                                         &encodersCallback);
  ros::Subscriber sub_imu = nh.subscribe("testbed/sensors/row/imu", 10,
                                         &imuCallback);
  //    attitude_pub = node.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/attitude", 1000);

  // Main Loop ----------------------------------------------------------------
  ROS_INFO("testbed for basic tf is ready\n");
  while(ros::ok()){

    // ros method to keep sampling time
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
