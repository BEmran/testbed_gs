/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 9, 2017, 12:44 PM
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>

/*****************************************************************************************
encodersCallback: generate a trasnform from the encoder data
*****************************************************************************************/
void encodersCallback(const geometry_msgs::Vector3Stamped& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin( tf::Vector3(0.0, 0.0, 1.0) );
    q.setRPY(msg.vector.x, msg.vector.y, msg.vector.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "testbed"));
}

/*****************************************************************************************
main: Run main function
*****************************************************************************************/
int main(int argc, char** argv){
    ros::init(argc, argv, "testbed_tf_basic");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("testbed/sensors/encoders", 10, &encodersCallback);

    ros::spin();
    return 0;
};
