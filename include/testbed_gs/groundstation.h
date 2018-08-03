#ifndef GROUNDSTATION_H
#define GROUNDSTATION_H

#include <ros/ros.h>
#include <cstdlib>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <testbed_gs/cmd_angle.h>
#include <testbed_gs/cmd_du.h>

class GroundStation
{
private:
    int _queue_size;
    // ros parameters --------------------------------------------------------
    std::string _name;
    ros::NodeHandle _nh;
    // Define publishs --------------------------------------------------------
    ros::Publisher _pub_du;
    ros::Publisher _pub_ang;
    // Define services --------------------------------------------------------
    ros::ServiceServer _ser_ang;
    ros::ServiceServer _ser_du;
    // Define publish messages ------------------------------------------------
    geometry_msgs::TwistStamped _msg_du;
    geometry_msgs::Vector3Stamped _msg_ang;
    // Angle max min values ---------------------------------------------------
    float _r_ang_max, _r_ang_min;
    float _p_ang_max, _p_ang_min;
    float _y_ang_max, _y_ang_min;
    // du max min values ------------------------------------------------------
    float _z_du_max, _z_du_min;
    float _r_du_max, _r_du_min;
    float _p_du_max, _p_du_min;
    float _y_du_max, _y_du_min;
    // Methods ----------------------------------------------------------------
    void initializeParams();

public:
    float _cmd_ang[3];

    GroundStation(ros::NodeHandle nh, std::string name);
    ~GroundStation();
    float sat (float x, float max, float min);
    void cmdAnglePuplish(float r, float p, float w);
    bool cmdAngleCallback(testbed_gs::cmd_angle::Request  &req,
                          testbed_gs::cmd_angle::Response &res);
    bool cmdDuCallback   (testbed_gs::cmd_du::Request     &req,
                          testbed_gs::cmd_du::Response    &res);
};

#endif // GROUNDSTATION_H
