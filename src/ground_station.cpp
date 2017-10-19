#include <ros/ros.h>
#include <cstdlib>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <testbed_gs/cmd_angle.h>
#include <testbed_gs/cmd_du.h>

#define _ROLL_MAX 0.25 // radian
#define _ROLL_MIN -0.25 // radian
#define _PITCH_MAX 0.25 // radian
#define _PITCH_MIN -0.25 // radian
#define _YAW_MAX 7 // radian
#define _YAW_MIN -7 // radian

#define _DU_Z_MAX 0.7 // power !
#define _DU_Z_MIN 0.0
#define _DU_R_MAX 0.4
#define _DU_R_MIN -0.4
#define _DU_P_MAX 0.4
#define _DU_P_MIN -0.4
#define _DU_Y_MAX 0.5
#define _DU_Y_MIN -0.5
using namespace std;

class Ground_Station{
    private:
        ros::NodeHandle _nh;
        int _queue_size;
        std::string _name;
        ros::Publisher _pub_du;
        ros::Publisher _pub_ang;
        ros::ServiceServer _ser_ang;
        ros::ServiceServer _ser_du;
        geometry_msgs::TwistStamped _msg_du;
        geometry_msgs::Vector3Stamped _msg_ang;
    public:
        Ground_Station(ros::NodeHandle nh, std::string name){
            _nh = nh;
            _name = name;
            _queue_size = 10;
            _pub_du = nh.advertise<geometry_msgs::TwistStamped>("testbed/cmd/du", _queue_size);
            _pub_ang = nh.advertise<geometry_msgs::Vector3Stamped>("testbed/cmd/angle", _queue_size);
            _ser_ang = nh.advertiseService("testbed/cmd/angle", &Ground_Station::cmdAngleCallback,this);
            _ser_du = nh.advertiseService("testbed/cmd/du", &Ground_Station::cmdDuCallback,this);
        }

        ~Ground_Station(){};
        bool cmdAngleCallback(testbed_gs::cmd_angle::Request  &req, testbed_gs::cmd_angle::Response &res);
        bool cmdDuCallback(testbed_gs::cmd_du::Request  &req, testbed_gs::cmd_du::Response &res);
        float sat (float x, float max, float min);
};

///*****************************************************************************************
//sat: applay saturation limit on input
//******************************************************************************************/
float Ground_Station::sat (float x, float max, float min) {
    if (x >= max)
        x = max;
    else if(x <= min)
        x = min;
    return x;
}
///*****************************************************************************************
//cmdAngleCallback: call back function to genrate a command angle
//******************************************************************************************/
bool Ground_Station::cmdAngleCallback(testbed_gs::cmd_angle::Request  &req, testbed_gs::cmd_angle::Response &res){

    res.ang.x = sat(req.ang.x, _ROLL_MAX, _ROLL_MIN);
    res.ang.y = sat(req.ang.y, _PITCH_MAX, _PITCH_MIN);
    res.ang.z = sat(req.ang.z, _YAW_MAX, _YAW_MIN);

    _msg_ang.header.stamp = ros::Time::now();
    _msg_ang.header.seq++;
    _msg_ang.vector.x = res.ang.x;
    _msg_ang.vector.y = res.ang.y;
    _msg_ang.vector.z = res.ang.z;
    _pub_ang.publish(_msg_ang);

    ROS_INFO("Response#%d: x=%+2.2f, y=%+2.2f, z=%+2.2f", _msg_ang.header.seq, res.ang.x, res.ang.y, res.ang.z);
    return true;
}

///*****************************************************************************************
//cmdDuCallback: call back function to genrate a command du for motor
//******************************************************************************************/
bool Ground_Station::cmdDuCallback(testbed_gs::cmd_du::Request  &req, testbed_gs::cmd_du::Response &res){

    res.du.linear.z = sat(req.du.linear.z, _DU_Z_MAX, _DU_Z_MIN);
    res.du.angular.x = sat(req.du.angular.x, _DU_R_MAX, _DU_R_MIN);
    res.du.angular.y = sat(req.du.angular.y, _DU_P_MAX, _DU_P_MIN);
    res.du.angular.z = sat(req.du.angular.z, _DU_Y_MAX, _DU_Y_MIN);

    _msg_du.header.stamp = ros::Time::now();
    _msg_du.header.seq++;
    _msg_du.twist.linear.z = res.du.linear.z;
    _msg_du.twist.angular.x = res.du.angular.x;
    _msg_du.twist.angular.y = res.du.angular.y;
    _msg_du.twist.angular.z = res.du.angular.z;
    _pub_du.publish(_msg_du);

    ROS_INFO("Request #%d: x=%+2.2f, r=%+2.2f, p=%+2.2f, w=%+2.2f", _msg_du.header.seq,
    res.du.linear.z, res.du.angular.x, res.du.angular.y, res.du.angular.z);
    return true;
}


int main(int argc, char** argv){

    std::string name = "ground_station";
    ros::init(argc, argv, name);
    ros::NodeHandle nh;
    Ground_Station gs(nh,name);

    //------------------------------------- Wait for user input --------------------------------------

    ROS_INFO("Ground station is ready\n");
    ros::spin();
    return 0;
};
