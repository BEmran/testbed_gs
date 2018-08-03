#include "groundstation.h"

/******************************************************************************
GroundStation: define the ground station object
******************************************************************************/
GroundStation::GroundStation(ros::NodeHandle nh, std::string name){

    // initilaize parameters --------------------------------------------------
    _nh = nh;
    _name = name;
    _queue_size = 10;
    _cmd_ang[0] = 0;
    _cmd_ang[1] = 0;
    _cmd_ang[2] = 0;

    // initilaize topcis ------------------------------------------------------
    _pub_du  = nh.advertise<geometry_msgs::TwistStamped>  ("testbed/cmd/du"   ,
                                                           _queue_size);
    _pub_ang = nh.advertise<geometry_msgs::Vector3Stamped>("testbed/cmd/angle",
                                                           _queue_size);
    _ser_ang = nh.advertiseService("testbed/cmd/angle",
                                   &GroundStation::cmdAngleCallback,this);
    _ser_du  = nh.advertiseService("testbed/cmd/du"   ,
                                   &GroundStation::cmdDuCallback   ,this);

    // initialize parameters using rospack ------------------------------------
    initializeParams();
}

/******************************************************************************
~GroundStation: destroy object
******************************************************************************/
GroundStation::~GroundStation(){

}

//*****************************************************************************
//sat: applay saturation limit on input
//****************************************************************************/
float GroundStation::sat (float x, float max, float min) {
    if (x >= max)
        x = max;
    else if(x <= min)
        x = min;
    return x;
}

//*****************************************************************************
//cmdAngleCallback: call back function to genrate a command angle
//****************************************************************************/
bool GroundStation::cmdAngleCallback(testbed_gs::cmd_angle::Request  &req,
                                     testbed_gs::cmd_angle::Response &res){

    // call publish angle msg -------------------------------------------------
    cmdAnglePuplish(req.ang.x, req.ang.y, req.ang.z);

    // replay to client with requsted comannded angle after saturation --------
    res.ang.x = _cmd_ang[0];
    res.ang.y = _cmd_ang[1];
    res.ang.z = _cmd_ang[2];

    // announce the result which will be published ----------------------------
    ROS_INFO("Request: r = %+7.2f, p = %+7.2f, w = %+7.2f",
             _cmd_ang[0], _cmd_ang[1], _cmd_ang[2]);

    return true;
}

//*****************************************************************************
//cmdDuCallback: call back function to genrate a command du for motor
//****************************************************************************/
bool GroundStation::cmdDuCallback(testbed_gs::cmd_du::Request  &req,
                                  testbed_gs::cmd_du::Response &res){

    // apply the saturation values on the requested du vales ------------------
    res.du.linear.z  = sat(req.du.linear.z , _z_du_max, _z_du_min);
    res.du.angular.x = sat(req.du.angular.x, _r_du_max, _r_du_min);
    res.du.angular.y = sat(req.du.angular.y, _p_du_max, _p_du_min);
    res.du.angular.z = sat(req.du.angular.z, _y_du_max, _y_du_min);

    // Prepare and publish msg ------------------------------------------------
    _msg_du.header.stamp = ros::Time::now();
    _msg_du.header.seq++;
    _msg_du.twist.linear.z = res.du.linear.z;
    _msg_du.twist.angular.x = res.du.angular.x;
    _msg_du.twist.angular.y = res.du.angular.y;
    _msg_du.twist.angular.z = res.du.angular.z;
    _pub_du.publish(_msg_du);

    // announce the sent values -----------------------------------------------
    ROS_INFO("Request: r = %+7.2f, p = %+7.2f, w = %+7.2f, z = %+7.2f\n",
             _msg_du.header.seq,
             res.du.angular.x,
             res.du.angular.y,
             res.du.angular.z,
             res.du.linear.z);

    return true;
}
//*****************************************************************************
//cmdAngle: publish ang command message
//****************************************************************************/
void GroundStation::cmdAnglePuplish(float r, float p, float w){

    // apply the saturation values on the requested du vales ------------------
    _cmd_ang[0] = sat(r, _r_ang_max, _r_ang_min);
    _cmd_ang[1] = sat(p, _p_ang_max, _p_ang_min);
    _cmd_ang[2] = sat(w, _y_ang_max, _y_ang_min);

    // Prepare and publish msg ------------------------------------------------
    _msg_ang.header.stamp = ros::Time::now();
    _msg_ang.header.seq++;
    _msg_ang.vector.x = _cmd_ang[0];
    _msg_ang.vector.y = _cmd_ang[1];
    _msg_ang.vector.z = _cmd_ang[2];
    _pub_ang.publish(_msg_ang);

}

/******************************************************************************
initializeParams: initialize parameter using rosparm package
******************************************************************************/
void GroundStation::initializeParams(){

    // Get du max min values --------------------------------------------------
    std::vector<double> du;
    if (_nh.getParam("testbed/du_command/thrust", du)){
        _z_du_min = du[0];
        _z_du_max = du[1];
    }
    else {
        _z_du_min = 0.0;
        _z_du_max = 2.0;
    }
    if (_nh.getParam("testbed/du_command/roll", du)){
        _r_du_min = du[0];
        _r_du_max = du[1];
    }
    else {
        _r_du_min = -0.2;
        _r_du_max = +0.2;
    }
    if (_nh.getParam("testbed/du_command/pitch", du)){
        _p_du_min = du[0];
        _p_du_max = du[1];
    }
    else {
        _p_du_min = -0.2;
        _p_du_max = +0.2;
    }
    if (_nh.getParam("testbed/du_command/yaw", du)){
        _y_du_min = du[0];
        _y_du_max = du[1];
    }
    else {
        _y_du_min = -0.1;
        _y_du_max = +0.1;
    }

    // Get du max min values --------------------------------------------------
    std::vector<double> ang;
    if (_nh.getParam("testbed/angle_command/roll", ang)){
        _r_ang_min = ang[0];
        _r_ang_max = ang[1];
    }
    else {
        _r_ang_min = -0.15;
        _r_ang_max = +0.15;
    }
    if (_nh.getParam("testbed/angle_command/pitch", ang)){
        _p_ang_min = ang[0];
        _p_ang_max = ang[1];
    }
    else {
        _p_ang_min = -0.15;
        _p_ang_max = +0.15;
    }
    if (_nh.getParam("testbed/angle_command/yaw", ang)){
        _y_ang_min = ang[0];
        _y_ang_max = ang[1];
    }
    else {
        _y_ang_min = -3.0;
        _y_ang_max = +3.0;
    }

    // print result -----------------------------------------------------------
    ROS_INFO("Maximum\Minimum values for du command signals are set to:\n");
    ROS_INFO(" - Thrust = [%+6.2f - %+6.2f] \n",_z_du_min,_z_du_max);
    ROS_INFO(" - Roll   = [%+6.2f - %+6.2f] \n",_r_du_min,_r_du_max);
    ROS_INFO(" - Pitch  = [%+6.2f - %+6.2f] \n",_p_du_min,_p_du_max);
    ROS_INFO(" - Yaw    = [%+6.2f - %+6.2f] \n",_y_du_min,_y_du_max);

    ROS_INFO("Maximum\Minimum values for angle command signals are set to:\n");
    ROS_INFO(" - Roll   = [%+6.2f - %+6.2f] \n",_r_ang_min,_r_ang_max);
    ROS_INFO(" - Pitch  = [%+6.2f - %+6.2f] \n",_p_ang_min,_p_ang_max);
    ROS_INFO(" - Yaw    = [%+6.2f - %+6.2f] \n",_y_ang_min,_y_ang_max);

}
