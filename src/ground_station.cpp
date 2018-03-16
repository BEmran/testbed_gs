#include <testbed_gs/groundstation.h>

//*****************************************************************************
//main function:
//****************************************************************************/
int main(int argc, char** argv){

    // Initilaize ros node ----------------------------------------------------
    std::string name = "ground_station";                // define ros node name
    ros::init(argc, argv, name);                        // inialize ros node
    ros::NodeHandle nh;                                 // define ros handle
    GroundStation gs (nh,name);                         // define RosNode object
    ros::Rate loop_rate(50);                            // define ros frequency

    // Main Loop --------------------------------------------------------------
    ROS_INFO("Ground station is ready\n");
    while(ros::ok()){

        // publish angle message so they will have a constant frequency message
        gs.cmdAnglePuplish(gs._cmd_ang[0], gs._cmd_ang[1], gs._cmd_ang[2]);

        // ros method to keep sampling time
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
