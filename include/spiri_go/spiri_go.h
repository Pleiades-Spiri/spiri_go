#pragma once

#ifndef __SPIRIGO_H
#define __SPIRIGO_H
 

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "mavros_msgs/State.h"
/*#include "um_pixhawk/filter_state.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include "um_pixhawk/StateestimationParamsConfig.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>*/



struct go
{
private:
    // subscribers
    // the local position of the copter
    ros::Subscriber local;
    // the state of the copter
    ros::Subscriber state;

    // publishers
    // sets the velocity
    ros::Publisher vel;
    ros::Publisher rc_out;

    // services
    // a service to set the mode
    ros::ServiceClient set_mode;
    // tell the copter to take off
    ros::ServiceClient takeoff;

    ros::NodeHandle nh_;

    // save last imu received for forwarding...
    sensor_msgs::Imu lastLocalReceived;

    // ROS message callbacks
    void localSubCb(const geometry_msgs::PoseStamped imudataPtr);
    void stateSubCb(const mavros_msgs::State);

    // state variables
    bool armed;
    bool guided;
    bool taking_off; // has taken off
    bool flying; // has finished taking off
    std::string mode;

    // location
    geometry_msgs::Pose location;

public:

    go();
    ~go();


    // main pose-estimation loop
    void Loop();

};
#endif /* __SPIRIGO_H */
