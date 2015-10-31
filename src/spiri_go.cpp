#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "std_msgs/String.h"
#include "spiri_go/spiri_go.h"
#include <sys/stat.h>
#include <string>

using namespace std;

go::go()
{
    ROS_INFO("Constructing Go");

    // Pubs the position of the copter centered on the take off location
    string local_ = nh.resolveName("/mavros/local_position/local");
    local = nh.subscribe(local_, 1, &go::localSubCb, this);

    // Pubs state info on the copter
    string state_ = nh.resolveName("/mavros/state");
    state = nh.subscribe(state_, 1, &go::stateSubCb, this);

    // control the velocity of the copter
    string vel_ = nh.resolveName("/mavros/setpoint_velocity/cmd_vel");
    vel = nh.advertise<geometry_msgs::TwistStamped>(vel_,1);

    // set the mode (mostly just to guided)
    string set_mode_ = nh.resolveName("/mavros/set_mode");
    set_mode = nh.serviceClient<mavros_msgs::SetMode>(set_mode_);

    // service for arming the copter
    string arm_vehicle_ = nh.resolveName("/mavros/cmd/arming");
    arm_vehicle = nh.serviceClient<mavros_msgs::CommandBool>(arm_vehicle_);

    // tell the copter to take off autonomously
    string takeoff_ = nh.resolveName("/mavros/cmd/takeoff");
    takeoff = nh.serviceClient<mavros_msgs::CommandTOL>(takeoff_);

    // MAVLink commander for yaw because setAttitude doesn't work in APM
    string mavlink_cmd_srv_ = nh.resolveName("/mavros/cmd/command");
    mavlink_cmd_srv = nh.serviceClient<mavros_msgs::CommandLong>(mavlink_cmd_srv_);

    // Some variables
    armed = false;
    guided = false;
    taking_off = false;
    flying = false;
    mode = "";

    // location z has to start here so you don't set flying too soon
    location.position.z = 0;

    ROS_INFO("Constructed Go");

}

go::~go()
{

}

/* ----- callback functions ----- */


bool go::isArmed(){
	return armed;
}

bool go::isControllable(){
	return guided;
}

std::string go::getMode(){
	return mode;
}

void go::localSubCb(const geometry_msgs::PoseStamped localPtr)
{
    location = localPtr.pose;

    //lastImudataReceived = *imudataPtr;
    ROS_INFO("Position: %f %f %f",
	location.position.x,
	location.position.y,
	location.position.z
    );
}

void go::stateSubCb(const mavros_msgs::State statePtr)
{
    armed = statePtr.armed;
    guided = statePtr.guided;
    mode = statePtr.mode;
}

/* ----- end callback functions ----- */


/* ----- getter functions based on callback ----- */

geometry_msgs::Point go::getLocalPosition()
{
    return location.position;
}

geometry_msgs::Quaternion go::getOrientation()
{
    return location.orientation;
}

//Eigen::Vector3d go::getAttitude()
//{
//    Eigen::Vector3d rpy;
//
//    tf::Quaternion orientationQ;
//    tf::quaternionMsgToTF(go::getOrientation(), orientationQ);
//    tf::Matrix3x3(orientationQ).getRPY(rpy.data()[0], rpy.data()[1], rpy.data()[2]);
//
//    return rpy;
//}


/* ----- end getter functions ----- */

void go::arm()
{
	mavros_msgs::CommandBool armCmd;

	armCmd.request.value = true;

    if(arm_vehicle.call(armCmd)){
        ROS_INFO("Vehicle armed");
        armed = true;
    }else{
        ROS_INFO("Failed to arm");
    }


}

void go::setGuided()
{
    mavros_msgs::SetMode modeCmd;

    modeCmd.request.base_mode = 0;
    modeCmd.request.custom_mode = "GUIDED";

    if(set_mode.call(modeCmd)){
        ROS_INFO("Set Guided Mode.");
        guided = true;
    }else{
        ROS_INFO("Failed to set to Guided Mode. Currently in %s mode", mode.c_str());
    }
}

void go::takeOff(float targetAlt)
{
    // try to take off
    mavros_msgs::CommandTOL to_cmd;

    to_cmd.request.altitude = 10;

    if(takeoff.call(to_cmd)){
        ROS_INFO("Taking off");
        taking_off = true;
    }else{
        ROS_INFO("Failed to take off");
    }
}


void go::armAndTakeOff(float targetAlt)
{
    // make arm and takeoff here to 2.1 m by default
}

void go::conditionYaw(float targetYaw, float targetYawRate)
{
    mavros_msgs::CommandLong yawCmd;

    yawCmd.request.command = 155;           // MAVLink command ID for MAV_CMD_CONDITION_YAW
    yawCmd.request.confirmation = 0;        // 0 is default for confirmation
    yawCmd.request.param1 = targetYaw;      // target heading/yaw in degrees from north (0 to 360)
    yawCmd.request.param2 = targetYawRate;  // target yaw rate in deg/s
    yawCmd.request.param3 = 1;              // direction; -1 ccw, 1 cw TODO: make this automatic
    yawCmd.request.param4 = 0;              // relative offset 1, absolute angle 0

    if(mavlink_cmd_srv.call(yawCmd)){
        ROS_INFO("Controlling yaw");
    }else{
        ROS_INFO("Condition yaw command rejected");
    }
}

void go::setENUVelocity(double eastwardVelocity, double northwardVelocity)
{
    geometry_msgs::TwistStamped control_msg;

    control_msg.twist.linear.x = eastwardVelocity;
    control_msg.twist.linear.y = northwardVelocity;

    vel.publish(control_msg);    
}

void go::Loop()
{
    ros::Rate pub_rate(10);

    while (nh.ok())
    {
        ros::spinOnce();

    	// Don't try to set to guided until armed
    	if(armed){
    	    if(not guided){
        		ROS_INFO("Setting mode to guided");
        		go::setGuided();
    	    }
    	}else{
    		go::arm();
    	    ROS_INFO("Attempting to arm");
    	}

        // get flying with the following
        if(not taking_off and guided and armed){
            go::takeOff(10);
    	}

    	if(not flying and taking_off){
    	    if(location.position.z >= 9.5){
        		// set to flying before reaching 10 to handle undershoot
        		flying = true;
                taking_off = false;
    	    }
    	}

    	// fly to the north
    	if(flying){
    	    go::setENUVelocity(5, 5);
    	}

        // --------------  sleep until rate is hit. ---------------
        pub_rate.sleep();
    }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "spiri_go");

  go go_thing;

  go_thing.Loop();

  return 0;
}
