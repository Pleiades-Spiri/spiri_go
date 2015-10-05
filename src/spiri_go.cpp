#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandTOL.h"
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
    string local_ = nh_.resolveName("/mavros/local_position/local");
    local = nh_.subscribe(local_, 1, &go::localSubCb, this);

    // Pubs state info on the copter
    string state_ = nh_.resolveName("/mavros/state");
    state = nh_.subscribe(state_, 1, &go::stateSubCb, this);

    // control the velocity of the copter
    string vel_ = nh_.resolveName("/mavros/setpoint_velocity/cmd_vel");
    vel = nh_.advertise<geometry_msgs::TwistStamped>(vel_,1);

    // set the mode (mostly just to guided)
    string set_mode_ = nh_.resolveName("/mavros/set_mode");
    set_mode = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_);

    // tell the copter to take off autonomously
    string takeoff_ = nh_.resolveName("/mavros/cmd/takeoff");
    takeoff = nh_.serviceClient<mavros_msgs::CommandTOL>(takeoff_);

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

void go::Loop()
{
    ros::Rate pub_rate(10);

    while (nh_.ok())
    {
        ros::spinOnce();

	// Don't try to set to guided until armed
	if(armed){
	    if(not guided){
		ROS_INFO("Setting mode to guided");
		mavros_msgs::SetMode sm;
		sm.request.base_mode = 0;
		sm.request.custom_mode = "GUIDED";
		if(set_mode.call(sm)){
		    ROS_INFO("Set Guided Mode.");
		    guided = true;
		}else{
		    ROS_INFO("Failed to set to Guided Mode. Currently in %s mode", mode.c_str());
		}
	    }
	}else{
	    ROS_INFO("Waiting to arm");
	}

        // get flying with the following
        if(not taking_off and guided and armed){
	    // try to take off
	    mavros_msgs::CommandTOL off_srv;
	    off_srv.request.altitude = 10;
	    ROS_INFO("Attempting take off");
	    if(takeoff.call(off_srv)){
		ROS_INFO("Taking off");
		taking_off = true;
	    }else{
		ROS_INFO("Failed to take off");
	    }
	}
	if(not flying and taking_off){
	    if(location.position.z >= 9.5){
		// set to flying before reaching 10 to handle undershoot
		flying = true;
	    }
	}

	// fly to the north
	if(flying){
	    geometry_msgs::TwistStamped control_msg;
		control_msg.twist.linear.x = 5;
		control_msg.twist.linear.y = 5;
	    vel.publish(control_msg);
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
