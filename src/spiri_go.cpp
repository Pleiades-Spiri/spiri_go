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
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandBool.h"
#include "std_msgs/String.h"
#include "spiri_go.h"
#include <sys/stat.h>
#include <string>

SpiriGo::SpiriGo():
	takeoff_as(nh, "spiri_take_off", boost::bind(&SpiriGo::armAndTakeOff, this, _1), false),
    land_here_as(nh, "spiri_land_here", boost::bind(&SpiriGo::landHere, this, _1), false)
{
    ROS_INFO("Constructing Go");
    
    // Pubs the position of the copter centered on the take off location
    string local_ = nh.resolveName("/mavros/local_position/local");
    local = nh.subscribe(local_, 1, &SpiriGo::localSubCb, this);
    
    // Pubs state info on the copter
    string state_ = nh.resolveName("/mavros/state");
    state = nh.subscribe(state_, 1, &SpiriGo::stateSubCb, this);
    
    // control the velocity of the copter
    string vel_ = nh.resolveName("/mavros/setpoint_velocity/cmd_vel");
    vel = nh.advertise<geometry_msgs::TwistStamped>(vel_,1);
    
    // set the mode (mostly just to guided)
    string set_mode_ = nh.resolveName("/mavros/set_mode");
    set_mode = nh.serviceClient<mavros_msgs::SetMode>(set_mode_);
    
    // Arming
    string arm_ = nh.resolveName("/mavros/cmd/arming");
    arm = nh.serviceClient<mavros_msgs::CommandBool>(arm_);
    
    // tell the copter to take off autonomously
    string takeoff_ = nh.resolveName("/mavros/cmd/takeoff");
    takeoff = nh.serviceClient<mavros_msgs::CommandTOL>(takeoff_);
    
    // MAVLink commander for yaw because setAttitude doesn't work in APM
    string mavlink_cmd_srv_ = nh.resolveName("/mavros/cmd/command");
    mavlink_cmd_srv = nh.serviceClient<mavros_msgs::CommandLong>(mavlink_cmd_srv_);
    
    
    // Create the services and actions that other nodes can interact with spiri_go through
    
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

SpiriGo::~SpiriGo()
{
    
}

/* ----- callback functions ----- */

bool SpiriGo::isArmed(){
    return armed;
}

bool SpiriGo::isControllable(){
    return guided;
}

std::string SpiriGo::getMode(){
    return mode;
}

void SpiriGo::localSubCb(const geometry_msgs::PoseStamped localPtr)
{
    location = localPtr.pose;
    
    //lastImudataReceived = *imudataPtr;
    ROS_INFO("Position: %f %f %f",
             location.position.x,
             location.position.y,
             location.position.z
             );
}

void SpiriGo::stateSubCb(const mavros_msgs::State statePtr)
{
    armed = statePtr.armed;
    guided = statePtr.guided;
    mode = statePtr.mode;
}

/* ----- end callback functions ----- */


/* ----- getter functions based on callback ----- */

geometry_msgs::Point SpiriGo::getLocalPosition()
{
    return location.position;
}

geometry_msgs::Quaternion SpiriGo::getOrientation()
{
    return location.orientation;
}


SpiriAttitude SpiriGo::getAttitude()
{
    SpiriAttitude rpy(1, 2, 3);
    
    //    tf::Quaternion orientationQ;
    //    tf::quaternionMsgToTF(SpiriGo::getOrientation(), orientationQ);
    //    tf::Matrix3x3(orientationQ).getRPY(rpy.roll, rpy.pitch, rpy.yaw);
    
    return rpy;
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

/* ----- service-specific methods ----- */

void SpiriGo::setMode(const char* targetMode)
{
    mavros_msgs::SetMode modeCmd;
    
    modeCmd.request.base_mode = 0;
    modeCmd.request.custom_mode = targetMode;
    
    if(set_mode.call(modeCmd)){
        ROS_INFO("Set to %s Mode.", targetMode);
    }else{
        ROS_INFO("Failed to set to %s Mode. Currently in %s mode", targetMode, mode.c_str());
    }
}

void SpiriGo::setGuided()
{
    setMode("GUIDED");
}

void SpiriGo::setArmed()
{
    mavros_msgs::CommandBool set_armed;
    set_armed.request.value = true;
    
    if(arm.call(set_armed)){
        ROS_INFO("Set Armed.");
    }else{
        ROS_INFO("Failed to set to Armed.");
    }
}

void SpiriGo::takeOff(float targetAlt = 5)
{
    // try to take off
    mavros_msgs::CommandTOL to_cmd;
    
    to_cmd.request.altitude = targetAlt;
    
    if(takeoff.call(to_cmd)){
        ROS_INFO("Taking off");
        taking_off = true;
    }else{
        ROS_INFO("Failed to initiate take off");
    }
}

void SpiriGo::conditionYaw(float targetYaw, float targetYawRate)
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

void SpiriGo::setENUVelocity(double eastwardVelocity, double northwardVelocity)
{
    geometry_msgs::TwistStamped control_msg;
    
    control_msg.twist.linear.x = eastwardVelocity;
    control_msg.twist.linear.y = northwardVelocity;
    
    vel.publish(control_msg);
}


/* ----- action-specific methods - these will sleep until finished!!! ----- */

void SpiriGo::armAndTakeOff(const spiri_go::TakeoffGoalConstPtr& goal)
{
    if(!takeoff_as.isActive()||takeoff_as.isPreemptRequested()) return;
    ros::Rate takeoff_rate(5);
    bool success = true;
    
    // Arm
    while(not armed)
    {
        setArmed();
        takeoff_rate.sleep();
    }
    
    ROS_INFO("Spiri Armed");
    
    // set to guided mode
    while(not guided)
    {
        setGuided();
        takeoff_rate.sleep();
    }
    
    ROS_INFO("Spiri Guided.");
    
    // take off
    taking_off = false;
    while(not taking_off)
    {
        takeOff(goal->height);
        takeoff_rate.sleep();
    }
    while(location.position.z < goal->height*0.96)
    {
        // Only wait to reach 96% of the target altitude to handle undershoot
        takeoff_rate.sleep();
    }
    flying = true;
    
    if(ros::ok()){
        takeoff_as.setSucceeded();
    } else {
        takeoff_as.setAborted();
    }
}

// action for Spiri to land in place 
void SpiriGo::landHere(const spiri_go::LandHereGoalConstPtr& goal)
{
    if(!land_here_as.isActive()||land_here_as.isPreemptRequested()) return;
    ros::Rate land_rate(5);
    bool success = true;    

    ROS_INFO("Attempting to land in place");

    while(mode != "LAND")
    {
        setMode("LAND");
        land_rate.sleep();
    }

    while(location.position.z > 0.05)
    {
        // Wait for the robot to think it's 5cm off the ground
        land_rate.sleep();
    }

    if(ros::ok()){
        land_here_as.setSucceeded();
    } else {
        land_here_as.setAborted();
    }    

}


void SpiriGo::Loop()
{
    ros::Rate pub_rate(10);
    
    //ActionTestServer server(ros::this_node::getName());
    
    ros::spin();
    /*while (nh.ok())
     {
     ros::spinOnce();
     
    	// Don't try to set to guided until armed
    	if(armed){
     if(not guided){
     ROS_INFO("Setting mode to guided");
     SpiriGo::setGuided();
     }
    	}else{
     <<<<<<< HEAD
     =======
     go::arm();
     ROS_INFO("Attempting to arm");
     >>>>>>> c800e528af8bb875e6b626ee134299c14f7e2caa
    	}
     
     // get flying with the following
     if(not taking_off and guided and armed){
     SpiriGo::takeOff(10);
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
     SpiriGo::setENUVelocity(5, 5);
    	}
     
     // --------------  sleep until rate is hit. ---------------
     pub_rate.sleep();
     }*/
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "spiri_go");
    
    SpiriGo go_thing;
    
    go_thing.takeoff_as.start();
    go_thing.land_here_as.start();

    go_thing.Loop();
    
    return 0;
}
