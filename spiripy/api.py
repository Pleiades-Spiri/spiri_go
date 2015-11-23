import rospy
import roslib
import time
roslib.load_manifest('spiri_go')
import actionlib
# Load specific service and action server definitions
from spiri_go.msg import *  # imports actions too
from spiri_go.srv import *
# Import the exceptions defined here
from spiripy import spiri_except

class SpiriGo:
    def __init__(self):
        rospy.init_node('spiri_go_api', anonymous=True)

    # Gets the client and ensures that the server is running
    # name: a string, the server's name (eg. 'spiri_take_off')
    # action: an object, the type of server (eg. TakeoffAction)
    def getActionClient(self, name, action):
        client = actionlib.SimpleActionClient(name, action)
        server_present = client.wait_for_server(rospy.Duration(1))
        if server_present:
            return client
        else:
            raise spiri_except.SpiriGoConnectionError("Could not create action client " + name)

    def getServiceClient(self, name, service):
        # ensure that the service exists
        try:
            rospy.wait_for_service(name, 1)
        except:
            raise spiri_except.SpiriGoConnectionError("Could not create service client " + name)
        return rospy.ServiceProxy(name, service)

    def getState(self):
        state_sc = self.getServiceClient('spiri_state', LastState)
        return state_sc()

    # waits for the ros interfaces to come online
    def wait_for_ros(self, timeout=-1):
        try:
            rospy.wait_for_service('spiri_state', timeout)
        except:
            raise spiri_except.SpiriGoConnectionError("Timed out connecting to ROS")

    # waits for mavros to connect to the flight controller
    def wait_for_fcu(self, timeout=-1):
        start_time = time.time()
        state = self.getState()

        while not state.connected:
            time.sleep(0.5)
            state = self.getState()
            duration = time.time() - start_time
            # print "waiting for fuc: " + str(duration) + "/" + str(timeout)
            # check if this should time out
            if timeout > 0 and duration > timeout:
                raise spiri_except.SpiriGoConnectionError("Timed out connecting to the FCU")

    def armAndTakeoff(self, height=4):
        # Connection to the server
        client = self.getActionClient('spiri_take_off', TakeoffAction)
        goal = TakeoffGoal()
        goal.height = height
        print "Sending takoff command"
        client.send_goal(goal)
        client.wait_for_result()

    def landHere(self):
        client = self.getActionClient('spiri_land_here', LandHereAction)
        goal = LandHereGoal()
        goal.height = 0
        print "Sending land here command"
        client.send_goal(goal)
        client.wait_for_result()

    def getLocalPosition(self):
        localPos_sc = self.getServiceClient('spiri_local_position', LocalPosition)
        return localPos_sc()
