import rospy
import roslib
roslib.load_manifest('spiri_go')
import actionlib
# Load specific service and action server definitions
from spiri_go.msg import TakeoffAction, TakeoffGoal
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
            raise spiri_except.SpiriGoConnectionError

    def armAndTakeoff(self, height=4):
        # Connection to the server
        client = self.getActionClient('spiri_take_off', TakeoffAction)
        goal = TakeoffGoal()
        goal.height = height
        print "Sending takoff command"
        client.send_goal(goal)
        client.wait_for_result()
