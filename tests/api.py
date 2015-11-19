import unittest
from spiripy import api, spiri_except
import subprocess
# Load specific service and action server definitions
from spiri_go.msg import TakeoffAction, TakeoffGoal

class TestSpiriApi(unittest.TestCase):
    def setUp(self):
        self.spiri_go = api.SpiriGo()

    # verify that client getter fails when it should
    def test_bogus_action_name(self):
        with self.assertRaises(spiri_except.SpiriGoConnectionError):
            self.spiri_go.getActionClient("bogus_action_name", TakeoffAction)

    # Use the client test as an example to base other's on
    # any comments thta apply to all may go only here
    def test_takeoff_client(self):
        action_name = "spiri_take_off"
        # Attempt to create the client
        self.spiri_go.getActionClient(action_name, TakeoffAction)
        # Test that the goal can be created and has the right parameters
        goal = TakeoffGoal()
        self.assertEquals(goal.height, 0)


#class TestSpiriApiWithSim(unittest.TestCase):

if __name__ == "__main__":
    # roslaunch spiri_go
    subprocess.Popen(["roslaunch", "spiri_go", "sitl.launch"],
        stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.PIPE
    )
    unittest.main()
