import unittest
from spiripy import api, spiri_except
import os, subprocess
# Load specific service and action server definitions
from spiri_go.msg import *
from spiri_go.srv import *

class TestSpiriApi(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        # print "setting the class up."
        # roslaunch spiri_go
        self.ros = subprocess.Popen(
            ["roslaunch", "spiri_go", "sitl.launch"],
            preexec_fn=os.setsid,
            #stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.PIPE
        )

    @classmethod
    def tearDownClass(self):
        #os.kill(self.ros.pid, 15)
        os.killpg(os.getpgid(self.ros.pid), 15)
        os.waitpid(self.ros.pid, os.WUNTRACED)

    def setUp(self):
        self.spiri_go = api.SpiriGo()

    # TESTS OF ACTIONS
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

    # TESTS OF SERVICES
    def test_bogus_service_name(self):
        with self.assertRaises(spiri_except.SpiriGoConnectionError):
            self.spiri_go.getServiceClient("bogus_service_name", LastState)

    # test getting the state
    def test_get_state(self):
        #service_name = 'spiri_state'
        #state_sc = self.spiri_go.getServiceClient(service_name, LastState)
        state = self.spiri_go.getState()
        # check that the state is in it's expected starting configuration
        self.assertFalse(state.connected)

    def test_get_local_position(self):
        service_name = 'spiri_local_position'
        state_sc = self.spiri_go.getServiceClient(service_name, LocalPosition)

    # Test the wait_for_fcu function's timeout feature
    def test_wait_for_fcu_timeout(self):
        with self.assertRaises(spiri_except.SpiriGoConnectionError):
            # should throw an error since there's no sim to connect to
            self.spiri_go.wait_for_fcu(1)


# The setup and teardown of these is very slow (almost a minute worst case),
# therefore it is recomended to use large monolithic testcases to reduce
# overhead. Generally though, do not test something here if it can be
# tested just as well without the simulator. Note that mavros is on for
# all test suits. This is the only one that uses the sitl sim though.
# class TestSpiriApiWithSim(unittest.TestCase):
#     def setUp(self):
#         # launch the simulator
#         home = os.path.expanduser("~")
#         ardu_path = os.path.join(home, "ardupilot", "ArduCopter")
#         self.sim = p=subprocess.Popen(
#             ["sim_vehicle.sh", "--daemon"],
#             cwd=ardu_path,
#             preexec_fn=os.setsid,
#             #stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE
#         )
#         # launch ROS
#         self.ros = subprocess.Popen(
#             ["roslaunch", "spiri_go", "sitl.launch"],
#             preexec_fn=os.setsid,
#             #stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE
#         )
#         self.spiri_go = api.SpiriGo()
#
#     def tearDown(self):
#         # kill ros
#         os.killpg(os.getpgid(self.ros.pid), 15)
#         # kill the simulator
#         os.killpg(os.getpgid(self.sim.pid), 15)
#         # wait for the processes to end
#         os.waitpid(self.ros.pid, os.WUNTRACED)
#         os.waitpid(self.sim.pid, os.WUNTRACED)
#
#     def test(self):
#         try:
#             print 'waiting 20 seconds for ROS'
#             self.spiri_go.wait_for_ros(20)
#         except:
#             # print an explanation, then reraise the error
#             print "Can't connect to ROS. Check your system setup."
#             raise
#
#         try:
#             print 'waiting 20 seconds for the flight controller'
#             self.spiri_go.wait_for_fcu(20)
#         except:
#             # print an explanation, then reraise the error
#             print "Can't connect to simulator. Check your system setup."
#             raise
#
#         print 'Test Takeoff'
#         to_height = 5
#         self.spiri_go.armAndTakeoff(to_height)
#         # This blocks until takeoff is over
#         local = self.spiri_go.getLocalPosition()
#         # ensure that you are close enough to the takeoff goal
#         self.assertGreater(local.z, to_height*0.96)
#         self.assertLess(local.z, to_height*1.04)
#
#         # Test Landing
#         self.spiri_go.landHere()
#         # block while waiting to land
#         local = self.spiri_go.getLocalPosition()
#         # a assume it will be at least this low after it's landed
#         self.assertLess(local.z, 0.1)


if __name__ == "__main__":
    unittest.main()
