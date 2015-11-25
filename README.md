## SpiriGo

This is a ROS catkin package of Spiri commands. Requires `mavros` to work.

### Running in the simulator:

To run the simulator, clone a copy of the ardupilot project from [Ardupilot][]. Follow the directions at [Ardupilot SITL][] to set up the simulator. There is also a script, `initalization/apm_sim` that will do this set-up automatically. To test Spiri ROS package, start the simulator with:

```
sim_vehicle.sh --map --console --aircraft test
```

next launch the sitl node for spiri:

```
roslaunch spiri_go sitl.launch
```

If you are using a ground control station such as APM Planner 2, it should connect on udp port 14551.

[ardupilot]: <href="https://github.com/diydrones/ardupilot>
[ardupilot sitl]: <http://dev.ardupilot.com/wiki/sitl-simulator-software-in-the-loop/>

### Using the Python API

Python scripts can use the spiripy api by importing:

```
from spiripy import api
```

and creating a SpiriGo object:

```
spiri_go = api.SpiriGo()
```

These can be launched as normal python scripts, but the spiri_go, and mavros packages must be running in the background for it to control a quadcopter.

### Running on the Jetson TK1 companion computer

#### Connect the Jetson to the Pixhawk via serial/UART connection

1. Connect the Jetson's UART ports to an added voltage converter (1.8 V to 5V);

2. Connect the voltage converter to the Pixhawk;

3. Make sure that the serial connection is established by checking `/dev/ttyTHS0`;

4. Validate the Jetson to Flight controller communication using maximum baud rate (921600).

See [this diagram](https://drive.google.com/open?id=0BxXn6LyBxnG6b01mc1N5X2diVlU) for pinout details

#### Running the launch file

Use this command to launch SpiriGo with serial communication (make sure you `bash` session knows where it is): 

```
roslaunch spiri_go jetson.launch
```

If you see 

```
[ INFO] [1447370932.205419674]: CON: Got HEARTBEAT, connected.
```

It means you've succeeded. Note the baud rate is set to `921600` because that's the highest allowed by `mavros`. We're looking into increasing this to `1500000` maybe if we really need to.
