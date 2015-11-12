## SpiriGo

This is a ROS catkin package of Spiri commands. Requires `mavros` to work.

### Running in the Simulator:

To run the simulator, clone a copy of the ardupilot project from [Ardupilot][]. Follow the directions at [Ardupilot SITL][] to set up the simulator. To test Spiri ROS package, start the simulator with:

```
sim_vehicle.sh --map --console --aircraft test
```

next launch the sitl node for spiri:

```
roslaunch spiri_go sitl.launch
```

If you are using a mission planner such as apm planner 2, it should connect on udp port 14551.

[ardupilot]: <href="https://github.com/diydrones/ardupilot>
[ardupilot sitl]: <http://dev.ardupilot.com/wiki/sitl-simulator-software-in-the-loop/>

### Running on the Jetson TK1 Companion Computer

Connect the Jetson to the Pixhawk via serial connection (detailed instructions, including the pin numbers and schematics, coming soon), and then run

```
roslaunch spiri_go jetson.launch
```

If you see 

```
[ INFO] [1447370932.205419674]: CON: Got HEARTBEAT, connected.
```

It means you've succeeded. Note the baud rate is set to `921600` because that's the highest allowed by `mavros`. We're looking into increasing this to `1500000` maybe if we really need to.
