# SpiriGo

This is a ROS catkin package of Spiri commands. Requires `mavros` to work.

## Installation

### On the Jetson TK1

1. Get the [latest version of the install script](https://github.com/Pleiades-Spiri/spiri_go/blob/master/install-spirigo.sh).
2. Run the script without root.

```bash
$ wget https://raw.githubusercontent.com/Pleiades-Spiri/spiri_go/master/install-spirigo.sh
$ chmod +x install-spirigo.sh
$ less install-spirigo.sh
```

This will give you a chance to read the script. When you're ready, run the script.

```bash
$ ./install-spirigo.sh
```
It will prompt you for your password right away even though it's without `sudo`. 

### On Ubuntu desktop (for simulator)

To run the simulator, clone a copy of the ardupilot project from [Ardupilot][]. Follow the directions at [Ardupilot SITL][] to set up the simulator. There is also a script, `initalization/apm_sim` that will do this set-up automatically. 

This script is meant to be ran from a fresh Ubuntu 14.04 installation. Please read if carefully if you plan on installing on a system with ROS already installed.

```bash
$ wget https://raw.githubusercontent.com/Pleiades-Spiri/spiri_go/master/initialize/apm_sim
$ chmod +x apm_sim
$ ./apm_sim
```

## Usage 

### On the Jetson TK1 companion computer

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

### In the simulator

To test Spiri ROS package, start the simulator with:

```
sim_vehicle.sh --map --console --aircraft test
```

Next launch the sitl node for spiri:

```
roslaunch spiri_go sitl.launch
```

If you are using a ground control station such as APM Planner 2, it should connect on UDP port `14551`.

[ardupilot]: <href="https://github.com/diydrones/ardupilot>
[ardupilot sitl]: <http://dev.ardupilot.com/wiki/sitl-simulator-software-in-the-loop/>

### Python API

There is a python API to control Spiri, using the services and actions provided by the ROS library.

To install this on your system, run:

```bash
python setup.py install
```

from the root of this repository.

To use this in a python script, include the following line:

```python
from spiripy import api
spiri = api.SpiriGo()
```

`spiri` will then be a SpiriGo instance with the following methods:

```python
wait_for_ros([timeout])
wait_for_fcu([timeout])
get_state()
get_local_position()
takeoff([height])
land()
```

Your script can be launched as a normal python scripts, but `spiri_go`, and `mavros` packages must be running in the background for it to control a quadcopter.

## Generating documentation

TODO

## Develpers

### Running tests

To test the python API, run:

```
python tests/api.py
```

To test those parts that require the simulator running, use:

```
python tests/sim.py
```

Any new methods that are made MUST have a corresponding unit test, and if possible should have a corresponding unit test with the simulator. It is good practice to write the test before implementing the method.
