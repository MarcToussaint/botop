TODOs:
* get the whole thing going (in simulation, as it was on your mac) on the iwaa computer
* implement the robotio.cpp

StartHere:
https://github.com/RobotLocomotion/KukaHardware
More on getting spartan running in docker:
https://github.com/RobotLocomotion/spartan/blob/master/setup/docker/README.md

https://github.com/RobotLocomotion/KukaHardware/blob/master/robot_operation_instructions.md

Possible needed changes in `docker_build.py`:
```
cmd += " --network=host"
cmd += " --no-cache"
```

changes in `docker_run.py`:
```
cmd += " -v ~/local:%(home_directory)s/local " % {'home_directory': home_directory}   # mount local path into docker

cmd += " -p 11311 " # expose ROS ports
cmd += " -p 3883 " # expose VRPN ports
cmd += "--network host"
```

when compiling in the docker, in ccmake:
```
BULLET3: ON
RLG: ON
TRI: OFF
SNOPT: OFF
ROS: ON
PERCEPTION: ON
```

addons in the docker:
```
#apt-get install qtcreator
cd ~/local/LGP-execution

apt-get install \
libglew-dev \
g++ liblapack-dev libf2c2-dev gnuplot libjsoncpp-dev libx11-dev \
libann-dev \
libassimp-dev libglew-dev libqhull-dev \
libglew-dev \
libx11-dev libpng12-dev freeglut3-dev libglew-dev graphviz-dev \
ros-kinetic-vrpn-client-ros emacs mesa-utils x11-apps iputils-ping less nmap net-tools
```
Alternatively, incorporate into `spartan.dockerfile`:
(put above in `install_dependencies_lgp.sh` at spartan/setup/docker)
```
COPY ./setup/docker/install_dependencies_lgp.sh /tmp/install_dependencies_lgp.sh
RUN yes "Y" | /tmp/install_dependencies_lgp.sh
```


saving and starting docker images
```
docker commit spartan spartan-after-compile-marc:2
setup/docker/docker_run.py --container spartan-after-compile-marc
```

Getting communication with OptiTrack:
```
In the MotiveTracker: Broadcast ON; use 'Local Interface' URL!; VRPN Streaming ON
vrpn_print_devices RigidBody01@128.30.27.150
roslaunch vrpn_client_ros sample.launch server:=128.30.27.150
```

launching things in the docker:
```
make catkin-projects/fast

use_ros
use_spartan
kuka_iiwa_procman &

apps/iiwa/runprocman.sh 

```


Help on `rai`: https://github.com/MarcToussaint/rai-maintenance/tree/master/help
