# LIS robot operation repo

This repo is how we, in the
[Learning & Intelligent Systems Lab](https://argmin.lis.tu-berlin.de/),
operate our robots. We decided for a spline-based convention to
command motion (in class BotOp), which turned out the cleanest
interface to our motion optimization methods, and now also MPC
methods.

This repo is really our working environment, is not clean, contains
much outdated code, and might be too specific for others to use, but
we're happy to share. When run without hardware, the BotOp interface
can emulate/simulate any robot (e.g., found in rai-robotModels).

See the [robotic python package](https://github.com/MarcToussaint/rai)
for a python wrapper.

The [rai](https://github.com/MarcToussaint/rai) module contains our
actual algorithmic code base (data structures, optimizers, kinematics,
KOMO, LGP, etc).

We don't use ROS at all anymore.

## Installation

This assumes a standard Ubuntu, tested on 18.04, 20.04, and latest docker. (When compiling in a docker, perhaps `export APTGETYES="--yes"; alias sudo=""` )

* The following assumes $HOME/git as your git path, and $HOME/.local to
  install 3rd-party libs -- please stick to this (no system-wide
  installs). The following installs basic ubuntu packages (used by external packages and botop):

      sudo apt update
	  sudo apt install --yes \
        g++ clang make cmake curl git wget \
        liblapack-dev libf2c2-dev libqhull-dev libeigen3-dev libann-dev libccd-dev \
        libjsoncpp-dev libyaml-cpp-dev libpoco-dev libboost-system-dev portaudio19-dev libusb-1.0-0-dev libhidapi-dev \
        libx11-dev libglu1-mesa-dev libglfw3-dev libglew-dev freeglut3-dev libpng-dev libassimp-dev
      mkdir -p $HOME/git $HOME/.local

* External libraries: You can skip librealsense and libfranka if you disable below. To standardize installations, I use a [basic install script](https://github.com/MarcToussaint/rai-extern/blob/main/install.sh) (have a look, if you have concerns what it does).

      export MAKEFLAGS="-j $(command nproc --ignore 2)"
	  wget https://github.com/MarcToussaint/rai-extern/raw/main/install.sh; chmod a+x install.sh
      ./install.sh fcl
      ./install.sh physx
      ./install.sh librealsense
      ./install.sh libfranka  ## for OLD frankas instead:   ./install.sh -v 0.7.1 libfranka

* You can skip this, if you disable pybind11 below.

      sudo apt install --yes python3-dev python3 python3-pip
      python3 -m pip install --user numpy pybind11 pybind11-stubgen
	  
	  # add the following to your .bashrc, if not done already
      echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc
      echo 'export PYTHONPATH="${PYTHONPATH}:$HOME/.local/lib"' >> ~/.bashrc

* Finally, clone and compile this repo:

      cd $HOME/git
	  git clone --recurse-submodules git@github.com:MarcToussaint/botop.git
      cd botop
      export PY_VERSION=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
      cmake -DUSE_BULLET=OFF -DUSE_OPENCV=OFF -DPY_VERSION=$PY_VERSION . -B build  #options: disable USE_LIBFRANKA USE_REALSENSE USE_PYBIND USE_PHYSX
	  make -C build/

* Optionally install lib, headers, and binaries to ~/.local

      make -C build/ install

* Test the things in test/

      bot -sim -loop


## Panda robot operation

* The user needs to be part of the `realtime` and `dialout` unix group:

      sudo usermod -a -G realtime <username>
      sudo usermod -a -G dialout <username>

You need to log out and back in (or even reboot) for this to take effect. Check with `groups` in a terminal.
* Turn on the power switch at the control box
* Open the panda web interface at `https://172.16.0.2/desk/`. Accept the security risk. You'll need to log in with user `mti` and passwd `mti-engage`
* Unlock the joints
* **ALWAYS 2 PEOPLE ARE REQUIRED! ONE TO HOLD THE EMERGENCY STOP**
* Perform a series of tests
  * `bot -close`
  * `bot -open`
  * `bot -home`
  * `bot -loop`


## Setting up your own coding environment/directory

* Use a separate repository for your own code
* Place the repository in $HOME/git (parallel to ~/git/botop)
* In a working directory with your `main.cpp` place the following `Makefile` - and compile

      BASE = ../../botop/rai
      
      include $(BASE)/build/generic.mk

* If you have more cpp-files, you can add `OBJS = main.o code.o etc.o`
* Otherwise, use your own build system and just link with librai.so


## cmd line tool `bot`

* `bot -open -close`
* `bot -home`
* `bot -loop -speed 2`
* `bot -float`
* `bot -hold`
* `bot -damp`



