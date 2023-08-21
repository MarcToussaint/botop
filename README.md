# LIS robot operation repo

This repo is how we, in the [Learning & Intelligent Systems Lab](https://argmin.lis.tu-berlin.de/), operate our robots. Perhaps the most significant aspect is that we decided for a spline-based convention to command motion (in class BotOp), which turned out the cleanest interface to our pose and motion optimization methods, and now also MPC methods.

This repo is really our working environment, is not clean, contains much outdated code, and might be too specific for others to use, but we're happy to share. When run without hardware, the BotOp interface can  emulate/simulate any robot (e.g., found in rai-robotModels).

The [rai](https://github.com/MarcToussaint/rai) submodule contains our actual algorithmic code base (data structures, optimizers, kinematics, KOMO, LGP, etc).

No, we don't use ROS at all anymore.

## Installation

This assumes a standard Ubuntu, tested on 18.04, 20.04, and latest docker. (When compiling in a docker, perhaps `export APTGETYES="--yes"; alias sudo=""` )

* The following assumes $HOME/git as your git path, and $HOME/opt to
  install 3rd-party libs -- please stick to this (no system-wide
  installs). Setup basics:

      sudo apt update
      sudo apt install --yes build-essential clang cmake curl git libpoco-dev libeigen3-dev libccd-dev libboost-system-dev portaudio19-dev libglu1-mesa-dev
      mkdir -p $HOME/git $HOME/opt

* Install several external libraries from source. Perhaps first choose # kernels for compile:

      export MAKEFLAGS="-j3"

   * [libfranka](https://github.com/frankaemika/libfranka) (needs 0.7.1 for old pandas, 0.9.2 or 0.10.0 for new!)
   
         cd $HOME/git; git clone --single-branch -b 0.7.1 --recurse-submodules https://github.com/frankaemika/libfranka
         cd $HOME/git/libfranka; mkdir build; cd build; cmake -DCMAKE_INSTALL_PREFIX=$HOME/opt -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF ..; make install

   * PhysX (physical simulator, enabled by default, could be disabled with ccmake)
   
         cd $HOME/git; git clone --single-branch -b release/104.1 https://github.com/NVIDIA-Omniverse/PhysX.git
         cd $HOME/git/PhysX/physx; ./generate_projects.sh linux; cd compiler/linux-release/; cmake ../../compiler/public -DPX_BUILDPVDRUNTIME=OFF -DPX_BUILDSNIPPETS=OFF -DCMAKE_INSTALL_PREFIX=$HOME/opt; make install

   * FCL 0.5 (version important)

         cd $HOME/git; git clone --single-branch -b fcl-0.5 https://github.com/flexible-collision-library/fcl.git
         cd $HOME/git/fcl; mkdir -p build; cd build; cmake -DCMAKE_INSTALL_PREFIX=$HOME/opt -DFCL_STATIC_LIBRARY=ON -DFCL_BUILD_TESTS=OFF ..; make install

   * [librealsense](https://github.com/IntelRealSense/librealsense) (enabled by default, could be disabled with ccmake)

         sudo apt install --yes libusb-1.0-0-dev libglfw3-dev libgtk-3-dev
         cd $HOME/git; git clone --recurse-submodules https://github.com/IntelRealSense/librealsense.git
         cd $HOME/git/librealsense; mkdir build; cd build; cmake -DCMAKE_INSTALL_PREFIX=$HOME/opt -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF ..; make install

* Install python and pybind

      sudo apt install --yes python3 python3-dev python3-numpy python3-pip python3-distutils
      echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc   #add this to your .bashrc, if not done already
      source ~/.bashrc
      python3 -m pip install --user jupyter nbconvert matplotlib pybind11

* Clone and compile this repo:

      cd $HOME/git; git clone --recurse-submodules git@github.com:MarcToussaint/botop.git
      cd $HOME/git/botop; APTGETYES=1 make -C rai -j1 installUbuntuAll  # calls sudo apt-get install
      export PYTHONVERSION=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
      cd $HOME/git/botop; mkdir -p build; cd build; cmake -DUSE_BULLET=OFF -DPYBIND11_PYTHON_VERSION=$PYTHONVERSION ..; make

* Optionally add binaries to your $PATH; or add symbolic links to your user bin 

      echo 'export PATH="$HOME/git/botop/build:$PATH"' >> ~/.bashrc
      OR SOMETHING LIKE:
      mkdir -p $HOME/bin
      ln -s $HOME/git/botop/build/bot $HOME/bin/
      ln -s $HOME/git/botop/build/kinEdit $HOME/bin/

* Test the things in test/

      bot -sim -loop

* Hacky: If you need to, overwrite the python pip-wheel with the locally compiled lib:

      ln -f -s $HOME/git/botop/build/libry.cpython-??-x86_64-linux-gnu.so $HOME/.local/lib/python3.8/site-packages/robotic/libry.so



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



