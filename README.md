# LIS robot operation repo

## Installation

This assumes a standard Ubuntu 18.04 or 20.04 machine.

* The following assumes $HOME/git as your git path, and $HOME/opt to
  install 3rd-party libs -- please stick to this (no system-wide
  installs). Setup basics:
```
sudo apt update
sudo apt install --yes build-essential cmake git libpoco-dev libeigen3-dev

mkdir -p $HOME/git
mkdir -p $HOME/opt/lib
mkdir -p $HOME/opt/include
```

* Add your ssh key to github: Use `ssh-keygen` and `cat ~/.ssh/id_rsa.pub`

* Install [libfranka](https://github.com/frankaemika/libfranka)
```
cd $HOME/git
git clone --recursive https://github.com/frankaemika/libfranka

cd libfranka
#git checkout 0.7.1 --recurse-submodules ##FOR THE OLD ARM!!!
git checkout 0.8.0 --recurse-submodules
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$HOME/opt \-DCMAKE_BUILD_TYPE=Release ..

nice -19 make franka -j $(command nproc)

cp libfranka* $HOME/opt/lib
cp -r ../include/franka $HOME/opt/include
```

* OPTIONAL (if enabled with ccmake ..) install [librealsense](https://github.com/IntelRealSense/librealsense)
```
sudo apt install --yes libusb-1.0-0-dev libglfw3-dev libgtk-3-dev

cd $HOME/git
git clone --recursive https://github.com/IntelRealSense/librealsense.git

cd librealsense
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$HOME/opt -DCMAKE_BUILD_TYPE=Release ..

nice -19 make realsense2 -j $(command nproc)

make install
```

* Install python and pybind
```
sudo apt-get install python3 python3-dev python3-numpy python3-pip python3-distutils
echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc   #add this to your .bashrc, if not done already
pip3 install --user jupyter nbconvert matplotlib pybind11
```

* Clone and compile this repo:
```
cd $HOME/git
git clone --recursive git@github.com:MarcToussaint/botop.git

cd botop
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt

mkdir build
cd build
cmake ..

nice -19 make -j $(command nproc)
```

* Add binaries to your $PATH; or add symbolic links to your user bin 
```
echo 'export PATH="$HOME/git/botop/build:$PATH"' >> ~/.bashrc
OR SOMETHING LIKE:
ln -s $HOME/git/botop/build/bot $HOME/bin/
ln -s $HOME/git/botop/build/kinEdit $HOME/bin/
```

* Test the things in test/
```
bot -sim -loop
```


## Panda robot operation

* The user needs to be part of the `realtime` unix group:
```
sudo usermod -a -G realtime <username>
```
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
```
BASE = ../../botop/rai

include $(BASE)/build/generic.mk
```
* If you have more cpp-files, you can add `OBJS = main.o code.o etc.o`
* Otherwise, use your own build system and just link with librai.so


## cmd line tool `bot`

* `bot -open -close`
* `bot -home`
* `bot -loop -speed 2`
* `bot -float`
* `bot -hold`
* `bot -damp`



