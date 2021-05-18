# LIS robot operation repo

## Installation

This assumes a standard Ubuntu 18.04 or 20.04 machine.

* The following assumes $HOME/git as your git path, and $HOME/opt
to install 3rd-party libs -- please stick to this (no system-wide installs)

* Install [libfranka](https://github.com/frankaemika/libfranka)
```
sudo apt update
sudo apt install --yes build-essential cmake git libpoco-dev libeigen3-dev

mkdir -p $HOME/git
cd $HOME/git
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
git checkout 0.7.1

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make franka -j $(command nproc)
```

* Clone and compile this repo:
```
mkdir -p $HOME/git
cd $HOME/git
git clone --recursive git@git.tu-berlin.de:lis/botop.git
#git clone --recursive git@github.com:MarcToussaint/botop.git
cd botop

make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
# If this fails, please try `make -j1 printUbuntuAll` to print all packages and install manually

mkdir build
cd build
cmake ..
make -j $(command nproc)
```

* Add binaries to your $PATH; or add symbolic links to your user bin 
```
export PATH="$HOME/git/botop/build:$PATH"
OR SOMETHING LIKE:
ln -s $HOME/botop/build/bot $HOME/bin/
ln -s $HOME/botop/build/kinEdit $HOME/bin/
```

* Test the things in test/
```
make tests
test/01-.../x.exe
```


## Panda robot operation

* The user needs to be part of the `realtime` unix group:
```
sudo usermod -a -G realtime <username>
```
* Turn on the power switch at the control box
* Open the panda web interface at `https://172.16.0.2/desk/`. You'll need to log in with user `mti` and passwd `mti-engage`
* Unlock the joints
* **ALWAYS 2 PEOPLE ARE REQUIRED! ONE TO HOLD THE EMERGENCY STOP**
* Perform a series of tests
  * `bot -openclose -home -loop -speed 2`


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

  * `bot -openclose`
  * `bot -home`
  * `bot -loop -speed 2`
  * `bot -float`
  * `bot -hold`



