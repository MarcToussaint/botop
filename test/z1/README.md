## Instructions:

https://github.com/unitreerobotics/z1_sdk
https://support.unitree.com/home/en/Z1_developer



* BEFORE POWER: align joints to zero

## Network

* test: `ping 192.168.123.110`

* In network settings:
  * add a new wired connection
  * identity 'z1 connection
  * ipv4: 192.168.123.235  255.255.255.0  192.168.123.1
  * select that network; ping should now work


## First time use
* in the z1_controller build:
  * 

## Use
* start 
cd ~/git/z1_controller/build/
./z1_ctrl

* then start your program
network setup:

https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html

IPv4 setting

IP address: 192.168.1.10
Subnet mask 255.255.255.0
Gateway: --empty--
DNS: 8.8.8.8

Then ping test!

Maybe NOT:
sudo ip link set enx0c3796dd0b0a up
sudo ip addr add 192.168.1.1/24 dev enx0c3796dd0b0a


# version!

git checkout v1.10.0
compile demos does not work!!
compile only lib
install path ~/.local
make install

# in botop/test/20-trossen
minimal main.cpp
LIBS += -ltrossen_arm
to link (with lib located in ~/.local/lib/libtrossen_arm.a )

# coding
ip of leader: .3!
