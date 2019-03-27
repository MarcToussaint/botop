FROM ubuntu:16.04

RUN apt-get --yes update
RUN apt-get --yes install \
    git sudo make nano g++ x11-apps mesa-utils \
    terminator lsb-release

# add ROS repos
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get --yes update

# install full dependencies
RUN apt-get --yes install \
    g++ liblapack-dev libf2c2-dev gnuplot libjsoncpp-dev libx11-dev \
    libx11-dev libpng12-dev freeglut3-dev libglew-dev graphviz-dev \
    libann-dev \
    libassimp-dev libglew-dev libqhull-dev \
    ros-kinetic-desktop ros-kinetic-object-recognition-msgs \
    ros-kinetic-ar-track-alvar-msgs \
    libpcl-dev

RUN apt-get --yes install \
    ros-kinetic-vrpn-client-ros

WORKDIR /root

RUN echo 'source ~/bashrc' >> ~/.bashrc