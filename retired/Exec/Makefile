BASE = ../../rai
BASE2 = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

CPATHS	+= $(HOME)/spartan/build/install/include

DEPEND = Sim Core Kin Gui Geo KOMO Optim RosCom Algo

ROS = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/_make/generic.mk
