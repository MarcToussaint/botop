BASE = ../rai

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

GL = 1
ROS = 0
DEPEND = Core Gui Kin KOMO Geo Algo RosCom Perception Control LGP Logic

include $(BASE)/makeutils/generic.mk
