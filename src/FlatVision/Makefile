BASE = ../../rai
BASE2 = ../../src
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core Gui Perception opencv_reg

OPENCV = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/makeutils/generic.mk
