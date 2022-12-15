BASE = ../../rai
BASE2 = ../../src
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

EIGEN = 1

DEPEND = Core 

CXX_FLAGS = -DENABLE_OPTITRACK

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/makeutils/generic.mk
