BASE = ../../rai
BASE2 = ..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core Perception Franka RealSense FlatVision NewControl CameraCalibration RosCom opencv_reg
OPENCV = 1

PYBIND = 0


SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/makeutils/generic.mk
