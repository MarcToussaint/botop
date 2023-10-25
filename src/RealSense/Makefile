BASE = ../../rai
BASE2 = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core
#to compile realsense:
DEPEND_UBUNTU = libusb-1.0-0-dev libglfw3-dev libgtk-3-dev

CXXFLAGS  += -DRAI_REALSENSE `pkg-config --cflags realsense2`
LIBS      += `pkg-config --libs realsense2`

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/_make/generic.mk
