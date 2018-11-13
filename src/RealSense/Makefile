BASE = ../../rai
BASE2 = ../../src
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core

CXXFLAGS  += -DRAI_REALSENSE `pkg-config --cflags realsense2`
LIBS      += `pkg-config --libs realsense2`

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
