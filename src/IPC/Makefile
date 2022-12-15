BASE = ../../rai
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core extern_lcm
DEPEND_UBUNTU = libglib2.0-dev

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/makeutils/generic.mk
