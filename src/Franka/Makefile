BASE = ../../rai
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core Control

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

CPATH := $(CPATH):$(HOME)/git/libfranka/include
LPATH := $(LPATH):$(HOME)/git/libfranka/build
LIBS += -lfranka

include $(BASE)/build/generic.mk
