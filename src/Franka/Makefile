BASE = ../../rai
BASE2 = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core 

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

CPATH := $(CPATH):$(HOME)/git/libfranka/include
LPATH := $(LPATH):$(HOME)/git/libfranka/build
LIBS += -lfranka

include $(BASE)/_make/generic.mk
