BASE = ../../rai
BASE2 = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

OPTIM=fast

OPENCV = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/_make/generic.mk
