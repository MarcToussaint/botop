BASE = ../../rai

DEPEND = Core Gui

OBJS = main.o examples_common.o

EIGEN=1

CPATH := $(CPATH):$(HOME)/git/libfranka/include
LPATH := $(LPATH):$(HOME)/git/libfranka/build
LIBS += -lfranka

include $(BASE)/makeutils/generic.mk
