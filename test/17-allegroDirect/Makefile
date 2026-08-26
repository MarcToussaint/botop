BASE = ../../../rai
BASE2 = ../../../botop

OBJS = canAPI.o  main.o  RockScissorsPaper.o
DEPEND = Core Gui Kin Geo Perception

RAI_CMAKE = 1

#PYLON_PATH = $(HOME)/git/tests/basler/pylon
#CXXFLAGS += $(shell $(PYLON_PATH)/bin/pylon-config --cflags)
#LDFLAGS += $(shell $(PYLON_PATH)/bin/pylon-config --libs-rpath)
#LIBS += $(shell $(PYLON_PATH)/bin/pylon-config --libs)

CXXFLAGS += -DRAI_BASLER -I$(HOME)/.local/include/pylon
LIBS += -lBHand

include $(BASE)/_make/generic.mk
