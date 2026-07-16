BASE = ../../../rai
BASE2 = ../../../botop

DEPEND = Core Gui Kin Geo Perception

OPENCV = 1
RAI_CMAKE = 1

PYLON_PATH = $(HOME)/git/tests/basler/pylon

# CXXFLAGS += $(shell $(PYLON_PATH)/bin/pylon-config --cflags)
# LDFLAGS += $(shell $(PYLON_PATH)/bin/pylon-config --libs-rpath)
# LIBS += $(shell $(PYLON_PATH)/bin/pylon-config --libs)

LIBS += -lpylonbase -lpylonutility -lGenApi_gcc_v3_5_Basler_pylon_v1 -lGCBase_gcc_v3_5_Basler_pylon_v1

include $(BASE)/_make/generic.mk
