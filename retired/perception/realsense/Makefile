BASE = ../../rai

DEPEND = Core Gui

UBUNTU_DEPEND = libglfw3-dev

GL=1

CXXFLAGS  += -DRAI_REALSENSE `pkg-config --cflags realsense2`
LIBS      += `pkg-config --libs realsense2`

include $(BASE)/makeutils/generic.mk
