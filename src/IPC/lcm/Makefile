BASE   = ../../../rai
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).so

CFLAGS += -O3 -fPIC -w `pkg-config glib-2.0 --cflags`
LIBS += `pkg-config glib-2.0 --libs`

SRCS = $(shell find . -maxdepth 2 -name '*.c' )
OBJS = $(SRCS:%.c=%.o)

include $(BASE)/makeutils/generic.mk
