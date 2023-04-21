# Hisilicon mpp sample Makefile
include ../Makefile.param

SMP_SRCS := $(wildcard *.c)
TARGET := $(SMP_SRCS:%.c=%)

MPI_LIBS += $(REL_LIB)/libtde.a

TARGET_PATH := $(PWD)

ifeq ($(OSTYPE), liteos)
ifeq ($(ARM_ARCH), amp)
.PHONY : clean all
all:

clean:
else
include $(PWD)/../$(ARM_ARCH)_$(OSTYPE).mak
endif
else
include $(PWD)/../$(ARM_ARCH)_$(OSTYPE).mak
endif

