# Hisilicon Hi35xx sample Makefile

include ../Makefile.param

SMP_SRCS := sample_dis.c
ifeq ($(GYRO_DIS), y)
SMP_SRCS += sample_gyro_dis.c
SMP_SRCS += sample_fov2ldc.c
endif
TARGET := sample_dis

TARGET_PATH := $(PWD)

# compile linux or HuaweiLite
include $(PWD)/../$(ARM_ARCH)_$(OSTYPE).mak
