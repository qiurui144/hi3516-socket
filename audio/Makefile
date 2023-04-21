# Hisilicon Hi35xx sample Makefile

include ../Makefile.param

HI_SUPPORT_MP3DEC := NO

SMP_SRCS := $(wildcard *.c)

TARGET := $(SMP_SRCS:%.c=%)

CFLAGS += -L$(REL_LIB) -lsecurec -Wl,-rpath=/usr/lib

SMP_SRCS += $(wildcard $(PWD)/adp/audio_aac_adp.c)
SMP_SRCS += $(wildcard $(PWD)/adp/audio_dl_adp.c)

# if need mp3dec
ifeq ($(HI_SUPPORT_MP3DEC), YES)
	MP3_DEC_LIB_DIR := $(PWD)/mp3dec/lib
	CFLAGS += -D__Mp3decSupport__ -L$(MP3_DEC_LIB_DIR) -lmp3dec
	SMP_SRCS += $(wildcard $(PWD)/adp/audio_mp3_adp.c)
endif

TARGET_PATH := $(PWD)

# compile linux or HuaweiLite
include $(PWD)/../$(ARM_ARCH)_$(OSTYPE).mak