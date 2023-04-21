#****** date **********************************************************
DATE        := $(shell date +%Y-%m-%d)

#****** compiler ***********************************************************
ifeq ($(PARAM_FILE), )
     PARAM_FILE := ./../../../Makefile.param
     include $(PARAM_FILE)
endif

STRIP = $(CROSS)strip

#****** target name ****************************************************
TARGET = libmp3dec

#****** output directory *******************************************
ROOT_DIR = .
OBJ_DIR  = $(ROOT_DIR)/lib/obj
TAR_DIR  = $(ROOT_DIR)/lib

#****** source directory ***********************************************
LOCAL_PATH = .
MP3_EXT_PATH = $(LOCAL_PATH)/OpenSource/mp3dec
WRAP_PATH = $(LOCAL_PATH)
PUB_INCLUDE = $(REL_INC)

CFG_INC = \
	-I$(PUB_INCLUDE) \
	-I$(MP3_EXT_PATH)/src \
	-I$(MP3_EXT_PATH)/include \
	-I$(MP3_EXT_PATH)/layer12 \
	-I$(WRAP_PATH)/src \
	-I$(WRAP_PATH)/include \
	-I$(LOCAL_PATH)/src

#
# compiler flags (for normal, DEBUG, and WMOPS compilation)
#
CFLAGS_NORM   = $(LIBS_CFLAGS)
CFLAGS_NORM  += -O4 -DTARGET_ARM -mlittle-endian  -DMPEG_LAYER12_SUPPORT
ifeq "$(HI_MP3DECODE_INTERLEACED_ENA)" "YES"
CFLAGS_NORM  += -DINTERLEAVED_OUTPUT
endif

ifeq ($(PROFILEER),YES)
	CFLAGS_NORM  += -DCPULOADING_PROFILE
endif
	CFLAGS_NORM  += -DARM_CPU_MHZ=$(CPU_MHZ)

#****** compiler param ***********************************************************
CFLAGS += -Wall $(CFLAGS_NORM)
CFLAGS += $(CFG_INC)

CC += $(CFLAGS)

CFLAGSDEPEND = -MM $(CFLAGS)                    # for make depend
COMPILE=$(CC) -c $(CFLAGS) -o "$(OBJ_DIR)/$(PREFIX)$(*F).o" "$<"
#
# construct SPC library name:
#
TARGET_LIB = $(TAR_DIR)/$(TARGET).a
TARGET_SHARE_LIB = $(TAR_DIR)/$(TARGET).so

#
# source/object files
#

C_SRCS := \
	$(MP3_EXT_PATH)/layer12/layer12_frame.c

CPP_SRC := \
	$(MP3_EXT_PATH)/src/pvmp3_normalize.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_alias_reduction.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_crc.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_decode_header.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_decode_huff_cw.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_getbits.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_dequantize_sample.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_framedecoder.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_get_main_data_size.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_get_side_info.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_get_scale_factors.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_mpeg2_get_scale_data.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_mpeg2_get_scale_factors.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_mpeg2_stereo_proc.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_huffman_decoding.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_huffman_parsing.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_tables.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_imdct_synth.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_mdct_6.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_dct_6.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_poly_phase_synthesis.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_equalizer.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_seek_synch.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_stereo_proc.cpp \
 	$(MP3_EXT_PATH)/src/pvmp3_reorder.cpp \

ifeq ($(HIARCH),hi3559av100es)
	HI_SUPPORT_ARM_64:=YES
else ifeq ($(HIARCH),hi3559av100)
	HI_SUPPORT_ARM_64:=YES
else
	HI_SUPPORT_ARM_64:=NO
endif

ifeq ($(HI_SUPPORT_ARM_64),YES)
CPP_SRC += \
	$(MP3_EXT_PATH)/src/pvmp3_polyphase_filter_window.cpp \
	$(MP3_EXT_PATH)/src/pvmp3_mdct_18.cpp \
	$(MP3_EXT_PATH)/src/pvmp3_dct_9.cpp \
	$(MP3_EXT_PATH)/src/pvmp3_dct_16.cpp
else
ASM_SRC := \
	$(MP3_EXT_PATH)/src/asm/pvmp3_polyphase_filter_window_gcc.s \
 	$(MP3_EXT_PATH)/src/asm/pvmp3_mdct_18_gcc.s \
 	$(MP3_EXT_PATH)/src/asm/pvmp3_dct_9_gcc.s \
	$(MP3_EXT_PATH)/src/asm/pvmp3_dct_16_gcc.s
endif

WRAP_SRC := \
	$(WRAP_PATH)/src/mp3dec_wrap.c

all:build_prepare target_lib target_share_lib

C_OBJ := $(patsubst %.c,%.o,$(C_SRCS))
$(C_OBJ): %.o: %.c
	$(CC) -c -g -O2 $^ -o $@

CPP_OBJ := $(patsubst %.cpp,%.o,$(CPP_SRC))
$(CPP_OBJ): %.o: %.cpp
	$(CC) -c -g -O2 $^ -o $@

ASM_OBJ=$(patsubst %.s,%.o,$(ASM_SRC))
$(ASM_OBJ): %.o: %.s
	$(CC) -c -g -O2 $^ -o $@

WRAP_OBJ=$(patsubst %.c,%.o,$(WRAP_SRC))
$(WRAP_OBJ): %.o: %.c
	$(CC) -c -g -O2 $^ -o $@

build_prepare:
	mkdir -p $(TAR_DIR)

target_lib: $(CPP_OBJ) $(C_OBJ) $(ASM_OBJ) $(WRAP_OBJ)
	$(AR) rc $(TARGET_LIB) $(CPP_OBJ) $(C_OBJ) $(ASM_OBJ) $(WRAP_OBJ)
	$(STRIP) --strip-unneeded $(TARGET_LIB)

target_share_lib: $(CPP_OBJ) $(C_OBJ) $(ASM_OBJ) $(WRAP_OBJ)
	$(CC) -o $(TARGET_SHARE_LIB) -shared $(CPP_OBJ) $(C_OBJ) $(ASM_OBJ) $(WRAP_OBJ) $(LIBS_LD_CFLAGS)

clean:
	-@rm -f $(CPP_OBJ)
	-@rm -f $(C_OBJ)
	-@rm -f $(WRAP_OBJ)
	-@rm -f $(ASM_OBJ)
	-@rm -f $(TARGET_LIB)
	-@rm -f $(TARGET_SHARE_LIB)
	-@rm -rf $(TAR_DIR)