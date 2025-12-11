PROJECT = rtp-sample
BUILD_DIR = build

CSTD = -std=c11
CFLAGS += -Wno-missing-prototypes -Wno-sign-compare
OPT = -Og -g
INCLUDES += -I/usr/lib/arm-none-eabi/include

CFILES  = main.c
CFILES += kernel.c

# TODO - you will need to edit these two lines!
DEVICE=stm32f103c8t6
OOCD_FILE = board/stm32f1.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
