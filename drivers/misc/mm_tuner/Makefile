##############################################################################
# Makefile for TUNER Driver
##############################################################################

#driver name
ifndef DRV_NAME
DRV_NAME = mm_tuner
endif

LOCAL_INC    += $(src)/include
LOCAL_COMMON += $(src)/include_share

subdir-ccflags-y += -I$(LOCAL_INC)
subdir-ccflags-y += -I$(LOCAL_COMMON)

OBJS := src/tuner_drv.o
OBJS += src/tuner_drv_hw.o

#PATH compile options
CONFIG_MMTUNER_DPATH=SPI
CONFIG_MMTUNER_CPATH=SPI

ifeq ($(CONFIG_MMTUNER_DEBUG), y)
  subdir-ccflags-y += -DDEBUG
endif
ifeq ($(CONFIG_MMTUNER_MN8855x), m)
  subdir-ccflags-y += -fno-pic
endif
ifdef $(CONFIG_MMTUNER_I2CPARPORTX)
  subdir-ccflags-y += -DTUNER_CONFIG_IRQ_PC_LINUX
endif
ifeq ($(CONFIG_MMTUNER_HOSTCPU), pandaboard)
  subdir-ccflags-y += -DPANDABOARD
endif
ifeq ($(CONFIG_MMTUNER_HOSTCPU), dragon410c)
  subdir-ccflags-y += -DDRAGON410C
endif
ifeq ($(CONFIG_NO_DEVICE_TREE), y)
  subdir-ccflags-y += -DTUNER_CONFIG_NO_DEVICE_TREE
endif

#
# SPI compile options
#  CONFIG_MMTUNER_TSIF_SPI_EDGE : Use edge mode for SPI
#  CONFIG_MMTUNER_TSIF_SI_EXT   : Use Extend Read command for packet read
#  CONFIG_MMTUNER_DIV_SPI_MSG   : Use message separetion mode for SPI
#
ifeq ($(CONFIG_MMTUNER_TSIF_SPI_EDGE), y)
  subdir-ccflags-y += -DTUNER_CONFIG_SPI_EDGE
endif
ifeq ($(CONFIG_MMTUNER_TSIF_SPI_EXT), y)
  subdir-ccflags-y += -DTUNER_CONFIG_SPI_EXTREAD
endif
ifeq ($(CONFIG_MMTUNER_DIV_SPI_MSG), y)
  OBJS += src/tuner_drv_hw_spi.o
  subdir-ccflags-y += -DTUNER_CONFIG_SPI_DIVMSG
endif
ifeq ($(CONFIG_MMTUNER_SPI_BREAKCODE), y)
  subdir-ccflags-y += -DTUNER_CONFIG_SPI_BREAKCODE
endif


ifeq ($(CONFIG_MMTUNER_DPATH), SDIO)
  OBJS += src/tuner_drv_hw_sdio.o
  subdir-ccflags-y += -DDPATH_SDIO
endif
ifeq ($(CONFIG_MMTUNER_DPATH), SPI)
  OBJS += src/tuner_drv_hw_spi.o
  subdir-ccflags-y += -DDPATH_SPI
endif
ifeq ($(CONFIG_MMTUNER_CPATH), I2C)
  OBJS += src/tuner_drv_hw_i2c.o
  subdir-ccflags-y += -DCPATH_I2C
endif
ifeq ($(CONFIG_MMTUNER_CPATH), SPI)
  subdir-ccflags-y += -DCPATH_SPI
endif
ifeq ($(CONFIG_MMTUNER_CPATH), SDIO)
  subdir-ccflags-y += -DCPATH_SDIO
endif

#SPI performance measurement debug
#GP Get Packet
#DR Get DATAREADY
#subdir-ccflags-y += -DSPI_GP_MESURE_DEBUG
#subdir-ccflags-y += -DSPI_DR_MESURE_DEBUG

#
# PID NULL FILTER
#
ifeq ($(CONFIG_MMTUNER_NULLFILTER), y)
  subdir-ccflags-y += -DTUNER_CONFIG_PF_NULLFILTER
endif

#
# BYTE ORDER CONFIGURATION.
#
ifeq ($(CONFIG_MMTUNER_MSBFIRST), y)
  subdir-ccflags-y += -DTUNER_CONFIG_SLV_MSBFIRST
endif

#
# DMA ALIGNMENT
#
ifdef CONFIG_MMTUNER_ALIGN
  subdir-ccflags-y += -DTUNER_CONFIG_SPI_ALIGN=${CONFIG_MMTUNER_ALIGN}
endif


#
# AES compile options
#
#   CONFIG_MMTUNER_AES = y : Use AES but does not check TS header of 1st packet
#   CONFIG_MMTUNER_AES = c : Use AES and check TS header of 1st packet
#
ifeq ($(CONFIG_MMTUNER_AES), y)
  OBJS += src/tuner_drv_hw_aes.o
  subdir-ccflags-y += -DTUNER_CONFIG_AES_ENABLE
endif

ifeq ($(CONFIG_MMTUNER_AES), c)
  OBJS += src/tuner_drv_hw_aes.o
  OBJS += src/aes_sample.o
  subdir-ccflags-y += -DTUNER_CONFIG_AES_ENABLE
  subdir-ccflags-y += -DTUNER_CONFIG_AES_TSCHK
endif


#
# GPIF compile options
#
# CONFIG_MMTUNER_DPATH = GPIF  : Use GPIF for DPATH
# CONFIG_MMTUNER_CPATH = GPIF  : Use GPIF for CPATH
# CONFIG_MMTUNER_TSIF_GPIF_DMA : Use DMA transfer for DPATH
#
ifeq ($(CONFIG_MMTUNER_DPATH), GPIF)
  OBJS += src/gpif.o
  OBJS += src/gpmc.o
  OBJS += src/tuner_drv_hw_gpif.o
  subdir-ccflags-y += -DDPATH_GPIF
endif
ifeq ($(CONFIG_MMTUNER_CPATH), GPIF)
  subdir-ccflags-y += -DCPATH_GPIF
endif
ifeq ($(CONFIG_MMTUNER_TSIF_GPIF_DMA), y)
  subdir-ccflags-y += -DTUNER_CONFIG_GPIF_DMA
endif

#
# Addtional libaray path is here
#
ifdef CONFIG_MMTUNER_LIBS
  subdir-ccflags-y += -I${CONFIG_MMTUNER_LIBS}
endif



$(DRV_NAME)-objs += $(OBJS)

obj-$(CONFIG_MMTUNER_MN8855x) = $(DRV_NAME).o

clean-files := *.o .*.cmd modules.builtin modules.order
