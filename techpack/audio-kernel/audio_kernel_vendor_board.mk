AUDIO_DLKM_ENABLE := false
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
  ifeq ($(TARGET_KERNEL_DLKM_AUDIO_OVERRIDE),true)
    AUDIO_DLKM_ENABLE := true
  endif
else
  AUDIO_DLKM_ENABLE := true
endif

ifeq ($(AUDIO_DLKM_ENABLE), true)
  include vendor/qcom/opensource/audio-kernel/audio_kernel_modules.mk
  ifeq ($(ENABLE_AUDIO_LEGACY_TECHPACK),true)
    include vendor/qcom/opensource/audio-kernel/legacy/audio_kernel_modules.mk
  endif
  BOARD_VENDOR_KERNEL_MODULES += $(AUDIO_KERNEL_MODULES)
endif
