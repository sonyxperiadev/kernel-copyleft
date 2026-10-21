LINUXINCLUDE += -I$(SSG_MODULE_ROOT)/ \
                -I$(SSG_MODULE_ROOT)/linux/ 

ifneq ($(CONFIG_ARCH_QTI_VM), y)
    LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig.h
    include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig.conf
endif

#Enable Qseecom if CONFIG_ARCH_KHAJE OR CONFIG_ARCH_KHAJE or CONFIG_QTI_QUIN_GVM or CONFIG_ARCH_MONACO or CONFIG_ARCH_SCUBA_AUTO or CONFIG_ARCH_SA410M is set to y
ifneq (, $(filter y, $(CONFIG_QTI_QUIN_GVM) $(CONFIG_ARCH_KHAJE) $(CONFIG_ARCH_SA8155) $(CONFIG_ARCH_MONACO) $(CONFIG_ARCH_SCUBA_AUTO) $(CONFIG_ARCH_SA410M)))
    include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qseecom.conf
    LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qseecom.h

    obj-$(CONFIG_QSEECOM) += qseecom_dlkm.o
    qseecom_dlkm-objs := qseecom/qseecom.o
endif

include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_smcinvoke.conf
LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_smcinvoke.h

obj-$(CONFIG_QCOM_SMCINVOKE) += smcinvoke_dlkm.o
smcinvoke_dlkm-objs := smcinvoke/smcinvoke_kernel.o smcinvoke/smcinvoke.o

obj-$(CONFIG_QTI_TZ_LOG) += tz_log_dlkm.o
tz_log_dlkm-objs := tz_log/tz_log.o

obj-$(CONFIG_CRYPTO_DEV_QCEDEV) += qce50_dlkm.o
qce50_dlkm-objs := crypto-qti/qce50.o

obj-$(CONFIG_CRYPTO_DEV_QCEDEV) += qcedev-mod_dlkm.o
qcedev-mod_dlkm-objs := crypto-qti/qcedev.o crypto-qti/qcedev_smmu.o crypto-qti/compat_qcedev.o

obj-$(CONFIG_CRYPTO_DEV_QCRYPTO) += qcrypto-msm_dlkm.o
qcrypto-msm_dlkm-objs := crypto-qti/qcrypto.o

obj-$(CONFIG_HDCP_QSEECOM) += hdcp_qseecom_dlkm.o
hdcp_qseecom_dlkm-objs := hdcp/hdcp_qseecom.o

obj-$(CONFIG_HW_RANDOM_MSM_LEGACY) += qrng_dlkm.o
qrng_dlkm-objs := qrng/msm_rng.o
