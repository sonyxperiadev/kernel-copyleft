LINUXINCLUDE += -I$(SSG_MODULE_ROOT)/ \
                -I$(SSG_MODULE_ROOT)/linux/ \
                -I$(SSG_MODULE_ROOT)/include/linux/ \
                -I$(SSG_MODULE_ROOT)/include/uapi/ \
                -I$(SSG_MODULE_ROOT)/include/uapi/linux/

ifneq ($(CONFIG_ARCH_QTI_VM), y)
    LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig.h
    include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig.conf
endif

#Enable Qseecom if CONFIG_ARCH_KHAJE OR CONFIG_ARCH_KHAJE or CONFIG_QTI_QUIN_GVM is set to y
ifneq (, $(filter y, $(CONFIG_QTI_QUIN_GVM) $(CONFIG_ARCH_KHAJE) $(CONFIG_ARCH_SA8155) $(CONFIG_ARCH_BLAIR) $(CONFIG_ARCH_SA6155)))
    include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qseecom.conf
    LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qseecom.h
else
    LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qseecom_compat.h
endif

obj-$(CONFIG_QSEECOM) += qseecom_dlkm.o
qseecom_dlkm-objs := qseecom/qseecom.o

include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_smcinvoke.conf
LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_smcinvoke.h

obj-$(CONFIG_QCOM_SMCINVOKE) += smcinvoke_dlkm.o
smcinvoke_dlkm-objs := smcinvoke/smcinvoke_kernel.o smcinvoke/smcinvoke.o

obj-$(CONFIG_QTI_TZ_LOG) += tz_log_dlkm.o
tz_log_dlkm-objs := tz_log/tz_log.o

obj-$(CONFIG_CRYPTO_DEV_QCEDEV) += qce50_dlkm.o
qce50_dlkm-objs := crypto-qti/qce50.o

obj-$(CONFIG_CRYPTO_DEV_QCEDEV) += qcedev-mod_dlkm.o
qcedev-mod_dlkm-objs := crypto-qti/qcedev.o crypto-qti/qcedev_smmu.o

obj-$(CONFIG_CRYPTO_DEV_QCRYPTO) += qcrypto-msm_dlkm.o
qcrypto-msm_dlkm-objs := crypto-qti/qcrypto.o

obj-$(CONFIG_HDCP_QSEECOM) += hdcp_qseecom_dlkm.o
hdcp_qseecom_dlkm-objs := hdcp/hdcp_main.o hdcp/hdcp_smcinvoke.o hdcp/hdcp_qseecom.o

obj-$(CONFIG_HW_RANDOM_MSM_LEGACY) += qrng_dlkm.o
qrng_dlkm-objs := qrng/msm_rng.o

ifneq (, $(filter y, $(ARCH_QTI_VM) $(CONFIG_ARCH_PINEAPPLE) $(CONFIG_ARCH_SUN)))
    include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_smmu_proxy.conf
    LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_smmu_proxy.h

    obj-$(CONFIG_QTI_SMMU_PROXY) += smmu_proxy_dlkm.o
    smmu_proxy_dlkm-objs := smmu-proxy/qti-smmu-proxy-common.o
    ifneq ($(CONFIG_ARCH_QTI_VM), y)
    smmu_proxy_dlkm-objs += smmu-proxy/qti-smmu-proxy-pvm.o
    else
    smmu_proxy_dlkm-objs += smmu-proxy/qti-smmu-proxy-tvm.o
    endif
endif
