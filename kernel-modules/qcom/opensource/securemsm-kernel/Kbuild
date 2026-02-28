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
ifneq (, $(filter y, $(CONFIG_QTI_QUIN_GVM) $(CONFIG_ARCH_KHAJE) $(CONFIG_ARCH_SA8155) $(CONFIG_ARCH_BLAIR) $(CONFIG_ARCH_SA6155) $(CONFIG_ARCH_MONACO)))
    include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qseecom.conf
    LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qseecom.h
else
    LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qseecom_compat.h
endif

ifeq ($(CONFIG_ARCH_QTI_VM), y)
    ifneq (, $(filter y, $(CONFIG_ARCH_LEMANS)))
        include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qrng.conf
        LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qrng.h
    endif
endif

obj-$(CONFIG_QCOM_SI_CORE_TEST) += si_core_test.o
si_core_test-objs := si_core_tests/si_core_test.o

obj-$(CONFIG_QSEECOM) += qseecom_dlkm.o
qseecom_dlkm-objs := qseecom/qseecom.o
qseecom_dlkm-$(CONFIG_COMPAT) += qseecom/qseecom_32bit_impl.o

include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_smcinvoke.conf
LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_smcinvoke.h

obj-$(CONFIG_QCOM_SMCINVOKE) += smcinvoke_dlkm.o
ifneq ($(CONFIG_QCOM_SI_CORE), y)
    smcinvoke_dlkm-objs := smcinvoke/compat/smcinvoke_kernel.o
    smcinvoke_dlkm-objs += smcinvoke/compat/smcinvoke.o
else
    smcinvoke_dlkm-objs := smcinvoke/si_core_xts/qseecom.o
    smcinvoke_dlkm-objs += smcinvoke/si_core_xts/smci_kernel.o
    smcinvoke_dlkm-objs += smcinvoke/si_core_xts/smci.o
endif

obj-$(CONFIG_QTI_TZ_LOG) += tz_log_dlkm.o
tz_log_dlkm-objs := tz_log/tz_log.o

obj-$(CONFIG_CRYPTO_DEV_QCEDEV) += qce50_dlkm.o
qce50_dlkm-objs := crypto-qti/qce50.o

obj-$(CONFIG_CRYPTO_DEV_QCEDEV) += qcedev-mod_dlkm.o
qcedev-mod_dlkm-objs := crypto-qti/qcedev.o crypto-qti/qcedev_smmu.o
qcedev-mod_dlkm-$(CONFIG_COMPAT) += crypto-qti/compat_qcedev.o

obj-$(CONFIG_CRYPTO_DEV_QCRYPTO) += qcrypto-msm_dlkm.o
qcrypto-msm_dlkm-objs := crypto-qti/qcrypto.o

obj-$(CONFIG_HDCP_QSEECOM) += hdcp_qseecom_dlkm.o
hdcp_qseecom_dlkm-objs := hdcp/hdcp_main.o hdcp/hdcp_smcinvoke.o hdcp/hdcp_qseecom.o

obj-$(CONFIG_HW_RANDOM_MSM_LEGACY) += qrng_dlkm.o
qrng_dlkm-objs := qrng/msm_rng.o

ifneq (, $(filter y, $(ARCH_QTI_VM) $(CONFIG_ARCH_PINEAPPLE) $(CONFIG_ARCH_SUN) $(CONFIG_ARCH_PARROT)))
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

#Enable QCE Dev Frontend if CONFIG_QTI_QUIN_GVM is set to y
ifeq ($(CONFIG_QTI_QUIN_GVM), y)

    include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qcedev_fe.conf
    LINUXINCLUDE += -include $(SSG_MODULE_ROOT)/config/sec-kernel_defconfig_qcedev_fe.h

    obj-$(CONFIG_QCEDEV_FE) += qcedev_fe_dlkm.o
    qcedev_fe_dlkm-objs := qcedev_fe/qcedev_fe.o qcedev_fe/qcedev_smmu.o
endif #CONFIG_QTI_QUIN_GVM
