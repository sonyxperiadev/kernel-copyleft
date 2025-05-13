LINUXINCLUDE   += -I$(NFC_ROOT)/include/uapi/linux/nfc/
LINUXINCLUDE   += -I$(NFC_ROOT)/../../../qcom/opensource/securemsm-kernel/smcinvoke/
LINUXINCLUDE   += -I$(NFC_ROOT)/../../../qcom/opensource/securemsm-kernel/linux/
LINUXINCLUDE   += -I$(NFC_ROOT)/../../../qcom/opensource/securemsm-kernel/include/linux/

obj-m += stm_nfc_i2c.o

ccflags-y := $(call cc-option,-Wno-misleading-indentation)
stm_nfc_i2c-y :=  nfc/st21nfc.o
