//SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */
/* This file should be removed once TSP feature POR */
#ifndef _IPA_TEST_MODULE_TSP_H_
#define _IPA_TEST_MODULE_TSP_H_

#include <linux/msm_ipa.h>

#define IPA_IOCTL_TSP_GET_INGR_TC_NUM           91
#define IPA_IOCTL_TSP_GET_EGR_EP_NUM            92
#define IPA_IOCTL_TSP_GET_EGR_TC_NUM            93
#define IPA_IOCTL_TSP_GET_INGR_TC               94
#define IPA_IOCTL_TSP_GET_EGR_EP                95
#define IPA_IOCTL_TSP_GET_EGR_TC                96
#define IPA_IOCTL_TSP_SET_INGR_TC               97
#define IPA_IOCTL_TSP_SET_EGR_EP                98
#define IPA_IOCTL_TSP_SET_EGR_TC                99
#define IPA_IOCTL_TSP_COMMIT                    100
#define IPA_IOCTL_TSP_RESET                     101

/**
 * struct ipa_ioc_tsp_ingress_class_params - IPA Ingress traffic-class params
 *
 * @max_burst: Maximal-burst allowed in bytes
 * @max_rate: Maximal bandwidth rate for matching ingress traffic-class in 1.2MB/second units
 * @include_l2_len: Bool - Include L2 size for bandwidth calculation?
 */
struct ipa_ioc_tsp_ingress_class_params {
	__u16 max_burst;
	__u16 max_rate;
	__u32 include_l2_len;
};

/**
 * struct ipa_ioc_tsp_ingress_class_get - IPA Ingress traffic-class get ioctl struct
 *
 * @index: Ingress traffic-class index
 * @reserved: Reserved for alignment
 * @params: Output parameter - Ingress traffic-class Params, valid only when
 *          ioctl return val is non-negative
 */
struct ipa_ioc_tsp_ingress_class_get {
	__u32 index;
	__u32 reserved;
	struct ipa_ioc_tsp_ingress_class_params params;
};

/**
 * struct ipa_ioc_tsp_ingress_class_set - IPA Ingress traffic-class set ioctl struct
 *
 * @index: Ingress traffic-class index
 * @commit: Bool - Commit the setting to the HW
 * @params: Params to set
 */
struct ipa_ioc_tsp_ingress_class_set {
	__u32 index;
	__u32 commit;
	struct ipa_ioc_tsp_ingress_class_params params;
};

/**
 * struct ipa_ioc_tsp_egress_prod_params - IPA TSP-enabled producer params
 *
 * @client: For output - which "clients" pipe does this entry apply to?
 * @max_rate: Maximal bandwidth rate for producer in 1.2MB/second units
 * @max_burst: Maximal-burst allowed in bytes
 * @max_out_bytes: max output size in bytes allowed per producer
 * @tc_lo: Lowest egress traffic-class index assignes to this producer
 * @tc_hi: Highest egress traffic-class index assignes to this producer
 * @policing_by_max_out: Bool - enable policing by max output size
 *                       in case of valid egress_tc, max output size policing will be valid
 *                       regardless to this flag
 * @reserved: Reserved for alignment
 */
struct ipa_ioc_tsp_egress_prod_params {
	enum ipa_client_type client;
	__u16 max_rate;
	__u16 max_burst;
	__u32 max_out_bytes;
	__u8 tc_lo;
	__u8 tc_hi;
	__u8 policing_by_max_out;
	__u8 reserved;
};

/**
 * struct ipa_ioc_tsp_egress_prod_get - IPA TSP-enabled producer get ioctl struct
 *
 * @index: TSP-enabled producer index
 * @reserved: Reserved for alignment
 * @params: Output parameter - TSP-enabled producer Params, valid only when
 *          ioctl return val is non-negative
 */
struct ipa_ioc_tsp_egress_prod_get {
	__u32 index;
	__u32 reserved;
	struct ipa_ioc_tsp_egress_prod_params params;
};

/**
 * struct ipa_ioc_tsp_egress_prod_set - IPA TSP-enabled producer set ioctl struct
 *
 * @index: Producer index
 * @commit: Bool - Commit the setting to the HW
 * @params: Params to set
 */
struct ipa_ioc_tsp_egress_prod_set {
	__u32 index;
	__u32 commit;
	struct ipa_ioc_tsp_egress_prod_params params;
};

/**
 * struct ipa_ioc_tsp_egress_class_params - IPA egress traffic-class params
 *
 * @guaranteed_rate: Guaranteed bandwidth rate for traffic-class in 1.2MB/second units
 *      If guaranteed_rate, guaranteed_freq and guaranteed_burst are all set to 0,
 *      the guaranteed bandwidth rate will be disabled,
 *      and only maximal bandwidth rate will be considered.
 * @max_rate: Maximal bandwidth rate for traffic-class in 1.2MB/second units
 * @guaranteed_burst: Maximal-burst allowed for guaranteed bandwidth rate (in bytes)
 * @max_burst: Maximal-burst allowed for maximal bandwidth rate (in bytes)
 */
struct ipa_ioc_tsp_egress_class_params {
	__u16 guaranteed_rate;
	__u16 max_rate;
	__u16 guaranteed_burst;
	__u16 max_burst;
};

/**
 * struct ipa_ioc_tsp_egress_class_get - IPA Egress traffic-class get ioctl struct
 *
 * @index: Egress traffic-class index
 * @reserved: Reserved for alignment
 * @params: Output parameter - Egress traffic-class Params, valid only when
 *          ioctl return val is non-negative
 */
struct ipa_ioc_tsp_egress_class_get {
	__u32 index;
	__u32 reserved;
	struct ipa_ioc_tsp_egress_class_params params;
};

/**
 * struct ipa_ioc_tsp_egress_class_set - IPA Engress traffic-class set ioctl struct
 *
 * @index: Egress traffic-class index
 * @commit: Bool - Commit the setting to the HW
 * @params: Params to set
 */
struct ipa_ioc_tsp_egress_class_set {
	__u32 index;
	__u32 commit;
	struct ipa_ioc_tsp_egress_class_params params;
};

#define IPA_IOC_TSP_GET_INGR_TC_NUM _IOWR(IPA_IOC_MAGIC, \
				IPA_IOCTL_TSP_GET_INGR_TC_NUM, \
				uint32_t)
#define IPA_IOC_TSP_GET_EGR_EP_NUM _IOWR(IPA_IOC_MAGIC, \
				IPA_IOCTL_TSP_GET_EGR_EP_NUM, \
				uint32_t)
#define IPA_IOC_TSP_GET_EGR_TC_NUM _IOWR(IPA_IOC_MAGIC, \
				IPA_IOCTL_TSP_GET_EGR_TC_NUM, \
				uint32_t)
#define IPA_IOC_TSP_GET_INGR_TC _IOWR(IPA_IOC_MAGIC, \
				IPA_IOCTL_TSP_GET_INGR_TC, \
				struct ipa_ioc_tsp_ingress_class_get)
#define IPA_IOC_TSP_GET_EGR_EP _IOWR(IPA_IOC_MAGIC, \
				IPA_IOCTL_TSP_GET_EGR_EP, \
				struct ipa_ioc_tsp_egress_prod_get)
#define IPA_IOC_TSP_GET_EGR_TC _IOWR(IPA_IOC_MAGIC, \
				IPA_IOCTL_TSP_GET_EGR_TC, \
				struct ipa_ioc_tsp_egress_class_get)
#define IPA_IOC_TSP_SET_INGR_TC _IOWR(IPA_IOC_MAGIC, \
				IPA_IOCTL_TSP_SET_INGR_TC, \
				struct ipa_ioc_tsp_ingress_class_set)
#define IPA_IOC_TSP_SET_EGR_EP _IOWR(IPA_IOC_MAGIC, \
				IPA_IOCTL_TSP_SET_EGR_EP, \
				struct ipa_ioc_tsp_egress_prod_set)
#define IPA_IOC_TSP_SET_EGR_TC _IOWR(IPA_IOC_MAGIC, \
				IPA_IOCTL_TSP_SET_EGR_TC, \
				struct ipa_ioc_tsp_egress_class_set)
#define IPA_IOC_TSP_COMMIT _IO(IPA_IOC_MAGIC, IPA_IOCTL_TSP_COMMIT)
#define IPA_IOC_TSP_RESET _IO(IPA_IOC_MAGIC, IPA_IOCTL_TSP_RESET)

#endif /* _IPA_TEST_MODULE_TSP_H_ */
