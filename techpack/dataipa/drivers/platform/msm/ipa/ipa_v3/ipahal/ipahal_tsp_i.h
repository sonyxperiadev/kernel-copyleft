
/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _IPAHAL_TSP_I_H_
#define _IPAHAL_TSP_I_H_

/**
* struct ipahal_tsp_ingress_class - IPA Ingress traffic-class
*
* @last_rtc: For HW use, initialize to 0x0000
* @max_bucket: For HW use, initialize to same value as max_burst
* @max_burst: Maximal-burst allowed in bytes
* @max_rate: Maximal bandwidth rate for matching ingress traffic-class in 1.2MB/second units
*/
struct ipahal_tsp_ingress_class {
	u16 last_rtc;
	u16 max_bucket;
	u16 max_burst;
	u16 max_rate;
};

/**
 * struct ipahal_tsp_egress_prod_even - IPA TSP-enabled producer (even index)
 *
 * @last_rtc: For HW use, initialize to 0x0000
 * @max_bucket: For HW use, initialize to same value as max_burst
 * @max_rate: Maximal bandwidth rate for producer in 1.2MB/second units
 * @max_freq: In units of 0.833*usec/64KB. Claculated as:
 * 		MAX-Freq = 65536/max_rate (always be rounded up)
 * @max_burst: Maximal-burst allowed in bytes
 * @reserved: Reserved
 */
struct ipahal_tsp_egress_prod_even {
	u16 last_rtc;
	u16 max_bucket;
	u16 max_rate;
	u16 max_freq;
	u16 max_burst;
	u16 reserved;
};

/**
 * struct ipahal_tsp_egress_prod_odd - IPA TSP-enabled producer (odd index)
 *
 * @reserved: Reserved
 * @max_burst: Maximal-burst allowed in bytes
 * @last_rtc: For HW use, initialize to 0x0000
 * @max_bucket: For HW use, initialize to same value as max_burst
 * @max_rate: Maximal bandwidth rate for producer in 1.2MB/second units
 * @max_freq: In units of 0.833*usec/64KB. Claculated as:
 * 		MAX-Freq = 65536/max_rate (always be rounded up)
 */
struct ipahal_tsp_egress_prod_odd {
	u16 reserved;
	u16 max_burst;
	u16 last_rtc;
	u16 max_bucket;
	u16 max_rate;
	u16 max_freq;
};

/**
 * union ipahal_tsp_egress_prod - IPA TSP-enabled producer (even or odd)
 *
 * @even: TSP-enabled producer (even index)
 * @odd: TSP-enabled producer (odd index)
 */
union ipahal_tsp_egress_prod {
	struct ipahal_tsp_egress_prod_even even;
	struct ipahal_tsp_egress_prod_odd odd;
};

/**
 * struct ipahal_tsp_egress_prod_pair - IPA TSP-enabled producer pair (even and odd)
 *
 * @even: TSP-enabled producer (even index)
 * @odd: TSP-enabled producer (odd index)
 */
struct ipahal_tsp_egress_prod_pair {
	struct ipahal_tsp_egress_prod_even even;
	struct ipahal_tsp_egress_prod_odd odd;
};

/**
 * struct ipahal_tsp_egress_class_even - IPA egress traffic-class (even index)
 *
 * @last_rtc: For HW use, initialize to 0x0000
 * @reserved: Reserved
 * @guaranteed_bucket: For HW use, initialize to same value as guaranteed_burst
 * @max_bucket: For HW use, initialize to same value as max_burst
 * @guaranteed_rate: Guaranteed bandwidth rate for traffic-class in 1.2MB/second units
 * @max_rate: Maximal bandwidth rate for traffic-class in 1.2MB/second units
 * @guaranteed_freq: In units of 0.833*usec/64KB, Calculated as:
 * 			 guaranteed_freq = 65536/guaranteed_rate (always be rounded up)
 * @max_freq: In units of 0.833*usec/64KB, Calculated as:
 * 			 max_freq = 65536/max_rate (always be rounded up)
 * @guaranteed_burst: Maximal-burst allowed for guaranteed bandwidth rate (in bytes)
 * @max_burst: Maximal-burst allowed for maximal bandwidth rate (in bytes)
 */
struct ipahal_tsp_egress_class_even {
	u16 last_rtc;
	u16 reserved;
	u16 guaranteed_bucket;
	u16 max_bucket;
	u16 guaranteed_rate;
	u16 max_rate;
	u16 guaranteed_freq;
	u16 max_freq;
	u16 guaranteed_burst;
	u16 max_burst;
};

/**
 * struct ipahal_tsp_egress_class_odd - IPA egress traffic-class (odd index)
 *
 * @guaranteed_burst: Maximal-burst allowed for guaranteed bandwidth rate (in bytes)
 * @max_burst: Maximal-burst allowed for maximal bandwidth rate (in bytes)
 * @last_rtc: For HW use, initialize to 0x0000
 * @reserved: Reserved
 * @guaranteed_bucket: For HW use, initialize to same value as guaranteed_burst
 * @max_bucket: For HW use, initialize to same value as max_burst
 * @guaranteed_rate: Guaranteed bandwidth rate for traffic-class in 1.2MB/second units
 * @max_rate: Maximal bandwidth rate for traffic-class in 1.2MB/second units
 * @guaranteed_freq: In units of 0.833*usec/64KB, Calculated as:
 * 			 guaranteed_freq = 65536/guaranteed_rate (always be rounded up)
 * @max_freq: In units of 0.833*usec/64KB, Calculated as:
 * 			 max_freq = 65536/max_rate (always be rounded up)
 */
struct ipahal_tsp_egress_class_odd {
	u16 guaranteed_burst;
	u16 max_burst;
	u16 last_rtc;
	u16 reserved;
	u16 guaranteed_bucket;
	u16 max_bucket;
	u16 guaranteed_rate;
	u16 max_rate;
	u16 guaranteed_freq;
	u16 max_freq;
};

/**
 * union ipahal_tsp_egress_prod - IPA egress traffic-class (even or odd)
 *
 * @even: egress traffic-class (even index)
 * @odd: egress traffic-class (odd index)
 */
union ipahal_tsp_egress_class {
	struct ipahal_tsp_egress_class_even even;
	struct ipahal_tsp_egress_class_odd odd;
};

/**
 * struct ipahal_tsp_egress_prod_pair - IPA egress traffic-class pair (even and odd)
 *
 * @even: egress traffic-class (even index)
 * @odd: egress traffic-class (odd index)
 */
struct ipahal_tsp_egress_class_pair {
	struct ipahal_tsp_egress_class_even even;
	struct ipahal_tsp_egress_class_odd odd;
};

#endif /* _IPAHAL_TSP_I_H_ */
