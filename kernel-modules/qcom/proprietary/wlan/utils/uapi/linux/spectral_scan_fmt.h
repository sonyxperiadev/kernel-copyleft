/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

/**
 * DOC: spectral_scan_fmt.h
 *
 * WLAN Host Device Driver Spectral Scan Implementation
 */

#ifndef _SPECTRAL_SCAN_FMT_H_
#define _SPECTRAL_SCAN_FMT_H_

#ifndef __ATTRIB_PACK
#define __ATTRIB_PACK __packed
#endif

#define MAX_INTERF 10 /* 5 categories x (lower + upper) bands */
#define MAX_NUM_BINS                  (520)
#define MAX_SPECTRAL_CHAINS           (3)
#define MAX_NUM_BINS_PRI80            (1024)
#define MAX_NUM_BINS_SEC80            (520)
#define MAX_NUM_BINS_5MHZ             (32)

/**
 * struct INTERF_RSP_INFO - Interference source info
 * @interf_min_freq: Interference minimum frequency
 * @interf_max_freq: Interference maximum frequency
 * @interf_type: Interference type
 */
struct INTERF_RSP_INFO {
	uint16_t interf_min_freq;
	uint16_t interf_max_freq;
	uint8_t  interf_type;
} __ATTRIB_PACK;

/**
 * struct INTERF_SRC_RSP_INFO - Interference source info
 * @count: number of interference sources
 * @interf: interference info
 */
struct INTERF_SRC_RSP_INFO {
	uint16_t count;
	struct INTERF_RSP_INFO interf[MAX_INTERF];
} __ATTRIB_PACK;

/**
 * struct samp_msg_data_info - FFT sampling data
 * @spectral_tstamp: timestamp
 * @spectral_last_tstamp: last time stamp
 * @ch_width: channel width
 * @spectral_data_len: bin size for the sampling data
 * @spectral_data_len_sec80: bin size for secondary 80Mhz segment
 * @spectral_rssi: RSSI value
 * @spectral_rssi_sec80: RSSI value for secondary 80Mhz segment
 * @spectral_bwinfo: bandwidth info
 * @spectral_max_index: index of max magnitude
 * @spectral_max_index_sec80: index of max magnitude for secondary 80Mhz segment
 * @spectral_max_mag: maximum magnitude
 * @spectral_max_mag_sec80: maximum magnitude for secondary 80Mhz segment
 * @spectral_upper_max_index: index of max magnitude in upper band
 * @spectral_lower_max_index: index of max magnitude in lower band
 * @bin_pwr_count: number of FFT bins
 * @bin_pwr_count_sec80: number of FFT bins in secondary 80MHz segment
 * @noise_floor: current noise floor
 * @noise_floor_sec80: current noise floor in secondary 80MHz segment
 * @spectral_max_exp: maximum exp
 * @spectral_combined_rssi: combined RSSI from all antennas
 * @spectral_upper_rssi: RSSI of upper band
 * @spectral_lower_rssi: RSSI of lower band
 * @spectral_chain_ctl_rssi: RSSI for control channel, for all antennas
 * @spectral_chain_ext_rssi: RSSI for extension channel, for all antennas
 * @spectral_max_scale: scale factor
 * @spectral_nb_upper: not used
 * @spectral_nb_lower: not used
 * @lb_edge_extrabins: number of extra bins on left band edge
 * @rb_edge_extrabins: number of extra bins on right band edge
 * @bin_pwr: FFT magnitudes
 * @bin_pwr_sec80: FFT magnitudes in secondary 80MHz segment
 * @interf_list: list of interference source
 */
struct samp_msg_data_info {
	int32_t     spectral_tstamp;
	int32_t     spectral_last_tstamp;
	uint32_t    ch_width;
	int16_t     spectral_data_len;
	int16_t     spectral_data_len_sec80;
	int16_t     spectral_rssi;
	int16_t     spectral_rssi_sec80;
	int16_t     spectral_bwinfo;
	int16_t     spectral_max_index;
	int16_t     spectral_max_index_sec80;
	int16_t     spectral_max_mag;
	int16_t     spectral_max_mag_sec80;
	int16_t     spectral_upper_max_index;
	int16_t     spectral_lower_max_index;
	uint16_t    bin_pwr_count;
	uint16_t    bin_pwr_count_sec80;
	int16_t     noise_floor;
	int16_t     noise_floor_sec80;
	uint8_t     spectral_max_exp;
	int8_t      spectral_combined_rssi;
	int8_t      spectral_upper_rssi;
	int8_t      spectral_lower_rssi;
	int8_t      spectral_chain_ctl_rssi[MAX_SPECTRAL_CHAINS];
	int8_t      spectral_chain_ext_rssi[MAX_SPECTRAL_CHAINS];
	uint8_t     spectral_max_scale;
	uint8_t     spectral_nb_upper;
	uint8_t     spectral_nb_lower;
	uint8_t     lb_edge_extrabins;
	uint8_t     rb_edge_extrabins;
	uint8_t     bin_pwr[MAX_NUM_BINS];
	uint8_t     bin_pwr_sec80[MAX_NUM_BINS];
	struct      INTERF_SRC_RSP_INFO interf_list;
} __ATTRIB_PACK;

#define SAMP_SIGNATURE 0xdeadbeef

/**
 * struct spectral_samp_msg_info - FFT sampling data
 * @signature: flag indicating it is a samp message (0xdeadbeef)
 * @freq: Operating frequency in MHz
 * @vhtop_ch_freq_seg1: VHT channel frequency segment 1
 * @vhtop_ch_freq_seg2: VHT channel frequency segment 2
 * @freq_loading: how busy was the channel
 * @dcs_enabled: if DCS is enabled
 * @int_type: DCS interface type, defined as above values 0, 1, 2
 * @macaddr: interface mac address
 * @samp_data: sampling data
 */
struct spectral_samp_msg_info {
	uint32_t      signature;
	uint16_t      freq;
	uint16_t      vhtop_ch_freq_seg1;
	uint16_t      vhtop_ch_freq_seg2;
	uint16_t      freq_loading;
	uint8_t       dcs_enabled;
	uint8_t       int_type;
	uint8_t       macaddr[6];
	struct samp_msg_data_info samp_data;
} __ATTRIB_PACK;

/**
 * enum spectral_scan_mode - Spectral scan mode
 * @SPECTRAL_SCAN_MODE_NORMAL: Normal mode
 * @SPECTRAL_SCAN_MODE_AGILE: Agile mode
 */
enum spectral_scan_mode {
	SPECTRAL_SCAN_MODE_NORMAL,
	SPECTRAL_SCAN_MODE_AGILE,
	SPECTRAL_SCAN_MODE_MAX,
};

/**
 * struct interference_rsp - Interference record
 * @interf_type:         giving type of interference
 * @interf_min_freq:     Minimum frequency in MHz at which interference has been
 * found
 * @interf_max_freq:     Maximum frequency in MHz at which interference has been
 * found
 */
struct interference_rsp {
	uint8_t interf_type;
	uint16_t interf_min_freq;
	uint16_t interf_max_freq;
} __ATTRIB_PACK;

/**
 * struct interf_src_rsp - List of interference sources
 * @count: Number of interference records
 * @interf: Array of interference records
 */
struct interf_src_rsp {
	uint16_t count;
	struct interference_rsp interf[MAX_INTERF];
} __ATTRIB_PACK;

/**
 * struct spectral_classifier_params - spectral classifier parameters
 * @spectral_20_40_mode:  Is AP in 20/40 mode?
 * @spectral_dc_index:    DC index
 * @spectral_dc_in_mhz:   DC in MHz
 * @upper_chan_in_mhz:    Upper channel in MHz
 * @lower_chan_in_mhz:    Lower channel in MHz
 */
struct spectral_classifier_params {
	int32_t spectral_20_40_mode;
	int32_t spectral_dc_index;
	int32_t spectral_dc_in_mhz;
	int32_t upper_chan_in_mhz;
	int32_t lower_chan_in_mhz;
} __ATTRIB_PACK;

/**
 * struct spectral_samp_data_info_umac_v0 - Spectral Analysis Messaging
 * Protocol Data format version 0
 * @spectral_data_len:        Indicates the bin size
 * @spectral_data_len_sec80:  Indicates the bin size for secondary 80 segment
 * @spectral_rssi:            Indicates RSSI
 * @spectral_rssi_sec80:      Indicates RSSI for secondary 80 segment
 * @spectral_combined_rssi:   Indicates combined RSSI from all antennas
 * @spectral_upper_rssi:      Indicates RSSI of upper band
 * @spectral_lower_rssi:      Indicates RSSI of lower band
 * @spectral_chain_ctl_rssi:  RSSI for control channel, for all antennas
 * @spectral_chain_ext_rssi:  RSSI for extension channel, for all antennas
 * @spectral_max_scale:       Indicates scale factor
 * @spectral_bwinfo:          Indicates bandwidth info
 * @spectral_tstamp:          Indicates timestamp
 * @spectral_max_index:       Indicates the index of max magnitude
 * @spectral_max_index_sec80: Indicates the index of max magnitude for secondary
 *                            80 segment
 * @spectral_max_mag:         Indicates the maximum magnitude
 * @spectral_max_mag_sec80:   Indicates the maximum magnitude for secondary 80
 *                            segment
 * @spectral_max_exp:         Indicates the max exp
 * @spectral_last_tstamp:     Indicates the last time stamp
 * @spectral_upper_max_index: Indicates the index of max mag in upper band
 * @spectral_lower_max_index: Indicates the index of max mag in lower band
 * @spectral_nb_upper:        Not Used
 * @spectral_nb_lower:        Not Used
 * @classifier_params:        Indicates classifier parameters
 * @bin_pwr_count:            Indicates the number of FFT bins
 * @lb_edge_extrabins:        Number of extra bins on left band edge
 * @rb_edge_extrabins:        Number of extra bins on right band edge
 * @bin_pwr_count_sec80:      Indicates the number of FFT bins in secondary 80
 *                            segment
 * @bin_pwr:                  Contains FFT magnitudes
 * @bin_pwr_sec80:            Contains FFT magnitudes for the secondary 80
 *                            segment
 * @interf_list:              List of interfernce sources
 * @noise_floor:              Indicates the current noise floor
 * @noise_floor_sec80:        Indicates the current noise floor for secondary 80
 *                            segment
 * @ch_width:                 Channel width 20/40/80/160 MHz
 * @spectral_mode:            Spectral scan mode
 */
struct spectral_samp_data_info_umac_v0 {
	int16_t spectral_data_len;
	int16_t spectral_data_len_sec80;
	int16_t spectral_rssi;
	int16_t spectral_rssi_sec80;
	int8_t spectral_combined_rssi;
	int8_t spectral_upper_rssi;
	int8_t spectral_lower_rssi;
	int8_t spectral_chain_ctl_rssi[MAX_SPECTRAL_CHAINS];
	int8_t spectral_chain_ext_rssi[MAX_SPECTRAL_CHAINS];
	uint8_t spectral_max_scale;
	int16_t spectral_bwinfo;
	int32_t spectral_tstamp;
	int16_t spectral_max_index;
	int16_t spectral_max_index_sec80;
	int16_t spectral_max_mag;
	int16_t spectral_max_mag_sec80;
	uint8_t spectral_max_exp;
	int32_t spectral_last_tstamp;
	int16_t spectral_upper_max_index;
	int16_t spectral_lower_max_index;
	uint8_t spectral_nb_upper;
	uint8_t spectral_nb_lower;
	struct spectral_classifier_params classifier_params;
	uint16_t bin_pwr_count;
	/*
	 * For 11ac chipsets prior to AR900B version 2.0, a max of 512 bins are
	 * delivered.  However, there can be additional bins reported for
	 * AR900B version 2.0 and QCA9984 as described next:
	 *
	 * AR900B version 2.0: An additional tone is processed on the right
	 * hand side in order to facilitate detection of radar pulses out to
	 * the extreme band-edge of the channel frequency.
	 * Since the HW design processes four tones at a time,
	 * this requires one additional Dword to be added to the
	 * search FFT report.
	 *
	 * QCA9984: When spectral_scan_rpt_mode=2, i.e 2-dword summary +
	 * 1x-oversampled bins (in-band) per FFT,
	 * then 8 more bins (4 more on left side and 4 more on right side)
	 * are added.
	 */
	uint8_t lb_edge_extrabins;
	uint8_t rb_edge_extrabins;
	uint16_t bin_pwr_count_sec80;
	uint8_t bin_pwr[MAX_NUM_BINS];
	uint8_t bin_pwr_sec80[MAX_NUM_BINS];
	struct interf_src_rsp interf_list;
	int16_t noise_floor;
	int16_t noise_floor_sec80;
	uint32_t ch_width;
	uint8_t spectral_agc_total_gain;
	uint8_t spectral_agc_total_gain_sec80;
	uint8_t spectral_gainchange;
	uint8_t spectral_gainchange_sec80;
	enum spectral_scan_mode spectral_mode;
} __ATTRIB_PACK;

/**
 * struct spectral_samp_msg_info_umac_v0 - Spectral SAMP message version 0
 * @signature:          Validates the SAMP message
 * @freq:               Operating frequency in MHz
 * @vhtop_ch_freq_seg1: VHT Segment 1 centre frequency in MHz
 * @vhtop_ch_freq_seg2: VHT Segment 2 centre frequency in MHz
 * @freq_loading:       How busy was the channel
 * @dcs_enabled:        Whether DCS is enabled
 * @int_type:           Interference type indicated by DCS
 * @macaddr:            Indicates the device interface
 * @samp_data:          SAMP Data
 */
struct spectral_samp_msg_info_umac_v0 {
	uint32_t signature;
	uint16_t freq;
	uint16_t vhtop_ch_freq_seg1;
	uint16_t vhtop_ch_freq_seg2;
	uint16_t freq_loading;
	uint16_t dcs_enabled;
	uint32_t int_type;
	uint8_t macaddr[6];
	struct spectral_samp_data_info_umac_v0 samp_data;
} __ATTRIB_PACK;

/**
 * enum dcs_int_type - Interference type indicated by DCS
 * @SPECTRAL_DCS_INT_NONE:  No interference
 * @SPECTRAL_DCS_INT_CW:  CW interference
 * @SPECTRAL_DCS_INT_WIFI:  WLAN interference
 */
enum dcs_int_type {
	SPECTRAL_DCS_INT_NONE,
	SPECTRAL_DCS_INT_CW,
	SPECTRAL_DCS_INT_WIFI
};

/**
 * struct spectral_samp_data_umac_v1 - Spectral Analysis Messaging Protocol Data format
 * @spectral_data_len:        Indicates the bin size
 * @spectral_data_len_sec80:  Indicates the bin size for secondary 80 segment
 * @spectral_rssi:            Indicates RSSI
 * @spectral_rssi_sec80:      Indicates RSSI for secondary 80 segment
 * @spectral_combined_rssi:   Indicates combined RSSI from all antennas
 * @spectral_upper_rssi:      Indicates RSSI of upper band
 * @spectral_lower_rssi:      Indicates RSSI of lower band
 * @spectral_chain_ctl_rssi:  RSSI for control channel, for all antennas
 * @spectral_chain_ext_rssi:  RSSI for extension channel, for all antennas
 * @spectral_max_scale:       Indicates scale factor
 * @spectral_bwinfo:          Indicates bandwidth info
 * @spectral_tstamp:          Indicates timestamp
 * @spectral_max_index:       Indicates the index of max magnitude
 * @spectral_max_index_sec80: Indicates the index of max magnitude for secondary
 *                            80 segment
 * @spectral_max_mag:         Indicates the maximum magnitude
 * @spectral_max_mag_sec80:   Indicates the maximum magnitude for secondary 80
 *                            segment
 * @spectral_max_exp:         Indicates the max exp
 * @spectral_last_tstamp:     Indicates the last time stamp
 * @spectral_upper_max_index: Indicates the index of max mag in upper band
 * @spectral_lower_max_index: Indicates the index of max mag in lower band
 * @spectral_nb_upper:        Not Used
 * @spectral_nb_lower:        Not Used
 * @classifier_params:        Indicates classifier parameters
 * @bin_pwr_count:            Indicates the number of FFT bins
 * @lb_edge_extrabins:        Number of extra bins on left band edge
 * @rb_edge_extrabins:        Number of extra bins on right band edge
 * @bin_pwr_count_sec80:      Indicates the number of FFT bins in secondary 80
 *                            segment
 * @bin_pwr:                  Contains FFT magnitudes
 * @bin_pwr_sec80:            Contains FFT magnitudes for the secondary 80
 *                            segment
 * @interf_list:              List of interfernce sources
 * @noise_floor:              Indicates the current noise floor
 * @noise_floor_sec80:        Indicates the current noise floor for secondary 80
 *                            segment
 * @ch_width:                 Channel width 20/40/80/160 MHz
 * @spectral_mode:            Spectral scan mode
 * @spectral_pri80ind:        Indication from hardware that the sample was
 *                            received on the primary 80 MHz segment. If this
 *                            is set when smode = SPECTRAL_SCAN_MODE_AGILE, it
 *                            indicates that Spectral was carried out on pri80
 *                            instead of the Agile frequency due to a
 *                            channel switch - Software may choose
 *                            to ignore the sample in this case.
 * @spectral_pri80ind_sec80:  Indication from hardware that the sample was
 *                            received on the primary 80 MHz segment instead of
 *                            the secondary 80 MHz segment due to a channel
 *                            switch - Software may choose to ignore the sample
 *                            if this is set. Applicable only if smode =
 *                            SPECTRAL_SCAN_MODE_NORMAL and for 160/80+80 MHz
 *                            Spectral operation.
 * @last_raw_timestamp:       Previous FFT report's raw timestamp. In case of
 *                            160Mhz it will be primary 80 segment's timestamp
 *                            as both primary & secondary segment's timestamp
 *                            are expected to be almost equal.
 * @timestamp_war_offset:     Offset calculated based on reset_delay and
 *                            last_raw_timestamp. It will be added to
 *                            raw_timestamp to get spectral_tstamp.
 * @raw_timestamp:            Actual FFT timestamp reported by HW on primary
 *                            segment.
 * @raw_timestamp_sec80:      Actual FFT timestamp reported by HW on sec80 MHz
 *                            segment.
 * @reset_delay:              Time gap between the last spectral report before
 *                            reset and the end of reset. It is provided by FW
 *                            via direct DMA framework.
 * @target_reset_count:       Indicates the number of times target went through
 *                            reset routine after spectral was enabled.
 * @bin_pwr_count_5mhz:       Indicates the number of FFT bins in the extra
 *                            5 MHz for 165 MHz/ Restricted 80p80 mode
 * @bin_pwr_5mhz:             Contains FFT magnitudes corresponding to the extra
 *                            5 MHz in 165 MHz/ Restricted 80p80 mode
 */
struct spectral_samp_data_umac_v1 {
	int16_t spectral_data_len;
	int16_t spectral_data_len_sec80;
	int16_t spectral_rssi;
	int16_t spectral_rssi_sec80;
	int8_t spectral_combined_rssi;
	int8_t spectral_upper_rssi;
	int8_t spectral_lower_rssi;
	int8_t spectral_chain_ctl_rssi[MAX_SPECTRAL_CHAINS];
	int8_t spectral_chain_ext_rssi[MAX_SPECTRAL_CHAINS];
	uint8_t spectral_max_scale;
	int16_t spectral_bwinfo;
	int32_t spectral_tstamp;
	int16_t spectral_max_index;
	int16_t spectral_max_index_sec80;
	int16_t spectral_max_mag;
	int16_t spectral_max_mag_sec80;
	uint8_t spectral_max_exp;
	int32_t spectral_last_tstamp;
	int16_t spectral_upper_max_index;
	int16_t spectral_lower_max_index;
	uint8_t spectral_nb_upper;
	uint8_t spectral_nb_lower;
	struct spectral_classifier_params classifier_params;
	uint16_t bin_pwr_count;
	/*
	 * For 11ac chipsets prior to AR900B version 2.0, a max of 512 bins are
	 * delivered.  However, there can be additional bins reported for
	 * AR900B version 2.0 and QCA9984 as described next:
	 *
	 * AR900B version 2.0: An additional tone is processed on the right
	 * hand side in order to facilitate detection of radar pulses out to
	 * the extreme band-edge of the channel frequency.
	 * Since the HW design processes four tones at a time,
	 * this requires one additional Dword to be added to the
	 * search FFT report.
	 *
	 * QCA9984: When spectral_scan_rpt_mode=2, i.e 2-dword summary +
	 * 1x-oversampled bins (in-band) per FFT,
	 * then 8 more bins (4 more on left side and 4 more on right side)
	 * are added.
	 */
	uint8_t lb_edge_extrabins;
	uint8_t rb_edge_extrabins;
	uint16_t bin_pwr_count_sec80;
	uint8_t bin_pwr[MAX_NUM_BINS_PRI80];
	uint8_t bin_pwr_sec80[MAX_NUM_BINS_SEC80];
	struct interf_src_rsp interf_list;
	int16_t noise_floor;
	int16_t noise_floor_sec80;
	uint32_t ch_width;
	uint8_t spectral_agc_total_gain;
	uint8_t spectral_agc_total_gain_sec80;
	uint8_t spectral_gainchange;
	uint8_t spectral_gainchange_sec80;
	enum spectral_scan_mode spectral_mode;
	uint8_t spectral_pri80ind;
	uint8_t spectral_pri80ind_sec80;
	uint32_t last_raw_timestamp;
	uint32_t timestamp_war_offset;
	uint32_t raw_timestamp;
	uint32_t raw_timestamp_sec80;
	uint32_t reset_delay;
	uint32_t target_reset_count;
	uint32_t agile_ch_width;
	uint16_t bin_pwr_count_5mhz;
	uint8_t bin_pwr_5mhz[MAX_NUM_BINS_5MHZ];
} __ATTRIB_PACK;

/**
 * struct spectral_samp_msg_info_umac_v1 - Spectral SAMP message version 1
 * @signature:          Validates the SAMP message
 * @freq:               Operating frequency in MHz
 * @vhtop_ch_freq_seg1: VHT Segment 1 centre frequency in MHz
 * @vhtop_ch_freq_seg2: VHT Segment 2 centre frequency in MHz
 * @agile_freq1:        Center frequency in MHz of the entire span(for 80+80 MHz
 *                      agile Scan it is primary 80 MHz span) across which
 *                      Agile Spectral is carried out. Applicable only for Agile
 *                      Spectral samples.
 * @agile_freq2:        Center frequency in MHz of the secondary 80 MHz span
 *                      across which Agile Spectral is carried out. Applicable
 *                      only for Agile Spectral samples in 80+80 MHz mode.
 * @freq_loading:       How busy was the channel
 * @dcs_enabled:        Whether DCS is enabled
 * @int_type:           Interference type indicated by DCS
 * @macaddr:            Indicates the device interface
 * @samp_data:          SAMP Data
 */
struct spectral_samp_msg_info_umac_v1 {
	uint32_t signature;
	uint16_t freq;
	uint16_t vhtop_ch_freq_seg1;
	uint16_t vhtop_ch_freq_seg2;
	uint16_t agile_freq1;
	uint16_t agile_freq2;
	uint16_t freq_loading;
	uint16_t dcs_enabled;
	enum dcs_int_type int_type;
	uint8_t macaddr[6];
	struct spectral_samp_data_umac_v1 samp_data;
} __ATTRIB_PACK;

/**
 * struct spectral_samp_msg_header - header of Spectral SAMP message
 * @signature:          Validates the SAMP message
 */
struct spectral_samp_msg_header {
        uint32_t signature;
};
#endif /* _SPECTRAL_SCAN_FMT_H_ */
