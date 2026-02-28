# Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: GPL-2.0-only

import traceback
from print_out import print_out_str
from parser_util import register_parser, RamParser
import json
import pprint

@register_parser(
    '--bcl-info', 'Useful information from BCL data structures')
class BCL_info(RamParser):
    def __init__(self, dump):
        super(BCL_info, self).__init__(dump)

    def write_to_text(self,device_stats_dict):
        formatted_data = json.dumps(device_stats_dict,indent=2)
        self.writeln(f"{formatted_data}")

    def write_to_json(self,device_stats_dict):
        #Write the dictionary to a json file
        with self.ramdump.open_file('bcl_info/bcl_peripheral_stats.json',"w") as json_file:   #json_file is a file object that will close after the with open(...)
            json.dump(device_stats_dict,json_file,indent=2)

    def write(self, string):
        self.out.write(string)

    def writeln(self, string=""):
        self.out.write(string + '\n')
        self.out.flush()

    def parse(self):
        print_out_str("Started bcl parsing")
        self.out = self.ramdump.open_file('bcl_info/bcl_stats.txt', "w")
        try:
            self.write("BCL DATA".center(90, '-') + '\n')
            self.writeln()
            cnt = self.ramdump.read_int("bcl_device_ct")
            self.writeln(f"bcl_device_cnt:{cnt}")

            BCL_STATS_DATA = {"bcl_devices":[]}

            for i in range(cnt):
                bcl_ptr = self.ramdump.read_pointer(f"bcl_devices[{i}]")

                if not bcl_ptr:
                    self.writeln(f"bcl_devices[{i}] is NULL")
                    continue

                self.writeln(f"bcl_devices_address:({hex(bcl_ptr)})")
                self.writeln()

                try:
                    bcl_device = self.ramdump.read_datatype(bcl_ptr,"struct bcl_device")
                    if bcl_device is None:
                        raise ValueError(f"Failed to read struct device at address: {hex(bcl_ptr)}")
                except Exception as e:
                    self.writeln(f"Error reading struct device at address {hex(bcl_ptr)}")
                    raise ValueError(f"Failed to read struct device at address: {hex(bcl_ptr)}")

                try:
                    self.writeln(f"struct bcl_device: {hex(bcl_ptr)} [{i}]")
                    pprint.pprint(bcl_device.__dict__, indent=2, stream=self.out)
                    self.writeln()
                except:
                    self.writeln("Failed to dump bcl_device data")
                    self.writeln()

                stats_data_enabled = False
                bpm_data_enabled = False

                try:
                    # if stats pointer not present, return
                    _stats_data = bcl_device.stats
                    if not _stats_data:
                        self.writeln(f"bcl_devices[{i}] dont have stats data")
                        self.writeln()
                        pass
                    stats_data_enabled = True
                except:
                    self.writeln(f"bcl_devices[{i}] dont have stats data")
                    self.writeln()
                    pass

                bpm_stats = dict(
                    last_bpm_read_ts=None,
                    last_bpm_reset_ts=None,
                    max_ibat=None,
                    sync_vbat=None,
                    max_vbat=None,
                    sync_ibat=None,
                    lvl0_cnt=None,
                    lvl1_cnt=None,
                    lvl2_cnt=None
                )

                try:
                    _bpm_stats = bcl_device.bpm_stats
                    if not _bpm_stats:
                        self.writeln(f"bcl_devices[{i}] dont have BPM stats data")
                        self.writeln()
                        pass
                    else:
                        # if stats pointer not present, return
                        bpm_stats = dict(
                            last_bpm_read_ts=bcl_device.last_bpm_read_ts,
                            last_bpm_reset_ts=bcl_device.last_bpm_reset_ts,
                            max_ibat=_bpm_stats.max_ibat,
                            sync_vbat=_bpm_stats.sync_vbat,
                            max_vbat=_bpm_stats.min_vbat,
                            sync_ibat=_bpm_stats.sync_ibat,
                            lvl0_cnt=_bpm_stats.lvl0_cnt,
                            lvl1_cnt=_bpm_stats.lvl1_cnt,
                            lvl2_cnt=_bpm_stats.lvl2_cnt
                        )
                        bpm_data_enabled = True
                        try:
                            self.writeln("struct bcl_bpm:")
                            pprint.pprint(bcl_device.bpm_stats.__dict__, indent=2, stream=self.out)
                            self.writeln()
                        except:
                            self.writeln("Failed to dump struct bcl_bpm data")
                            self.writeln()
                except:
                    self.writeln(f"bcl_devices[{i}] dont have BPM stats data")
                    self.writeln()
                    pass

                if not stats_data_enabled and not bpm_data_enabled:
                    continue

                bcl_addr = hex(bcl_device.fg_bcl_addr)
                bcl_device_dict = {
                                   "name": f"bcl_{bcl_addr}_{i}",
                                   "max_ibat":bpm_stats.get("max_ibat"),
                                   "sync_vbat":bpm_stats.get("sync_vbat"),
                                   "min_vbat":bpm_stats.get("max_vbat"),
                                   "sync_ibat":bpm_stats.get("sync_ibat"),
                                   "last_bpm_read_ts":bpm_stats.get("last_bpm_read_ts"),
                                   "last_bpm_reset_ts":bpm_stats.get("last_bpm_reset_ts"),
                                   "stats":[]
                }

                #Iterate over stats array in the device
                try:
                    for j, stat_ptr in enumerate(bcl_device.stats):
                        if stat_ptr:
                            try:
                                stats_data = {
                                    "bcl_level": j,
                                    "counter": bcl_device.stats[j].counter,
                                    "self_cleared_counter": bcl_device.stats[j].self_cleared_counter,
                                    "trigger_state": bcl_device.stats[j].trigger_state,
                                    "max_mitig_ts(nsec)": bcl_device.stats[j].max_mitig_ts,
                                    "max_mitig_latency(usec)": bcl_device.stats[j].max_mitig_latency,
                                    "max_duration(usec)": bcl_device.stats[j].max_duration,
                                    "total_duration(usec)": bcl_device.stats[j].total_duration,
                                    "hw_counter": bpm_stats.get(f"lvl{j}_cnt"),
                                    "bcl_history": []
                                }

                                for k, history in enumerate(bcl_device.stats[j].bcl_history):
                                    try:
                                        stats_data["bcl_history"].append({
                                            "vbat(mV)": history.vbat,
                                            "ibat(mA)": history.ibat,
                                            "trigger_ts(nsec)": history.trigger_ts,
                                            "clear_ts(nsec)": history.clear_ts
                                        })
                                    except Exception as e:
                                        self.writeln(f"Failed to process bcl_history[{k}] in stats[{j}]: {e}")
                                bcl_device_dict["stats"].append(stats_data)
                            except Exception as e:
                                self.writeln(f"Failed to process stats[{j}]: {e}")
                except Exception as e:
                    self.writeln(f"Error occured while iterating over device.stats: {e}")

                BCL_STATS_DATA["bcl_devices"].append(bcl_device_dict)

            if BCL_STATS_DATA:
                # self.write_to_text(device_stats_dict)
                self.write_to_text(BCL_STATS_DATA)
                self.write_to_json(BCL_STATS_DATA)
        except Exception as e:
            self.writeln(traceback.format_exc())

        # close text file
        self.out.close()
        print_out_str("Done bcl parsing")
