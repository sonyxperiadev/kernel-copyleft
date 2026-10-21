# Copyright (c) 2015, 2020-2021 The Linux Foundation. All rights reserved.
# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from print_out import print_out_str
from parser_util import register_parser, RamParser
import linux_list as llist
import traceback

TSENS_MAX_SENSORS = 16
DEBUG_SIZE = 10
THERMAL_MAX_TRIPS = 12

@register_parser(
    '--thermal-info', 'Useful information from thermal data structures')
class Thermal_info(RamParser):
    def __init__(self, dump):
        super(Thermal_info, self).__init__(dump)
        # List of all sub-parsers as (func, info, outfile, mode) tuples.
        self.parser_list_5_10 = [
            (self.parse_thremal_zone_data, "ThermalZone", 'thermal_zone.txt', "w"),
            (self.parse_cooling_device_data, "CoolingDevice", 'cooling_device.txt', "w"),
        ]

        self.parser_list_5_4 = [
            (self.parse_thremal_zone_data, "ThermalZone", 'thermal_zone.txt', "w"),
            (self.parse_cooling_device_data, "CoolingDevice", 'cooling_device.txt', "w"),
            (self.parse_tsen_device_data, "Tsensors", 'tsens_info.txt', "w"),
        ]

        self.parser_list_v1 = [
            (self.tmdev_data, "TsensDbgData", 'tsens_dbg_info.txt', "w"),
        ]

    def print_thermal_info(
            self, sensor_dbg_info_start_address, ram_dump,
            time_stamp, sensor_mapping):
        for ncpu in ram_dump.iter_cpus():
            self.writeln(
                "------------------------------------------------")
            self.writeln(
                " TEMPERATURE ENTRIES FOR CPU:{0}".format(
                    int(ncpu)))
            self.writeln(
                "------------------------------------------------")
            cpu_sensor_addr = sensor_dbg_info_start_address + \
                sensor_mapping[ncpu]
            for i in range(0, 10):
                temp = self.ramdump.read_word(cpu_sensor_addr + (i * 8), True)
                time = self.ramdump.read_word(
                    cpu_sensor_addr + time_stamp + (i * 8), True)
                self.writeln(
                    "Temperature reading -  {0} ".format(int(temp)))
                self.writeln("TimeStamp - {0}\n".format(int(time)))

    def tmdev_data(self, ram_dump):
        sensor_mapping = []
        self.writeln("Thermal sensor data \n")

        tmdev = self.ramdump.address_of('tmdev')
        tmdev_address = self.ramdump.read_word(tmdev, True)
        sensor_dbg_info_size = ram_dump.sizeof('struct tsens_sensor_dbg_info')
        sensor_dbg_info = self.ramdump.field_offset(
            'struct tsens_tm_device',
            'sensor_dbg_info')
        time_stamp = self.ramdump.field_offset(
            'struct tsens_sensor_dbg_info',
            'time_stmp')
        cpus_sensor = self.ramdump.address_of('cpus')
        cpus_sensor_size = ram_dump.sizeof('struct cpu_info')
        sensor_id_offset = self.ramdump.field_offset(
            'struct cpu_info',
            'sensor_id')

        if not all((tmdev, sensor_dbg_info_size, sensor_dbg_info,
                    time_stamp, cpus_sensor, cpus_sensor_size,
                    sensor_id_offset)):
            self.writeln("Not supported for this target yet  :-( \n")
            return

        for i in ram_dump.iter_cpus():
            cpu_sensor_id_address = cpus_sensor + sensor_id_offset
            sensor_id = self.ramdump.read_u32(cpu_sensor_id_address, True)
            cpus_sensor = cpus_sensor + cpus_sensor_size
            sensor_mapping.append((sensor_id - 1) * sensor_dbg_info_size)

        self.print_thermal_info(
            (tmdev_address + sensor_dbg_info),
            ram_dump,
            time_stamp,
            sensor_mapping)

    def tsens_dbg_parse_fields(self, tsens_device_p):
        dev_o = self.ramdump.field_offset('struct tsens_device', 'dev')
        dev = self.ramdump.read_word(dev_o + tsens_device_p)
        kobj_o = self.ramdump.field_offset('struct device', ' kobj')
        kobj = (dev + kobj_o)
        name_o = self.ramdump.field_offset('struct kobject', 'name')
        name_addr = self.ramdump.read_word(name_o + kobj)
        name = self.ramdump.read_cstring(name_addr)
        if name is not None:
            self.writeln("%s" % (name))
            tsens_dbg_o = self.ramdump.field_offset('struct tsens_device', 'tsens_dbg')
            tsens_dbg = tsens_device_p + tsens_dbg_o
            sensor_dbg_info_o = self.ramdump.field_offset('struct tsens_dbg_context', 'sensor_dbg_info')
            sensor_dbg_info = sensor_dbg_info_o  + tsens_dbg
            self.writeln('v.v (struct tsens_device)0x{:8x} 0x{:8x}\n'.format(tsens_device_p,
                                                                                       sensor_dbg_info))
            for i in range(0, TSENS_MAX_SENSORS):
                idx = self.ramdump.read_u32(self.ramdump.array_index(sensor_dbg_info, 'struct tsens_dbg', i))
                tsens_dbg_addr = self.ramdump.array_index(sensor_dbg_info, 'struct tsens_dbg', i)
                self.writeln ("    idx: %d tsens_dbg_addr 0x%x" %(idx, tsens_dbg_addr))
                time_stmp_o = self.ramdump.field_offset('struct tsens_dbg', 'time_stmp')
                temp_o = self.ramdump.field_offset('struct tsens_dbg', 'temp')
                self.writeln("             time_stmp       temp ")
                for j in range(0, DEBUG_SIZE):
                    time_stmp = self.ramdump.read_word(self.ramdump.array_index(time_stmp_o + tsens_dbg_addr,
                                                                                'unsigned long long', j))
                    temp = self.ramdump.read_u64(
                    self.ramdump.array_index(temp_o + tsens_dbg_addr, 'unsigned long', j))
                    self.writeln("             %d   %d" % (time_stmp, temp))

    def parse_cooling_device_fields(self, cdev_struct_addr, thermal_cdev_dict:dict):
        cdev_data_struct = {}
        cdev_id = None
        try:
            if not cdev_struct_addr:
                return

            cdev_data_struct["cdev_struct_addr"] = cdev_struct_addr
            cdev_id = cdev_data_struct["cdev_id"] = self.ramdump.read_structure_field(cdev_struct_addr,
                                                        'struct thermal_cooling_device', 'id')
            type_off = self.ramdump.field_offset('struct thermal_cooling_device', 'type')
            cdev_data_struct["cdev_type"] = self.ramdump.read_cstring(cdev_struct_addr + type_off)
            if not cdev_data_struct["cdev_type"]:
                return

            cdev_data_struct["updated"] = self.ramdump.read_bool(
                self.ramdump.struct_field_addr(cdev_struct_addr, 'struct thermal_cooling_device', 'updated'))
            stats_addr = cdev_data_struct["stats_addr"] = self.ramdump.read_structure_field(cdev_struct_addr,
                                                           'struct thermal_cooling_device', 'stats')

            cdev_data_struct["devdata"] = self.ramdump.struct_field_addr(cdev_struct_addr,
                                                     'struct thermal_cooling_device', 'devdata')
            if stats_addr:
                cdev_data_struct["stats_state"] = self.ramdump.read_structure_field(stats_addr,
                                                                'struct cooling_dev_stats', 'state')
                cdev_data_struct["stats_total_trans"] = self.ramdump.read_structure_field(stats_addr,
                                                                      'struct cooling_dev_stats',
                                                                      'total_trans')
                cdev_data_struct["stats_last_time"] = self.ramdump.read_structure_field(stats_addr,
                                                                    'struct cooling_dev_stats',
                                                                    'last_time')
                cdev_data_struct["max_states"] = self.ramdump.read_structure_field(stats_addr,
                                                                    'struct cooling_dev_stats',
                                                                    'max_states')
        except Exception as e:
            cdev_data_struct["exception"] = str(e)

        if cdev_data_struct and cdev_id is not None:
            thermal_cdev_dict[cdev_id] = cdev_data_struct
        return

    def parse_thermal_zone_fields(self, tz_device_addr, thermal_tzone_dict:list, triggered_zones:list):
        tzone_data_dict = {}
        tzone_id = None
        try:
            if not tz_device_addr:
                return

            tzone_data_dict["tz_device_addr"] = hex(tz_device_addr)
            tzone_id = tzone_data_dict["tzone_id"] = self.ramdump.read_structure_field(tz_device_addr,
                                                            'struct thermal_zone_device', 'id')
            tzone_data_dict["mode"] = self.ramdump.read_structure_field(tz_device_addr,
                                                          'struct thermal_zone_device', 'mode')
            if tzone_data_dict["mode"] not in [0, 1]:
                return
            type_off = self.ramdump.field_offset('struct thermal_zone_device', 'type')
            tzone_data_dict["type"] = self.ramdump.read_cstring(tz_device_addr+type_off)
            algo_type_off = self.ramdump.field_offset('struct thermal_zone_device', 'governor')
            tzone_data_dict["algo_type"] = self.ramdump.read_structure_cstring(tz_device_addr + algo_type_off,
                                                            'struct thermal_governor', 'name')
            tzone_data_dict["temperature"] = self.ramdump.read_s32(tz_device_addr +
                        self.ramdump.field_offset('struct thermal_zone_device', 'temperature'))
            tzone_data_dict["last_temperature"] = self.ramdump.read_s32(tz_device_addr +
                        self.ramdump.field_offset('struct thermal_zone_device', 'last_temperature'))
            tzone_data_dict["polling_delay"] = self.ramdump.read_structure_field(tz_device_addr,
                                                          'struct thermal_zone_device', 'polling_delay')
            tzone_data_dict["passive_delay"] = self.ramdump.read_structure_field(tz_device_addr,
                                                          'struct thermal_zone_device', 'passive_delay')
            tzone_data_dict["passive"] = self.ramdump.read_structure_field(tz_device_addr,
                                                          'struct thermal_zone_device', 'passive')
            tzone_data_dict["prev_low_trip"] = self.ramdump.read_s32(tz_device_addr +
                        self.ramdump.field_offset('struct thermal_zone_device', 'prev_low_trip'))
            tzone_data_dict["prev_high_trip"] = self.ramdump.read_s32(tz_device_addr +
                        self.ramdump.field_offset('struct thermal_zone_device', 'prev_high_trip'))
            tzone_data_dict["trip_count"] = self.ramdump.read_structure_field(tz_device_addr,
                                                          'struct thermal_zone_device', 'trips')
            tzone_data_dict["trips_disabled"] = self.ramdump.read_structure_field(tz_device_addr,
                                                          'struct thermal_zone_device', 'trips_disabled')
            devdata_off = self.ramdump.field_offset('struct thermal_zone_device', 'devdata')
            if devdata_off:
                devdata_off = hex(tz_device_addr+devdata_off)
            tzone_data_dict["devdata"] = devdata_off

            node_addr = self.ramdump.struct_field_addr(tz_device_addr,
                                                       "struct thermal_zone_device",
                                                       "thermal_instances")
            list_offset = self.ramdump.field_offset('struct thermal_instance', 'tz_node')
            device_list_walker = llist.ListWalker(self.ramdump, node_addr, list_offset)
            tzone_data_dict["trips_data"] = trips_data = {}
            trip_triggered = False
            last_node = None
            if device_list_walker.is_empty():
                pass
            else :
                while not device_list_walker.is_empty():
                    try:
                        thermal_instance_addr = device_list_walker.next()
                    except Exception as e:
                        break
                    trip = self.ramdump.read_structure_field(thermal_instance_addr,
                                                        'struct thermal_instance', 'trip')
                    if trip > THERMAL_MAX_TRIPS:
                        return
                    if trip not in trips_data:
                        trips_data[trip] = []

                    _trip_data = {}
                    _trip_data["id"] = self.ramdump.read_structure_field(thermal_instance_addr,
                                                                                  'struct thermal_instance',
                                                                                  'id')
                    instance_name_addr = self.ramdump.struct_field_addr(thermal_instance_addr,
                                                                           'struct thermal_instance', 'name')
                    _trip_data["name"] = self.ramdump.read_cstring(instance_name_addr)
                    _trip_data["initialized"] = self.ramdump.read_bool(
                        self.ramdump.struct_field_addr(thermal_instance_addr,
                                                          'struct thermal_instance', 'initialized'))
                    _trip_data["lower"] = self.ramdump.read_structure_field(thermal_instance_addr,
                                                                    'struct thermal_instance', 'lower')
                    _trip_data["upper"] = self.ramdump.read_structure_field(thermal_instance_addr,
                                                                    'struct thermal_instance', 'upper')
                    _trip_data["target"] = self.ramdump.read_structure_field(thermal_instance_addr,
                                                                    'struct thermal_instance', 'target')
                    if _trip_data["target"] < 0xFFFFFF:
                        trip_triggered = True

                    _trip_data["cdev"] = {}
                    cdev_addr = self.ramdump.read_structure_field(thermal_instance_addr,
                                                                  'struct thermal_instance', 'cdev')
                    self.parse_cooling_device_fields(cdev_addr, _trip_data["cdev"])

                    trips_data[trip].append(_trip_data)

                if  trip_triggered:
                    triggered_zones.append(tzone_data_dict["type"])

        except Exception as e:
            tzone_data_dict["exception"] = str(e)

        if tzone_data_dict and tzone_id is not None:
            thermal_tzone_dict.append(tzone_data_dict)
        return

    def parse_thremal_zone_data(self, dump):
        self.tzone_struct_list = []
        self.triggered_zones = []
        # thermal_zone data
        thermal_tz_list = self.ramdump.read('thermal_tz_list.next')
        list_offset = self.ramdump.field_offset('struct thermal_zone_device', 'node')
        list_walker = llist.ListWalker(self.ramdump, thermal_tz_list, list_offset)
        list_walker.walk(thermal_tz_list, self.parse_thermal_zone_fields,
                         self.tzone_struct_list, self.triggered_zones)
        if len(self.tzone_struct_list) == 0:
            self.writeln("No thermal Zones or exception in parsing")
            return

        self.writeln("")
        self.writeln("Total Tzones: {0}".format(len(self.tzone_struct_list)))
        self.writeln("Trip violated Tzones: {0}".format(",".join(self.triggered_zones)))
        self.writeln("")
        format_str = "{0:<35} {1}"
        self.tzone_struct_list.sort(key=lambda x: float(x["temperature"]), reverse=True)
        for tzone_struct in self.tzone_struct_list:
            self.writeln("")
            self.writeln("[THERMAL_ZONE_{0}]".format(tzone_struct["tzone_id"]))
            self.writeln(format_str.format("algo_type", tzone_struct.get("algo_type")))
            self.writeln(format_str.format("sensor", tzone_struct.get("type")))

            if "exception" in tzone_struct.keys():
                self.writeln(format_str.format("Exception", tzone_struct.get("exception")))
                continue

            self.writeln(format_str.format("mode", "enabled" if (tzone_struct.get("mode") == 1) else "disabled"))
            self.writeln(format_str.format("polling_delay", tzone_struct.get("polling_delay")))
            self.writeln(format_str.format("passive_delay", tzone_struct.get("passive_delay")))
            self.writeln(format_str.format("temperature", tzone_struct.get("temperature")))
            self.writeln(format_str.format("last_temperature", tzone_struct.get("last_temperature")))
            self.writeln(format_str.format("prev_high_trip", tzone_struct.get("prev_high_trip")))
            self.writeln(format_str.format("prev_low_trip", tzone_struct.get("prev_low_trip")))
            self.writeln(format_str.format("trip_cnt", tzone_struct.get("trip_count")))
            self.writeln(format_str.format("trips_disabled", tzone_struct.get("trips_disabled")))
            self.writeln(format_str.format("tzone_data_struct",
                                           "v.v ((struct thermal_zone_device *){0}".format(
                                               tzone_struct.get("tz_device_addr"))))
            self.writeln(format_str.format("tzone devdata ",
                                           "v.v ((struct __thermal_zone *){0}".format(
                                               tzone_struct.get("devdata"))))

            # print trip data
            self.writeln("Devices:")
            cdev_format_str = "\t\t{0:<35} {1}"
            trips_data = tzone_struct.get("trips_data")
            if not trips_data:
                self.writeln("\tNo Cooling Devices")
                continue
            trip_nums = sorted(trips_data.keys())
            for trip_num in trip_nums:
                trip_cdevs_info = trips_data[trip_num]
                self.writeln("  Trip{0}:".format(trip_num))
                for trip_info in trip_cdevs_info:
                    cdev_dict = trip_info.get("cdev")
                    if not cdev_dict:
                        self.writeln("Invalid cdev for trip instance")
                        continue
                    cdev_id = list(cdev_dict.keys())[0]
                    cdev_dict = cdev_dict[cdev_id]
                    self.writeln("\t {0}".format(cdev_dict.get("cdev_type")))
                    self.writeln(cdev_format_str.format("id", cdev_id))
                    self.writeln(cdev_format_str.format("initialized", trip_info.get("initialized")))
                    self.writeln(cdev_format_str.format("state(lower, upper, target)",
                                                        "({0}, {1}, {2})".format(
                                                            trip_info.get("lower"),
                                                            trip_info.get("upper"),
                                                            trip_info.get("target"))))

                    stats_data = cdev_dict.get("stats_addr")
                    stats_state = "NA"
                    if stats_data:
                        stats_state = cdev_dict.get("stats_state")
                    self.writeln(cdev_format_str.format("status(updated, stats_state)",
                                                        "({0}, {1})".format(cdev_dict.get("updated"), stats_state)))
                    if stats_data:
                        self.writeln(cdev_format_str.format("stats_total_trans",
                                                            cdev_dict.get("stats_total_trans")))
                        self.writeln(cdev_format_str.format("stats_last_time",
                                                            hex(cdev_dict.get("stats_last_time"))))
                    self.writeln("")

        return

    def parse_cooling_device_data(self, dump):
        self.cdev_struct_list = {}
        # thermal_zone data
        thermal_cdev_list = self.ramdump.read('thermal_cdev_list.next')
        list_offset = self.ramdump.field_offset('struct thermal_cooling_device', 'node')
        list_walker = llist.ListWalker(self.ramdump, thermal_cdev_list, list_offset)
        list_walker.walk(thermal_cdev_list, self.parse_cooling_device_fields, self.cdev_struct_list)
        cdev_ids_list = self.cdev_struct_list.keys()
        if not cdev_ids_list:
            print_out_str("No cooling Devices or exception in parsing")
            return

        format_str = "{0:<35} {1}"
        cdev_ids_list = sorted(cdev_ids_list)
        for cdev_id in cdev_ids_list:
            cdev_struct = self.cdev_struct_list[cdev_id]
            self.writeln("")
            self.writeln("[COOLING_DEVICE_{0}]".format(cdev_id))
            self.writeln(format_str.format("type", cdev_struct.get("cdev_type")))
            self.writeln(format_str.format("updated", cdev_struct.get("updated")))
            stats_addr = cdev_struct.get("stats_addr")
            if stats_addr:
                self.writeln(format_str.format("state", cdev_struct.get("stats_state")))
                self.writeln(format_str.format("total_trans", cdev_struct.get("stats_total_trans")))
                self.writeln(format_str.format("last_time", hex(cdev_struct.get("stats_last_time"))))
                self.writeln(format_str.format("cdev_max_states", cdev_struct.get("max_states")))

            self.writeln(format_str.format("cdev_struct",
                                           "v.v (struct thermal_cooling_device*){0}".format(
                                               hex(cdev_struct.get("cdev_struct_addr")))))
            self.writeln(format_str.format("cdev_devdata",
                                           "v.v (struct *){0}".format(hex(cdev_struct.get("devdata")))))
            self.writeln(format_str.format("cdev_stats_struct",
                                           "v.v (struct cooling_dev_stats*){0}".format(hex(stats_addr))))

            if "exception" in cdev_struct.keys():
                self.writeln(format_str.format("Exception", cdev_struct.get("exception")))

        return

    def parse_tsen_device_data(self, dump):
        tsens_device_list = self.ramdump.address_of('tsens_device_list')
        list_offset = self.ramdump.field_offset('struct tsens_device', 'list')
        list_walker = llist.ListWalker(self.ramdump, tsens_device_list, list_offset)
        list_walker.walk(tsens_device_list, self.tsens_dbg_parse_fields)

    def write(self, string):
        self.out.write(string)

    def writeln(self, string=""):
        self.out.write(string + '\n')

    def parse(self):
        print_out_str("Started thermal parsing")
        kv = self.ramdump.kernel_version
        if (kv[0], kv[1]) == (5, 4):
            self.parser_list = self.parser_list_5_4
        elif (kv[0], kv[1]) > (5, 4):
            self.parser_list = self.parser_list_5_10
        else:
            self.parser_list = self.parser_list_v1

        for subparser in self.parser_list:
            try:
                self.out = self.ramdump.open_file('thermal_info/' + subparser[2], subparser[3])
                self.write(subparser[1].center(90, '-') + '\n')
                subparser[0](self.ramdump)
                self.writeln()
                self.out.close()
            except Exception as e:
                if self.out:
                    self.writeln(str(e))
                    self.out.close()
                print_out_str("Thermal info: Parsing failed in "
                              + subparser[0].__name__)
                print_out_str(traceback.format_exc())
        print_out_str("Done thermal parsing")
