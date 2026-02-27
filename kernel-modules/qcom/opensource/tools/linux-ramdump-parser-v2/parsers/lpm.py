# Copyright (c) 2015-2018, 2020-2021 The Linux Foundation. All rights reserved.
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

import linux_list
from parser_util import register_parser, RamParser
from operator import itemgetter
from collections import OrderedDict

@register_parser('--lpm', 'Parse LPM Driver info')
class lpm(RamParser):
    def __init__(self, *args):
        super(lpm, self).__init__(*args)
        self.head = ''
        self.output = []
        self.clusters = []
        self.cpu_possible_bits = None
        self.cpu_online_bits = 0
        self.lpm_debug = []
        self.related_cpus_bits = None


    def get_bits(self):
        bits_addr = self.ramdump.address_of('cpu_possible_bits')
        if bits_addr is None:
            bits_addr = self.ramdump.address_of('__cpu_possible_mask')
            if bits_addr is None:
                self.output.append("NOTE: 'cpu_possible_bits' not found")
                return

        self.cpu_possible_bits = self.ramdump.read_int(bits_addr)
        cpus = bin(self.cpu_possible_bits).count('1')
        self.output.append("{}\n".format('Available CPUs'))
        for i in self.ramdump.iter_cpus():
                self.output.append("{:10}{}:{}\n".format("", "CPU", i))
        self.output.append("\n")

        if (self.ramdump.kernel_version >= (4, 9, 0)):
            if self.ramdump.is_config_defined('CONFIG_SMP'):
                runqueues_addr = self.ramdump.address_of('runqueues')
                online_offset = self.ramdump.field_offset('struct rq', 'online')
                for i in self.ramdump.iter_cpus():
                    online = self.ramdump.read_int(runqueues_addr + online_offset, cpu=i)
                    self.cpu_online_bits |= (online << i)
        else:
            bits_addr = self.ramdump.address_of('cpu_online_bits')
            if bits_addr is None:
                bits_addr = self.ramdump.address_of('__cpu_online_mask')
                if bits_addr is None:
                    self.output.append("NOTE: 'cpu_online_bits' not found")
                    return

        self.cpu_online_bits = self.ramdump.read_int(bits_addr)
        self.output.append("{}\n".format('Online CPUs'))
        index = 0
        while self.cpu_online_bits:
            if self.cpu_online_bits & 1:
                self.output.append("{:10}{}:{}\n".format("", "CPU", index))
            self.cpu_online_bits = self.cpu_online_bits >> 1
            index += 1
        self.output.append("{}{}{}".format("\n", "-" * 120, "\n"))

    def get_cluster_level_info(self, lpm_cluster):
        offset = self.ramdump.field_offset('struct lpm_cluster', 'nlevels')
        nlevels = self.ramdump.read_int(lpm_cluster + offset)
        self.output.append("{:20}:{}\n".format("number of levels", nlevels))

        offset = self.ramdump.field_offset('struct lpm_cluster', 'min_child_level')
        node = self.ramdump.read_int(lpm_cluster + offset)
        self.output.append("{:20}:{}\n".format("min child level", node))

        offset = self.ramdump.field_offset('struct lpm_cluster', 'default_level')
        node = self.ramdump.read_int(lpm_cluster + offset)
        self.output.append("{:20}:{}\n".format("default level", node))

        offset = self.ramdump.field_offset('struct lpm_cluster', 'last_level')
        node = self.ramdump.read_int(lpm_cluster + offset)
        self.output.append("{:20}:{}\n".format("last_level", node))

        offset = self.ramdump.field_offset('struct lpm_cluster', 'levels')
        levels = lpm_cluster + offset
        self.output.append("\n")

        cluster_level_size = self.ramdump.sizeof('struct lpm_cluster_level')

        for i in range(nlevels):
                # ToDo: Need a better way to arrive at the next level info.
                level = levels + (i * cluster_level_size)

                # Case to handle 'mode'.'mode' removed in kernel version ( >= 4.9) for core power lpm drivers.
                if (self.ramdump.kernel_version < (4,9,0) ):
                    offset = self.ramdump.field_offset('struct lpm_cluster_level', 'mode')
                    addr = self.ramdump.read_word(level + offset, True)
                    node = self.ramdump.read_int(addr)
                    self.output.append("{:20}:{}\n".format("level mode", node))

                offset = self.ramdump.field_offset('struct lpm_cluster_level', 'level_name')
                addr = self.ramdump.read_word(level + offset, True)
                name = self.ramdump.read_cstring(addr, 48)
                self.output.append("{:20}:{}\n".format("level name", name))

                offset = self.ramdump.field_offset('struct lpm_cluster_level', 'min_child_level')
                addr = level + offset
                node = self.ramdump.read_int(addr)
                self.output.append("{:20}:{}\n".format("min child level", node))

                offset = self.ramdump.field_offset('struct lpm_cluster_level', 'num_cpu_votes')
                addr = level + offset
                node = self.ramdump.read_int(addr)
                self.output.append("{:20}:{}({})\n".format("num cpu votes", hex(node).rstrip("L"), bin(node).lstrip("0b")))

                offset = self.ramdump.field_offset('struct lpm_cluster_level', 'available')
                addr = level + offset

                offset = self.ramdump.field_offset('struct lpm_level_avail', 'idle_enabled')
                node = self.ramdump.read_bool(addr + offset)
                self.output.append("{:20}:{}\n".format("idle_enabled", node))

                offset = self.ramdump.field_offset('struct lpm_level_avail', 'suspend_enabled')
                node = self.ramdump.read_bool(addr + offset)
                self.output.append("{:20}:{}\n".format("suspend_enabled", node))
                self.output.append("\n")

    def get_cluster_info(self, lpm_cluster):
        offset = self.ramdump.field_offset('struct lpm_cluster', 'cluster_name')
        addr = self.ramdump.read_word(lpm_cluster + offset, True)
        node = self.ramdump.read_cstring(addr, 48)
        self.output.append("{:20}:{}\n".format("Cluster Name", node))

        offset = self.ramdump.field_offset('struct lpm_cluster', 'child_cpus')
        node = self.ramdump.read_int(lpm_cluster + offset)
        self.output.append("{:20}:{}({})\n".format("child_cpus", hex(node).rstrip("L"), bin(node).lstrip("0b")))
        if (self.ramdump.kernel_version >= (3, 18, 0) or
            self.ramdump.kernel_version < (3, 14, 0) ):
                offset = self.ramdump.field_offset(
                        'struct lpm_cluster', 'num_children_in_sync')
        else:
                offset = self.ramdump.field_offset(
                        'struct lpm_cluster', 'num_childs_in_sync')
        node = self.ramdump.read_int(lpm_cluster + offset)
        self.output.append("{:20}:{}({})\n".format(
                "num_children_in_sync", hex(node).rstrip("L"),
                    bin(node).lstrip("0b")))
        self.output.append("\n")

    def lpm_walker(self, lpm_cluster):
        if lpm_cluster == self.head:
                return
        self.clusters.append(lpm_cluster)

    def get_pm_domains(self):

        gpd_offset = self.ramdump.field_offset('struct generic_pm_domain', 'gpd_list_node')
        head = self.ramdump.read_word(self.ramdump.address_of('gpd_list'), True)
        self.head = head

        gpd_walker = linux_list.ListWalker(self.ramdump, head, gpd_offset)
        gpd_walker.walk(head, self.get_pm_domain_info)

    def get_pm_domain_info(self, node):
        if node == self.head:
            return

        name_offset = self.ramdump.field_offset('struct generic_pm_domain', 'name')
        name = self.ramdump.read_cstring(self.ramdump.read_word(node + name_offset))
        if not name:
            return
        name += ":idle_states:S{}"

        state_count = self.ramdump.read_structure_field(node, 'struct generic_pm_domain', 'state_count')
        #sanity check for state count , CPUIDLE_STATE_MAX
        if state_count >= 10:
            return
        accounting_time = self.ramdump.read_structure_field(node, 'struct generic_pm_domain', 'accounting_time')
        gpd_power_state_size = self.ramdump.sizeof('struct genpd_power_state')
        power_state_offset = self.ramdump.field_offset('struct generic_pm_domain', 'states')


        for i in range(state_count):
            power_state_addr = self.ramdump.read_word(node + power_state_offset + i * gpd_power_state_size)

            msec = self.ramdump.read_structure_field(power_state_addr, 'struct genpd_power_state', 'idle_time')
            ktime = self.ramdump.read_s64('last_jiffies_update') - accounting_time
            ktime = ktime // 10 ** 6

            if msec:
                msec += ktime
            usage = self.ramdump.read_structure_field(power_state_addr, 'struct genpd_power_state', 'usage')
            rejected = self.ramdump.read_structure_field(power_state_addr, 'struct genpd_power_state', 'rejected')
            residency_ns = self.ramdump.read_structure_field(power_state_addr, 'struct genpd_power_state','residency_ns')


            if not msec:
                msec = "0"
            if not usage:
                usage = "0"
            if not rejected:
                rejected = "0"
            if name:
                msg = u"{name:30}  {msec:30}  {usage:30} {rejected:30}".format(name=name.format(i),msec=msec,usage=usage,rejected=rejected)
                self.output.append(msg)

            self.output.append("\n")

        return

    def print_pm_domain_info(self):
        self.output.append("{}\n".format('Power Domain Info: '))
        self.output.append("{:30}{:30}{:30}{:30} \n".format("Power domain", "Time Spent(msec)", "Usage ", "Rejected "))
        self.get_pm_domains()
        self.output.append("{}{}".format("-" * 81, "\n"))

    def get_clusters(self):
        lpm_root_node = self.ramdump.read_word(
            self.ramdump.address_of('lpm_root_node'), True)
        if lpm_root_node is None:
            self.output_file.write("NOTE: 'lpm_root_node' not found\n")
            return

        self.clusters.append(lpm_root_node)

        offset = self.ramdump.field_offset('struct lpm_cluster', 'child')
        lpm_cluster = self.ramdump.read_word(lpm_root_node + offset, True)
        self.head = lpm_root_node + offset

        offset = self.ramdump.field_offset('struct lpm_cluster', 'list')
        lpm_walker = linux_list.ListWalker(self.ramdump, lpm_cluster, offset)
        lpm_walker.walk(lpm_cluster, self.lpm_walker)


    def get_cpu_level_info(self, cpu_cluster_base, cpu, cpu_level):
        self.output.append("{:20}:{}\n".format("CPU", cpu))

        if self.ramdump.kernel_version >= (4,9,0):
            cpu_level = cpu_level
        else:

            cpu_cluster = self.ramdump.read_word(cpu_cluster_base, cpu=cpu)
            offset = self.ramdump.field_offset('struct lpm_cluster', 'cpu')
            cpu_level = self.ramdump.read_word(cpu_cluster + offset, True)


        offset = self.ramdump.field_offset('struct lpm_cpu', 'nlevels')
        nlevels = self.ramdump.read_int(cpu_level + offset, True)
        self.output.append("{:20}:{}\n".format("number of levels", nlevels))
        offset = self.ramdump.field_offset('struct lpm_cpu', 'levels')
        levels = cpu_level + offset

        self.output.append("\n")

        cpu_level_available = self.ramdump.address_of('cpu_level_available')
        if cpu_level_available is None:
                self.output.append("NOTE: 'cpu_level_available' not found\n")
                return
        cpu_level_available = cpu_level_available + self.ramdump.sizeof('long') * cpu
        cpu_level_available = self.ramdump.read_word(cpu_level_available, True)

        for i in range(0, nlevels):
                level = levels + (i * self.ramdump.sizeof('struct lpm_cpu_level'))

                offset = self.ramdump.field_offset('struct lpm_cpu_level', 'name')
                addr = self.ramdump.read_word(level + offset, True)
                node = self.ramdump.read_cstring(addr, 48)
                self.output.append("{:20}:{}\n".format("level name", node))

                # Case to handle 'mode'.'mode' removed in kernel version ( > = 4.9) for core power lpm drivers.
                if (self.ramdump.kernel_version < (4,9,0) ):
                    offset = self.ramdump.field_offset('struct lpm_cpu_level', 'mode')
                    node = self.ramdump.read_int(level + offset, True)
                    self.output.append("{:20}:{}\n".format("level mode", node))

                level_available = cpu_level_available + i * self.ramdump.sizeof('struct lpm_level_avail')
                offset = self.ramdump.field_offset('struct lpm_level_avail', 'idle_enabled')
                node = self.ramdump.read_bool(level_available + offset)
                self.output.append("{:20}:{}\n".format("idle enabled", node))

                offset = self.ramdump.field_offset('struct lpm_level_avail', 'suspend_enabled')
                node = self.ramdump.read_bool(level_available + offset, True)
                self.output.append("{:20}:{}\n".format("suspend enabled", node))

                self.output.append("\n")

        self.output.append("{}{}".format("-" * 120, "\n"))

    def get_lpm(self):
        self.get_clusters()
        for i in self.clusters:
                self.get_cluster_info(i)
                self.get_cluster_level_info(i)
                self.output.append("{}{}".format("-" * 120, "\n"))

        if self.ramdump.kernel_version >= (4,9,0):
            cpu_cluster_base = self.ramdump.address_of('lpm_root_node')
        else:
            cpu_cluster_base = self.ramdump.address_of('cpu_cluster')


        if cpu_cluster_base is None:
                self.output.append("NOTE: 'cpu_cluster' not found\n")
                return

        if self.ramdump.kernel_version >= (4,9,0):
            cpu_cluster = self.ramdump.read_word(cpu_cluster_base)
            related_cpus_offset = self.ramdump.field_offset('struct lpm_cpu', 'related_cpus')

            bits_offset = self.ramdump.field_offset('struct cpumask', 'bits')
            offset = self.ramdump.field_offset('struct lpm_cluster', 'cpu')
            clust_node_list = ['next','prev']
            for clust_node in clust_node_list:
                cpunode_offset = self.ramdump.field_offset('struct list_head', clust_node)
                offset = offset + cpunode_offset
                cpu_level = self.ramdump.read_word(cpu_cluster + offset, True)
                self.related_cpus_bits = self.ramdump.read_int(cpu_level + related_cpus_offset + bits_offset, True)
                cpus = bin(self.related_cpus_bits).count('1')
                cpu_info = self.related_cpus_bits
                cpu_count = 0
                while (cpu_info):
                    if ( cpu_info  & 0x1):
                        self.get_cpu_level_info(cpu_cluster_base, cpu_count,cpu_level)
                    cpu_info = cpu_info >> 0x1
                    cpu_count = cpu_count + 1
        else:
            cpus = bin(self.cpu_possible_bits).count('1')
            for i in range(0, cpus):
                    self.get_cpu_level_info(cpu_cluster_base, i,0x0)


    def get_time_stats(self, tstats, nlevels):
        for i in range(nlevels):
                lstats = tstats + i * self.ramdump.sizeof('struct level_stats')

                offset = self.ramdump.field_offset('struct level_stats', 'name')
                addr = self.ramdump.read_word(lstats + offset, True)
                self.output.append("{:20}:{}\n".format("lpm name", self.ramdump.read_cstring(addr + offset, 48)))

                offset = self.ramdump.field_offset('struct level_stats', 'success_count')
                self.output.append("{:20}:{}\n".format("success_count", self.ramdump.read_int(lstats + offset, True)))

                offset = self.ramdump.field_offset('struct level_stats', 'failed_count')
                self.output.append("{:20}:{}\n".format("failed_count", self.ramdump.read_int(lstats + offset, True)))

                self.output.append("\n")

    def get_cluster_stats(self, cluster):
        offset = self.ramdump.field_offset('struct lpm_cluster', 'stats')
        stats = self.ramdump.read_word(cluster + offset, True)

        offset = self.ramdump.field_offset('struct lpm_stats', 'name')
        self.output.append("{} {}\n\n".format(self.ramdump.read_cstring(stats + offset, 48), "lpm stats"))

        offset = self.ramdump.field_offset('struct lpm_stats', 'num_levels')
        nlevels = self.ramdump.read_int(stats + offset, True)

        offset = self.ramdump.field_offset('struct lpm_stats', 'time_stats')
        tstats = self.ramdump.read_word(stats + offset, True)

        self.get_time_stats(tstats, nlevels)
        self.output.append("{}{}".format("-" * 120, "\n"))

    def get_cpu_stats(self, cpu_stats_base, cpu):
        stats = cpu_stats_base + self.ramdump.per_cpu_offset(cpu)

        offset = self.ramdump.field_offset('struct lpm_stats', 'name')
        self.output.append("{} {}\n\n".format(self.ramdump.read_cstring(stats + offset, 48), "lpm stats"))

        offset = self.ramdump.field_offset('struct lpm_stats', 'num_levels')
        nlevels = self.ramdump.read_int(stats + offset, True)

        offset = self.ramdump.field_offset('struct lpm_stats', 'time_stats')
        tstats = self.ramdump.read_word(stats + offset, True)

        self.get_time_stats(tstats, nlevels)
        self.output.append("{}{}".format("-" * 120, "\n"))

    def get_stats(self):
        for i in self.clusters:
                self.get_cluster_stats(i)

        cpu_stats_base = self.ramdump.address_of('cpu_stats')
        if cpu_stats_base is None:
                self.output.append("NOTE: 'cpu_stats' not found\n")
                return

        cpus = bin(self.cpu_possible_bits).count('1')
        for i in self.ramdump.iter_cpus():
                self.get_cpu_stats(cpu_stats_base, i)

    def get_debug_phys(self):
        lpm_debug_phys = self.ramdump.address_of('lpm_debug_phys')
        if lpm_debug_phys is None:
                self.output.append("NOTE: 'lpm_debug data' not found\n")
                return
        lpm_debug_phys = self.ramdump.read_word(lpm_debug_phys, True)

        for i in range(0, 256):
                debug = []

                addr = lpm_debug_phys + i * self.ramdump.sizeof('struct lpm_debug')

                offset = self.ramdump.field_offset('struct lpm_debug', 'time')
                time = self.ramdump.read_word(addr + offset, False)
                debug.append(time)

                offset = self.ramdump.field_offset('struct lpm_debug', 'evt')
                evt = self.ramdump.read_int(addr + offset, False)
                debug.append(evt)

                offset = self.ramdump.field_offset('struct lpm_debug', 'cpu')
                cpu = self.ramdump.read_int(addr + offset, False)
                debug.append(cpu)

                offset = self.ramdump.field_offset('struct lpm_debug', 'arg1')
                arg1 = self.ramdump.read_int(addr + offset, False)
                debug.append(arg1)

                offset = self.ramdump.field_offset('struct lpm_debug', 'arg2')
                arg2 = self.ramdump.read_int(addr + offset, False)
                debug.append(arg2)

                offset = self.ramdump.field_offset('struct lpm_debug', 'arg3')
                arg3 = self.ramdump.read_int(addr + offset, False)
                debug.append(arg3)

                offset = self.ramdump.field_offset('struct lpm_debug', 'arg4')
                arg4 = self.ramdump.read_int(addr + offset, False)
                debug.append(arg4)

                self.lpm_debug.append(debug)

    def print_debug_phys(self):
        debug = []
        lpm_debug = []

        self.output.append("\n")
        self.output.append("{:16}".format("TimeStamp"))
        self.output.append("{:8} {:8} {:8} ".format("Event", "CPU", "arg1"))
        self.output.append("{:16}{:16}{:16}\n".format("arg2", "arg3", "arg4"))
        self.output.append("{}{}".format("-" * 120, "\n"))

        lpm_debug = sorted(self.lpm_debug, key=itemgetter(0))

        for i in range(len(lpm_debug)):
                debug = lpm_debug[i]
                for j in range(len(debug)):
                        if j == 0 or j > 3:
                                self.output.append("{:16}".format(hex(debug[j]).rstrip("L")))
                        else:
                                self.output.append("{}{:8}".format(debug[j], ""))

                self.output.append("\n")

    def get_cpuidle_usage_details(self, state_usage_addr):
        usage_stats = OrderedDict()
        usage_stats['disable'] = self.ramdump.read_structure_field(state_usage_addr,
                                                                    'struct cpuidle_state_usage',
                                                                    'disabled')
        usage_stats['usage'] = self.ramdump.read_structure_field(state_usage_addr,
                                                                    'struct cpuidle_state_usage',
                                                                    'usage')
        usage_stats['time_ns'] = self.ramdump.read_structure_field(state_usage_addr,
                                                                    'struct cpuidle_state_usage',
                                                                    'time_ns')
        usage_stats['rejected'] = self.ramdump.read_structure_field(state_usage_addr,
                                                                    'struct cpuidle_state_usage',
                                                                    'rejected')
        usage_stats['s2idle_usage'] = self.ramdump.read_structure_field(state_usage_addr,
                                                                    'struct cpuidle_state_usage',
                                                                    's2idle_usage')
        usage_stats['s2idle_time'] = self.ramdump.read_structure_field(state_usage_addr,
                                                                    'struct cpuidle_state_usage',
                                                                    's2idle_time')
        return usage_stats

    def print_state_description(self, data):
        self.output.append("\n\n")
        self.output.append("CPUIdle States Description\n\n")
        self.output.append("{}{}".format("-" * 120, "\n"))
        self.output.append(" {:^30} {:^50}\n".format("State", "Description"))
        for key, value in data.items():
            self.output.append("{:^30}  {:^50}\n".format(key, value))

    def print_cpuidle_stats(self, data):
        self.output.append("\n\n")
        self.output.append("CPUIdle Statistics\n\n")

        self.output.append("{:^8}{:^20}{:^16}{:^16}{:^25}{:^16}{:^16}{:^16}\n".format("CPU",
                            "State", "Disabled", "Usage", "Time_ns",
                            "Rejected", "S2idle_usage", "S2idle_time"))

        self.output.append("{}{}".format("-" * 120, "\n"))
        for cpu in data.keys():
            self.output.append("{:^8}\n".format(cpu))
            for state in data[cpu].keys():
                self.output.append("{:^8}".format(""))
                self.output.append("{:^20}".format(state))
                for key, value in data[cpu][state].items():
                    if not value:
                        value = "0"
                    if key == "time_ns":
                        self.output.append("{:^25}".format(value))
                    else:
                        self.output.append("{:^16}".format(value))
                self.output.append("\n")
        self.output.append("\n{}{}".format("-" * 120, "\n"))

    def get_cpuidle_statistics(self):
        try:
            # Read cpuidle_device
            cpuidle_devices = self.ramdump.address_of('cpuidle_devices')
            cpuidle_drivers = self.ramdump.address_of('cpuidle_drivers')
            usage_details = {}
            state_dict = OrderedDict()
            for i in self.ramdump.iter_cpus():
                # Check for current cpu
                cpuidle_dev = self.ramdump.read_word(cpuidle_devices, cpu=i)
                cpuidle_drv = self.ramdump.read_word(cpuidle_drivers, cpu=i)
                state_count = self.ramdump.read_structure_field(cpuidle_drv,
                                                                'struct cpuidle_driver',
                                                                'state_count')
                cpuidle_drv_states = cpuidle_drv + self.ramdump.field_offset(
                                                                'struct cpuidle_driver',
                                                                'states')
                if (self.ramdump.kernel_version >= (5, 10, 0)):
                    cpuidle_kobj_offset = self.ramdump.field_offset(
                                                                'struct cpuidle_device',
                                                                'kobjs')
                    kobjs_base = cpuidle_dev + cpuidle_kobj_offset
                    cpu = "CPU{}".format(i)
                    usage_details[cpu] = {}
                    for state_idx in range(state_count):
                        if state_count < 10: # CPUIDLE_STATE_MAX
                            temp = kobjs_base + state_idx * self.ramdump.sizeof('void*')
                            cpuidle_state_usage_base = self.ramdump.read_u64(kobjs_base + state_idx * self.ramdump.sizeof('void*'))
                            if cpuidle_state_usage_base != 0x0:                               
                                state_usage_offset = cpuidle_state_usage_base + self.ramdump.field_offset('struct cpuidle_state_kobj','state_usage')
                                state_usage_addr = self.ramdump.read_u64(state_usage_offset)
                                cpuidle_drv_offset = cpuidle_state_usage_base + self.ramdump.field_offset('struct cpuidle_state_kobj','state')
                                cpuidle_drv_state = self.ramdump.read_u64(cpuidle_drv_offset)
                                state_name = self.ramdump.read_cstring(cpuidle_drv_state, 16)
                                desc_offset = self.ramdump.field_offset('struct cpuidle_state', 'desc')
                                state_desc = self.ramdump.read_cstring(cpuidle_drv_state + desc_offset, 32)
                                # Usage data per cpu per state
                                state = "state{}({})".format(state_idx, state_name)
                                state_dict[state_name] = state_desc
                                usage_details[cpu][state] = self.get_cpuidle_usage_details(state_usage_addr)
                            else:
                                usage_details[cpu][state] = 0
                        else:
                            break

                else:
                    cpuidle_state_usage_offset = self.ramdump.field_offset(
                                                                'struct cpuidle_device',
                                                                'states_usage')
                    cpuidle_state_usage_base = cpuidle_dev + cpuidle_state_usage_offset
                    cpu = "CPU{}".format(i)
                    usage_details[cpu] = {}
                    for state in range(state_count):
                        if state_count < 10: # CPUIDLE_STATE_MAX
                            state_usage_addr = cpuidle_state_usage_base + \
                                               state * self.ramdump.sizeof('struct cpuidle_state_usage')
                            cpuidle_drv_state = cpuidle_drv_states + \
                                               state * self.ramdump.sizeof('struct cpuidle_state')
                            state_name = self.ramdump.read_cstring(cpuidle_drv_state, 16)
                            desc_offset = self.ramdump.field_offset('struct cpuidle_state', 'desc')
                            state_desc = self.ramdump.read_cstring(cpuidle_drv_state + desc_offset, 32)
                            # Usage data per cpu per state
                            state = "state{}({})".format(state, state_name)
                            state_dict[state_name] = state_desc
                            usage_details[cpu][state] = self.get_cpuidle_usage_details(state_usage_addr)
                        else:
                            break
            self.print_state_description(state_dict)
            self.print_cpuidle_stats(usage_details)
        except Exception as err:
            self.output.append("\nUnable to extract CPUIdle Statistics\n\n")
            self.output.append("\n{}{}".format("-" * 120, "\n"))

    def parse(self):
        self.output_file = self.ramdump.open_file('lpm.txt')
        self.get_bits()
        self.get_lpm()
        self.get_stats()
        self.get_debug_phys()
        self.print_debug_phys()
        self.print_pm_domain_info()
        self.get_cpuidle_statistics()
        for i in self.output:
                self.output_file.write(i)
        self.output_file.close()
