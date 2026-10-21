# Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
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

@register_parser('--regulator', 'dump regulator and regulator consumer status')
class RegulatorDump(RamParser):

    def __init__(self, *args):
        super(RegulatorDump, self).__init__(*args)
        self.rdev_supply_map = {}
        self.mutex_waiting_map = {}
        self.mutex_waiting_name = {}
        self.rdev = []
        self.consumer = {}
        self.mutex_waiters = {}
        self.global_mutexes = [
            ('regulator_list_mutex', 'regulator_list_mutex'),
            ('regulator_nesting_mutex', 'regulator_nesting_mutex'),
            ('prepare_lock', 'prepare_lock (clock framework)'),
            ('icc_lock', 'icc_lock (interconnect framework)'),
        ]

    def rdev_name(self, rdev):
        constraints = self.ramdump.read_structure_field(rdev, 'struct regulator_dev',
                                                        'constraints')
        if constraints:
            name = self.ramdump.read_structure_field(constraints, 'struct regulation_constraints',
                                                    'name')
            if name:
                return self.ramdump.read_cstring(name)

        desc = self.ramdump.read_structure_field(rdev, 'struct regulator_dev', 'desc')
        name = self.ramdump.read_structure_field(desc, 'struct regulator_desc', 'name')
        if name:
            return self.ramdump.read_cstring(name)

        return '(NULL)'

    def consumer_name(self, regulator):
        dev = self.ramdump.read_structure_field(regulator, 'struct regulator', 'dev')
        dev_name = 'NULL'
        if dev:
            init_name = self.ramdump.read_structure_field(dev, 'struct device', 'init_name')
            if init_name:
                dev_name = self.ramdump.read_cstring(init_name)
            else:
                kobj = self.ramdump.struct_field_addr(dev, 'struct device', 'kobj')
                kobj_name = self.ramdump.read_structure_field(kobj, 'struct kobject', 'name')
                if kobj_name:
                    dev_name = self.ramdump.read_cstring(kobj_name)

        supply_name_addr = self.ramdump.read_structure_field(regulator, 'struct regulator',
                                                            'supply_name')
        if supply_name_addr:
            supply_name = self.ramdump.read_cstring(supply_name_addr)
        else:
            supply_name = 'NULL'

        # Make regulator child consumer names more human readable
        if (('regulator.' in supply_name) and ('-SUPPLY' in supply_name)
                and (regulator in self.rdev_supply_map)):
            rdev = self.rdev_supply_map[regulator]
            rdev_name = self.rdev_name(rdev)
            return supply_name + ' (' + rdev_name + ')'
        else:
            return dev_name + '-' + supply_name

    def task_name(self, task):
        if task is None:
            return None
        comm_addr = task + self.ramdump.field_offset('struct task_struct', 'comm')
        return self.ramdump.read_cstring(comm_addr, max_length=16)

    def mutex_owner(self, mutex):
        owner = self.ramdump.read_structure_field(mutex, 'struct mutex', 'owner')
        if owner:
            # Remove flag bits
            return owner & ~0x7
        else:
            return 0

    def klist_node_to_rdev(self, klist_node):
        # In Linux 4.19, 'knode_class' is an element in struct device
        dev = self.ramdump.container_of(klist_node, 'struct device', 'knode_class')
        if dev is None:
            # In Linux 5.4, 'knode_class' is an element in struct device_private
            dev_private = self.ramdump.container_of(klist_node, 'struct device_private',
                                                    'knode_class')
            if dev_private is None:
                return None
            dev = self.ramdump.read_structure_field(dev_private, 'struct device_private',
                                                    'device')
            if dev is None:
                return None
        return self.ramdump.container_of(dev, 'struct regulator_dev', 'dev')

    def is_rpmh_regulator_rdev(self, rdev):
        if (not self.rpmh_regulator_enable_addr) or (not self.vrm_ops_enable_addr):
            return False

        desc = self.ramdump.read_structure_field(rdev, 'struct regulator_dev', 'desc')
        ops = self.ramdump.read_structure_field(desc, 'struct regulator_desc', 'ops')
        ops_enable_addr = self.ramdump.read_structure_field(ops, 'struct regulator_ops', 'enable')
        if not ops_enable_addr:
            return False

        # Handle CFI branch table of regulator names
        instr = self.ramdump.read_u32(ops_enable_addr)
        mask = 0xFFFFFF
        offset = instr & mask
        # sign bit set
        if offset & 0x800000:
            offset = -((~offset & mask) + 1)
        ops_enable_jump_addr = ops_enable_addr + 4 * offset

        # Check if this regulator is an rpmh_regulator device by seeing if
        # its enable() op is rpm_regulator_enable
        return ((ops_enable_addr == self.rpmh_regulator_enable_addr)
                or (ops_enable_jump_addr == self.rpmh_regulator_enable_addr)
                or (ops_enable_addr == self.vrm_ops_enable_addr)
                or (ops_enable_jump_addr == self.vrm_ops_enable_addr))

    def dump_rpmh_regulator_req_data(self, aggr_vreg, regulator_type, set, req, reg_name):
        rpmh_resource_name = self.ramdump.read_structure_cstring(aggr_vreg,
                                                        'struct rpmh_aggr_vreg', 'resource_name')
        if rpmh_resource_name is None:
            return
        rpmh_resource_addr = self.ramdump.read_structure_field(aggr_vreg, 'struct rpmh_aggr_vreg',
                                                                'addr')
        if rpmh_resource_addr is None:
            return

        sent_reg = self.ramdump.struct_field_addr(req, 'struct rpmh_regulator_request', 'reg')
        valid = self.ramdump.read_structure_field(req, 'struct rpmh_regulator_request', 'valid')

        self.output_file.write('\tRPMh rsc={}, addr=0x{:05X}, set={}; commands sent: '.format(
            rpmh_resource_name, rpmh_resource_addr, set))
        is_first = True
        for i in range(4):
            if valid & (1 << i):
                reg = self.ramdump.read_u32(sent_reg + i * 4)
                self.output_file.write('{}{}={}'.format((', ', '')[is_first], reg_name[i], reg))
                is_first = False
                # Print the vlvl mapped to ARC hlvl
                if regulator_type == 1 and i == 0:
                    level_addr = self.ramdump.struct_field_addr(aggr_vreg, 'struct rpmh_aggr_vreg',
                                                                'level')
                    vlvl = self.ramdump.read_u32(level_addr + 4 * reg)
                    self.output_file.write(' (vlvl={})'.format(vlvl))
        if not valid:
            # No RPMh commands sent
            self.output_file.write('N/A')
        self.output_file.write('\n')

    def dump_rpmh_regulator_data(self, rdev):
        if not self.is_rpmh_regulator_rdev(rdev):
            return

        rpmh_vreg = self.ramdump.read_structure_field(rdev, 'struct regulator_dev', 'reg_data')
        if rpmh_vreg is None:
            return
        aggr_vreg = self.ramdump.read_structure_field(rpmh_vreg, 'struct rpmh_vreg', 'aggr_vreg')
        if aggr_vreg is None:
            # This condition is encountered when parsing a RAM dump which uses
            # the upstream qcom-rpmh-regulator driver
            return
        regulator_type = self.ramdump.read_structure_field(aggr_vreg, 'struct rpmh_aggr_vreg',
                                                            'regulator_type')
        if regulator_type is None:
            return

        if regulator_type == 0:
            # VRM
            reg_name = ['mv', 'en', 'mode', 'hr_mv']
        elif regulator_type == 1:
            # ARC
            reg_name = ['hlvl', 'N/A', 'N/A', 'N/A']
        elif regulator_type == 2:
            # XOB
            reg_name = ['N/A', 'en', 'N/A', 'N/A']
        else:
            self.output_file.write('Unknown type={}\n'.format(regulator_type))
            return

        sleep_request_sent_addr = self.ramdump.struct_field_addr(aggr_vreg,
                                                    'struct rpmh_aggr_vreg', 'sleep_request_sent')
        sleep_request_sent = self.ramdump.read_bool(sleep_request_sent_addr)
        if sleep_request_sent is None:
            return
        aggr_req_active = self.ramdump.struct_field_addr(aggr_vreg, 'struct rpmh_aggr_vreg',
                                                        'aggr_req_active')
        aggr_req_sleep = self.ramdump.struct_field_addr(aggr_vreg, 'struct rpmh_aggr_vreg',
                                                        'aggr_req_sleep')

        self.dump_rpmh_regulator_req_data(aggr_vreg, regulator_type,
                                    ('both', 'act')[sleep_request_sent], aggr_req_active, reg_name)
        if sleep_request_sent:
            self.dump_rpmh_regulator_req_data(aggr_vreg, regulator_type, 'slp', aggr_req_sleep,
                                              reg_name)

    def dump_consumer(self, regulator):
        name = self.consumer_name(regulator)
        enabled = '?'
        enabled = self.ramdump.read_structure_field(regulator, 'struct regulator', 'enable_count')
        if enabled is None:
            enabled = self.ramdump.read_structure_field(regulator, 'struct regulator', 'enabled')
        load_ua_addr = self.ramdump.struct_field_addr(regulator, 'struct regulator', 'uA_load')
        load_ua = self.ramdump.read_s32(load_ua_addr)

        voltage = self.ramdump.struct_field_addr(regulator, 'struct regulator', 'voltage')
        if voltage:
            # Linux 4.19 and 5.4: min_uV and max_uV are found in struct regulator_voltage
            min_uv = self.ramdump.read_structure_field(voltage, 'struct regulator_voltage',
                                                        'min_uV')
            max_uv = self.ramdump.read_structure_field(voltage, 'struct regulator_voltage',
                                                        'max_uV')
        else:
            # Linux 4.14: min_uV and max_uV are found in struct regulator
            min_uv = self.ramdump.read_structure_field(regulator, 'struct regulator', 'min_uV')
            max_uv = self.ramdump.read_structure_field(regulator, 'struct regulator', 'max_uV')
        if min_uv is None:
            min_uv = '?'
        if max_uv is None:
            max_uv = '?'

        self.output_file.write(
            '\t\t{:<64s}  en={}, min_uV={:7d}, max_uV={:7d}, load_uA={:7d}\n'.format(
                name, enabled, min_uv, max_uv, load_ua))

    def dump_mutex(self, mutex, mutex_name):
        owner = self.mutex_owner(mutex)
        if owner:
            comm = self.task_name(owner)
            self.output_file.write('\t{} is locked:\n'.format(mutex_name))
            self.output_file.write('\t\tOwner task  = 0x{:016X} {}\n'.format(owner, comm))
            for waiter_task in self.mutex_waiters[mutex]:
                comm = self.task_name(waiter_task)
                self.output_file.write(
                    '\t\tWaiter task = 0x{:016X} {}\n'.format(waiter_task, comm))

    def dump_rdev(self, rdev):
        name = self.rdev_name(rdev)
        self.output_file.write('{}:\n'.format(name))

        use_count = self.ramdump.read_structure_field(rdev, 'struct regulator_dev', 'use_count')
        self.output_file.write('\tuse_count  = {}\n'.format(use_count))

        open_count = self.ramdump.read_structure_field(rdev, 'struct regulator_dev', 'open_count')
        self.output_file.write('\topen_count = {}\n'.format(open_count))

        constraints = self.ramdump.read_structure_field(rdev, 'struct regulator_dev',
                                                        'constraints')
        min_uv = 0
        max_uv = 0
        if constraints != 0:
            min_uv = self.ramdump.read_structure_field(
                constraints, 'struct regulation_constraints', 'min_uV')
            max_uv = self.ramdump.read_structure_field(
                constraints, 'struct regulation_constraints', 'max_uV')
        self.output_file.write('\tmin_uV     = {}\n'.format(min_uv))
        self.output_file.write('\tmax_uV     = {}\n'.format(max_uv))

        # Calculate total current of enabled voters.  This is used in the Linux
        # regulator framework regulator_set_load() function to configure the
        # regulator mode.
        total_ua = 0
        for consumer in self.consumer[rdev]:
            enabled = self.ramdump.read_structure_field(consumer, 'struct regulator',
                                                        'enable_count')
            if enabled or enabled is None:
                # The per-consumer enable condition when summing the current is
                # only present if struct regulator contains an element named
                # enable_count.  This isn't present in older kernel versions.
                load_ua_addr = self.ramdump.struct_field_addr(consumer, 'struct regulator',
                                                            'uA_load')
                load_ua = self.ramdump.read_s32(load_ua_addr)
                total_ua += load_ua
        self.output_file.write('\ttotal_uA   = {}\n'.format(total_ua))

        self.output_file.write('\trdev       = 0x{:016X}\n'.format(rdev))

        try:
            self.dump_rpmh_regulator_data(rdev)
        except Exception:
            pass

        # Dump the mutex lock owner and waiters if rdev->mutex is locked
        mutex = self.ramdump.struct_field_addr(rdev, 'struct regulator_dev', 'mutex')
        self.dump_mutex(mutex, 'Mutex')

        # Dump the consumer requests for the regulator
        if self.consumer[rdev]:
            self.output_file.write('\tConsumers:\n')
            for consumer in self.consumer[rdev]:
                self.dump_consumer(consumer)

        self.output_file.write('\n')

    def dump_regulators(self):
        if self.rdev:
            self.output_file.write('Regulators:\n\n')
            for rdev in self.rdev:
                self.dump_rdev(rdev)

    def dump_regulator_top_level(self):
        display_global_mutexes = False
        for (mutex_symbol, mutex_label) in self.global_mutexes:
            mutex = self.ramdump.address_of(mutex_symbol)
            owner = self.mutex_owner(mutex)
            if owner:
                display_global_mutexes = True
                break

        if display_global_mutexes:
            self.output_file.write('Top Level Mutexes:\n')
            for (mutex_symbol, mutex_label) in self.global_mutexes:
                mutex = self.ramdump.address_of(mutex_symbol)
                self.dump_mutex(mutex, mutex_label)
            self.output_file.write('\n')

    def mutex_map_gen_walker(self, waiter, mutex, owner_task, lock_name):
        task = self.ramdump.read_structure_field(waiter, 'struct mutex_waiter', 'task')
        self.mutex_waiting_map[task] = owner_task
        self.mutex_waiting_name[task] = lock_name
        self.mutex_waiters[mutex].append(task)

    def store_mutex_waiters(self, mutex, lock_name):
        if mutex is None:
            return
        self.mutex_waiters[mutex] = []
        # Fill mutex_waiting_map to facilitate mutex deadlock detection
        mutex_owner = self.mutex_owner(mutex)
        if mutex_owner:
            wait_list = self.ramdump.struct_field_addr(mutex, 'struct mutex', 'wait_list')
            offset = self.ramdump.field_offset('struct mutex_waiter', 'list')
            wait_list_walker = llist.ListWalker(self.ramdump, wait_list, offset)
            if not wait_list_walker.is_empty():
                wait_list_walker.walk(wait_list_walker.next() + offset, self.mutex_map_gen_walker,
                    mutex, mutex_owner, lock_name)

    def consumer_init_walker(self, regulator, rdev):
        self.consumer[rdev].append(regulator)

    def regulator_init_walker(self, klist_node):
        rdev = self.klist_node_to_rdev(klist_node)
        if rdev is None:
            return

        name = self.rdev_name(rdev)
        if name is None:
            return

        self.rdev.append(rdev)
        self.consumer[rdev] = []

        # Fill rdev_supply_map
        supply = self.ramdump.read_structure_field(rdev, 'struct regulator_dev', 'supply')
        if supply:
            self.rdev_supply_map[supply] = rdev

        mutex = self.ramdump.struct_field_addr(rdev, 'struct regulator_dev', 'mutex')
        self.store_mutex_waiters(mutex, 'rdev->mutex (' + name + ')')

        consumer_list = self.ramdump.struct_field_addr(rdev, 'struct regulator_dev',
                                                        'consumer_list')
        offset = self.ramdump.field_offset('struct regulator', 'list')
        consumer_list_walker = llist.ListWalker(self.ramdump, consumer_list, offset)
        if not consumer_list_walker.is_empty():
            consumer_list_walker.walk(consumer_list_walker.next() + offset,
                                      self.consumer_init_walker, rdev)

    def init_regulators(self):
        regulator_class = self.ramdump.address_of('regulator_class')
        if regulator_class is None:
            self.output.write("ERROR: 'regulator_class' not found\n")
            return
        p = self.ramdump.read_structure_field(regulator_class, 'struct class', 'p')
        list_head = (p + self.ramdump.field_offset('struct subsys_private', 'klist_devices')
                    + self.ramdump.field_offset('struct klist', 'k_list'))
        list_offset = self.ramdump.field_offset('struct klist_node', 'n_node')

        init_list_walker = llist.ListWalker(self.ramdump, list_head, list_offset)
        if not init_list_walker.is_empty():
            init_list_walker.walk(init_list_walker.next() + list_offset,
                                  self.regulator_init_walker)

    def init_regulator_top_level(self):
        for (mutex_symbol, mutex_label) in self.global_mutexes:
            mutex = self.ramdump.address_of(mutex_symbol)
            self.store_mutex_waiters(mutex, mutex_label)

    def init_rpmh_regulator(self):
        # rpmh_regulator_enable_addr and vrm_ops_enable_addr should have the
        # same value as they both correspond to the same function
        # (rpmh_regulator_enable).  However, self.ramdump.address_of() does not
        # return the correct address for function symbols loaded from a DLKM.
        # This occurs because the gdb program itself outputs incorrect addresses
        # in this case.  Thus work around the issue by getting the address two
        # ways and using both in later checks.
        self.rpmh_regulator_enable_addr = self.ramdump.address_of('rpmh_regulator_enable')
        vrm_ops = self.ramdump.address_of('rpmh_regulator_vrm_ops')
        if vrm_ops:
            self.vrm_ops_enable_addr = self.ramdump.read_structure_field(vrm_ops, 'struct regulator_ops', 'enable')
        else:
            self.vrm_ops_enable_addr = 0

    def dump_regulator_cycle(self, back_edge):
        print_out_str('WARNING: REGULATOR MUTEX LOCK DEADLOCK DETECTED!')
        self.output_file.write('REGULATOR MUTEX LOCK DEADLOCK DETECTED!\n')
        self.output_file.write('Locked mutexes forming a cycle:\n')
        waiter = back_edge
        while True:
            owner = self.mutex_waiting_map[waiter]
            lock = self.mutex_waiting_name[waiter]
            self.output_file.write(
                '\tlock = {:<32s}: owner task = 0x{:016X} {:<17s}, '
                'waiter task = 0x{:016X} {:<17s}\n'.format(
                    lock, owner, self.task_name(owner), waiter, self.task_name(waiter)))
            waiter = owner
            if waiter == back_edge:
                break
        self.output_file.write('\n')

    def find_cycle(self, waiter, stack):
        if waiter is None:
            return None
        owner = self.mutex_waiting_map.get(waiter)
        if owner is None:
            return None
        if owner in stack:
            return waiter
        else:
            return self.find_cycle(owner, stack + [owner])

    def regulator_deadlock_check(self):
        for waiter in self.mutex_waiting_map.keys():
            back_edge = self.find_cycle(waiter, [waiter])
            if back_edge:
                self.dump_regulator_cycle(back_edge)
                return

    def parse(self):
        with self.ramdump.open_file('regulator.txt') as self.output_file:
            self.init_regulator_top_level()
            self.init_rpmh_regulator()
            self.init_regulators()
            try:
                self.regulator_deadlock_check()
            except Exception as error:
                # Treat deadlock detection exceptions as non-fatal so that
                # critical regulator details are still dumped.
                self.output_file.write('Regulator mutex deadlock check failed: {}\n'.format(error))
            self.dump_regulator_top_level()
            self.dump_regulators()
            print_out_str("Regulator information written to regulator.txt")
