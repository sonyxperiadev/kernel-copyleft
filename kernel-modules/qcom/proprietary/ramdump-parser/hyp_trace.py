"""
Copyright (c) 2020-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
"""

import sys
import os

THREAD_KIND_VCPU = 2
THREAD_KIND_IDLE = 1
THREAD_KIND_VCPU_NEW = 0

class HypDump(object):
    def __init__(self, ramdump):
        self.ramdump = ramdump
        self.arm64 = ramdump.arm64
        self.kaslr_addr = None
        self.kaslr_offset = None
        self.page_offset = 0x80000000
        self.hyp_kaslr_addr_offset = None
        self.hyp_cpu_thread = None
        self.hyp_struct_change = False
        self.vmtype = "svm"
        self.vmid = 45
        self.ttbr1 = None

    def get_val(self,addr):
        addr_offset = addr - self.hyp_kaslr_addr_offset
        addr_offset_data = addr_offset + self.page_offset
        val = self.ramdump.read_u64(addr_offset_data,False)
        return val
    def get_val_thread(self,addr):
        addr_offset = addr - self.thread_kaslr_offset
        addr_offset_data = addr_offset + self.thread_page_offset
        val = self.ramdump.read_u64(addr_offset_data,False)
        return val
    def get_val_32(self,addr):
        addr_offset = addr - self.hyp_kaslr_addr_offset
        addr_offset_data = addr_offset + self.page_offset
        val = self.ramdump.read_u32(addr_offset_data,False)
        return val
    def determine_kaslr(self):
        hyp_kaslr_addr = self.ramdump.address_of('aarch64_kaslr_base')
        hyp_kaslr_addr = hyp_kaslr_addr + self.page_offset
        self.hyp_kaslr_addr_offset = self.ramdump.read_u64(hyp_kaslr_addr,False)

    def get_phys_offset(self,thread_list_next,partition_base):
        addr_mask = 0xFFFFFFFFFF000000
        phys_offset_list = {}
        region_phys_offset = self.page_offset
        region_kaslr_offset = self.hyp_kaslr_addr_offset

        if self.hyp_struct_change:
            mapped_ranges_phys_offset  = self.ramdump.field_offset('partition_t', 'mapped_ranges.phys')
            mapped_ranges_virt_offset  = self.ramdump.field_offset('partition_t', 'mapped_ranges.virt')
            mapped_ranges_size_offset  = self.ramdump.field_offset('partition_t', 'mapped_ranges.size')
            mapped_ranges_entry_size = self.ramdump.sizeof('((partition_t *)0x0)->mapped_ranges[0]')
            mapped_ranges = int(self.ramdump.sizeof('((partition_t *)0x0)->mapped_ranges') / mapped_ranges_entry_size)
        else:
            mapped_ranges_phys_offset  = self.ramdump.field_offset('struct partition', 'mapped_ranges.phys')
            mapped_ranges_virt_offset  = self.ramdump.field_offset('struct partition', 'mapped_ranges.virt')
            mapped_ranges_size_offset  = self.ramdump.field_offset('struct partition', 'mapped_ranges.size')
            mapped_ranges_entry_size = self.ramdump.sizeof('((struct partition *)0x0)->mapped_ranges[0]')
            mapped_ranges = int(self.ramdump.sizeof('((struct partition *)0x0)->mapped_ranges') / mapped_ranges_entry_size)

        for idx in range (0, mapped_ranges):
            mapped_ranges_phys_addr = partition_base + mapped_ranges_phys_offset + (idx * mapped_ranges_entry_size)
            mapped_ranges_phys_addr = mapped_ranges_phys_addr - self.hyp_kaslr_addr_offset
            mapped_ranges_phys_addr = mapped_ranges_phys_addr + self.page_offset
            mapped_ranges_phys = self.ramdump.read_u64(mapped_ranges_phys_addr,False)

            if mapped_ranges_phys:
                mapped_ranges_virt_addr = partition_base + mapped_ranges_virt_offset + (idx * mapped_ranges_entry_size)
                mapped_ranges_virt_addr = mapped_ranges_virt_addr - self.hyp_kaslr_addr_offset
                mapped_ranges_virt_addr = mapped_ranges_virt_addr + self.page_offset
                mapped_ranges_virt = self.ramdump.read_u64(mapped_ranges_virt_addr,False)

                mapped_ranges_size_addr = partition_base + mapped_ranges_size_offset + (idx * mapped_ranges_entry_size)
                mapped_ranges_size_addr = mapped_ranges_size_addr - self.hyp_kaslr_addr_offset
                mapped_ranges_size_addr = mapped_ranges_size_addr + self.page_offset
                mapped_ranges_size = self.ramdump.read_u64(mapped_ranges_size_addr,False)

                phys_offset_list[mapped_ranges_phys] = {}
                phys_offset_list[mapped_ranges_phys]["virt"] = mapped_ranges_virt
                phys_offset_list[mapped_ranges_phys]["size"] = mapped_ranges_size

        for item in phys_offset_list:
            temp_virt = phys_offset_list[item]["virt"]
            temp_size = phys_offset_list[item]["size"]
            if thread_list_next >= temp_virt and thread_list_next < temp_virt + temp_size:
                thread_list_next_offset = thread_list_next - temp_virt
                temp_thread_list_next = thread_list_next_offset + item
                temp_thread_list_next = self.ramdump.read_u64(temp_thread_list_next,False)
                mask_temp_thread_list_next = temp_thread_list_next & addr_mask
                mask_thread_list_next = thread_list_next & addr_mask
                if mask_thread_list_next <= mask_thread_list_next:
                    region_phys_offset = item
                    region_kaslr_offset = temp_virt
                    break
        return region_phys_offset,region_kaslr_offset

    def get_trace_phy(self):
        if self.vmtype == "oemvm":
            self.vmid = 49

        hyp_cpu_thread = self.ramdump.address_of('cpulocal_primary_thread')
        hyp_cpu_thread = hyp_cpu_thread - self.hyp_kaslr_addr_offset
        hyp_cpu_thread_offset = hyp_cpu_thread + self.page_offset
        self.hyp_cpu_thread_data = self.ramdump.read_u64(hyp_cpu_thread_offset,False)
        partition_list_offset = self.ramdump.field_offset('struct partition', 'partition_list_node')
        if partition_list_offset is None:
            self.hyp_struct_change = True
        if self.hyp_struct_change:
            partition_list_offset = self.ramdump.field_offset('partition_t', 'partition_list_node')
            thread_list_offset = self.ramdump.field_offset('thread_t', 'thread_list_node')
            partition_therad_list_offset = self.ramdump.field_offset('partition_t', 'thread_list')
        else:
            thread_list_offset = self.ramdump.field_offset('struct thread', 'thread_list_node')
            partition_therad_list_offset = self.ramdump.field_offset('struct partition', 'thread_list')
        partition_list = self.ramdump.address_of('partition_list')
        partition_list_next = partition_list - self.hyp_kaslr_addr_offset
        partition_list_next = partition_list_next + self.page_offset
        next = self.ramdump.read_u64(partition_list_next,False)
        found_flag = False
        while(partition_list != next):
            partition_base = next - partition_list_offset
            thread_list_head = partition_base + partition_therad_list_offset
            thread_list_next_offset = thread_list_head - self.hyp_kaslr_addr_offset
            thread_list_next = thread_list_next_offset + self.page_offset
            thread_list_next = self.ramdump.read_u64(thread_list_next,False)
            if thread_list_head != thread_list_next:
                self.thread_page_offset,self.thread_kaslr_offset = self.get_phys_offset(thread_list_next,partition_base)
            while(thread_list_head != thread_list_next):
                cpu_thread_data = thread_list_next - thread_list_offset
                if self.hyp_struct_change:
                    kid_offset = self.ramdump.field_offset('thread_t', 'kind')
                else:
                    kid_offset = self.ramdump.field_offset('struct thread', 'kind')
                kind_data = cpu_thread_data + kid_offset
                val = self.get_val_thread(kind_data)
                if val == THREAD_KIND_VCPU or val == THREAD_KIND_VCPU_NEW:
                    if self.hyp_struct_change:
                        addrspace_offset = self.ramdump.field_offset('thread_t', 'addrspace')
                        vcpu_regs_el1_offset = self.ramdump.field_offset('thread_t', 'vcpu_regs_el1')
                        vcpu_regs_el2_offset = self.ramdump.field_offset('thread_t', 'vcpu_regs_el2')
                    else:
                        addrspace_offset = self.ramdump.field_offset('struct thread', 'addrspace')
                        vcpu_regs_el1_offset = self.ramdump.field_offset('struct thread', 'vcpu_regs_el1')
                        vcpu_regs_el2_offset = self.ramdump.field_offset('struct thread', 'vcpu_regs_el2')
                    addrspace_data = cpu_thread_data + addrspace_offset
                    addrspace_res = self.get_val_thread(addrspace_data)
                    if addrspace_res:
                        vmid_offset = self.ramdump.field_offset('addrspace_t', 'vmid')
                        vmid_data = addrspace_res + vmid_offset
                        vmid_res = self.get_val_thread(vmid_data)
                        if vmid_res == self.vmid:
                            vm_pgtable_offset = self.ramdump.field_offset('addrspace_t', 'vm_pgtable')
                            vm_pgtable = addrspace_res + vm_pgtable_offset
                            vttbr_offset = self.ramdump.field_offset('pgtable_vm_t', 'vttbr_el2')
                            vttbr_el2 = vm_pgtable + vttbr_offset
                            vttbr_el2_data = self.get_val_thread(vttbr_el2)
                            self.vttbr_el2_data = vttbr_el2_data
                            self.vttbr = vttbr_el2_data & 0xFFFFFFFFFFFE
                            VTCR_EL2_offset = self.ramdump.field_offset('pgtable_vm_t', 'vtcr_el2')
                            VTCR_EL2 = vm_pgtable + VTCR_EL2_offset
                            VTCR_EL2_data = self.get_val_thread(VTCR_EL2)
                            self.VTCR_EL2 = VTCR_EL2_data
                            #self.VTCR_EL2 = VTCR_EL2_data  & 0xFFFFFFFFFFFE

                            ttbr1_offset = self.ramdump.field_offset('vcpu_el1_registers_t', 'ttbr1_el1')
                            vcpu_regs_el1 = cpu_thread_data + vcpu_regs_el1_offset
                            ttbr1_data = vcpu_regs_el1 + ttbr1_offset
                            self.ttbr1 = self.get_val_thread(ttbr1_data)

                            self.ttbr1_data_info = self.ttbr1
                            self.ttbr1 = self.ttbr1 & 0xFFFFFFFFFFFE

                            SCTLR_EL1_offset = self.ramdump.field_offset('vcpu_el1_registers_t', 'sctlr_el1')
                            SCTLR_EL1_data = vcpu_regs_el1 + SCTLR_EL1_offset
                            self.SCTLR_EL1 = self.get_val_thread(SCTLR_EL1_data)
                            #self.SCTLR_EL1 = self.SCTLR_EL1 & 0xFFFFFFFFFFFE

                            TTBR0_EL1_offset = self.ramdump.field_offset('vcpu_el1_registers_t', 'ttbr0_el1')
                            TTBR0_EL1_data = vcpu_regs_el1 + TTBR0_EL1_offset
                            self.TTBR0_EL1 = self.get_val_thread(TTBR0_EL1_data)
                            #self.TTBR0_EL1 = self.TTBR0_EL1 & 0xFFFFFFFFFFFE

                            TCR_EL1_offset = self.ramdump.field_offset('vcpu_el1_registers_t', 'tcr_el1')
                            TCR_EL1_data = vcpu_regs_el1 + TCR_EL1_offset
                            self.TCR_EL1 = self.get_val_thread(TCR_EL1_data)
                            #self.TCR_EL1 = self.TCR_EL1 & 0xFFFFFFFFFFFE

                            HCR_EL2_offset = self.ramdump.field_offset('vcpu_el2_registers_t', 'hcr_el2')
                            vcpu_regs_el2 = cpu_thread_data + vcpu_regs_el2_offset
                            HCR_EL2_data = vcpu_regs_el2 + HCR_EL2_offset
                            self.HCR_EL2 = self.get_val_thread(HCR_EL2_data)
                            #self.HCR_EL2 = self.HCR_EL2 & 0xFFFFFFFFFFFE
                            found_flag = True
                            break
                if found_flag:
                    break
                thread_list_next_offset = thread_list_next - self.thread_kaslr_offset
                thread_list_next = thread_list_next_offset + self.thread_page_offset
                thread_list_next = self.ramdump.read_u64(thread_list_next,False)

            next_offset = next - self.hyp_kaslr_addr_offset
            next_offset = next_offset + self.page_offset
            next = self.ramdump.read_u64(next_offset,False)

    def get_ttbr(self):
        cspace_offset = self.ramdump.field_offset('struct thread', 'cspace_cspace')
        cspace_offset_addr = self.hyp_cpu_thread_data + cspace_offset
        cspace_offset_addr = cspace_offset_addr - self.hyp_kaslr_addr_offset
        cspace_offset_addr_actual = cspace_offset_addr + self.page_offset

        self.cspace_offset_addr_actual_data = self.ramdump.read_u64(cspace_offset_addr_actual,False)
        cap_thread_offset = self.ramdump.field_offset('cspace_t', 'tables')
        self.cspace_table_addr = self.cspace_offset_addr_actual_data +  cap_thread_offset #offset of table
        cspace_table_addr_offset = self.cspace_table_addr - self.hyp_kaslr_addr_offset
        cspace_table_addr_offset = cspace_table_addr_offset + self.page_offset
        self.cspace_table_addr_actual = self.ramdump.read_u64(cspace_table_addr_offset,False)

        cap_count_offset = self.ramdump.field_offset('cspace_t', 'cap_count')
        cap_count_addr = self.cspace_offset_addr_actual_data +  cap_count_offset
        cap_count_addr_offset = cap_count_addr - self.hyp_kaslr_addr_offset
        cap_count_addr_offset = cap_count_addr_offset + self.page_offset
        cap_count_value = self.ramdump.read_u64(cap_count_addr_offset,False)
        i = 0

        cap_slots_offset = self.ramdump.field_offset('cap_table_t', 'cap_slots')
        cap_data_offset = self.ramdump.field_offset('cap_t', 'data')
        cap_object_offset = self.ramdump.field_offset('cap_data_t', 'object')
        cap_info_offset = self.ramdump.field_offset('cap_data_t', 'info')
        cap_bf_offset = self.ramdump.field_offset('cap_info_t', 'bf')
        cap_slots_addr = self.cspace_table_addr_actual +  cap_slots_offset

        object_thread_offset = self.ramdump.field_offset('object_ptr_t', 'thread')
        addrspace_offset = self.ramdump.field_offset('thread_t', 'addrspace')
        vm_pgtable_offset = self.ramdump.field_offset('addrspace_t', 'vm_pgtable')

        vttbr_offset = self.ramdump.field_offset('pgtable_vm_t', 'vttbr_el2')
        regs_el1_offset = self.ramdump.field_offset('thread_t', 'vcpu_regs_el1')
        ttbr1_offset = self.ramdump.field_offset('vcpu_el1_registers_t', 'ttbr1_el1')
        while i < cap_count_value:
            cap_slots_addr_next = self.ramdump.array_index(
                cap_slots_addr, 'cap_t', i)
            cap_data = cap_slots_addr_next + cap_data_offset
            cap_object = cap_data + cap_object_offset
            cap_info = cap_data + cap_info_offset
            cap_bf = cap_info + cap_bf_offset

            cap_bf_addr_offset = cap_bf - self.hyp_kaslr_addr_offset
            cap_bf_addr_offset = cap_bf_addr_offset + self.page_offset
            cap_bf_addr = self.ramdump.read_u64(cap_bf_addr_offset,False)
            cap_type = cap_bf_addr >> 8
            i = i + 1
            if cap_type == 8:
                svm_thread_offset = cap_object + object_thread_offset
                svm_thread_addr_offset = (svm_thread_offset - self.hyp_kaslr_addr_offset) + self.page_offset
                svm_thread_addr = self.ramdump.read_u64(svm_thread_addr_offset,False)

                addrspace_addr_offset = svm_thread_addr + addrspace_offset
                addrspace_addr_data = (addrspace_addr_offset - self.hyp_kaslr_addr_offset) + self.page_offset
                addrspace_addr = self.ramdump.read_u64(addrspace_addr_data,False)
                pgtable_addr = addrspace_addr + vm_pgtable_offset

                vttbr_addr_offset = pgtable_addr + vttbr_offset
                vttbr_addr = (vttbr_addr_offset - self.hyp_kaslr_addr_offset) + self.page_offset
                self.vttbr = self.ramdump.read_u64(vttbr_addr,False)
                self.vttbr = self.vttbr & 0xFFFFFFFFFFFE
                regs_el1 = svm_thread_addr + regs_el1_offset

                ttbr1_addr_offset = regs_el1 + ttbr1_offset
                ttbr1_addr = (ttbr1_addr_offset - self.hyp_kaslr_addr_offset) + self.page_offset
                self.ttbr1 = self.ramdump.read_u64(ttbr1_addr,False)
                self.ttbr1 = self.ttbr1 & 0xFFFFFFFFFFFE
