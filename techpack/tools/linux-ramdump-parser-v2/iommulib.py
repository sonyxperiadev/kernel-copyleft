# Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import rb_tree
import linux_list as llist
from mm import phys_to_virt
from print_out import print_out_str

ARM_SMMU_DOMAIN = 0
MSM_SMMU_DOMAIN = 1
MSM_SMMU_AARCH64_DOMAIN = 2


class Domain(object):
    def __init__(self, pg_table, redirect, ctx_list, client_name,
                 domain_type=MSM_SMMU_DOMAIN, level=3, domain_num=-1):
        self.domain_num = domain_num
        self.pg_table = pg_table
        self.redirect = redirect
        self.ctx_list = ctx_list
        self.client_name = client_name
        self.level = level
        self.domain_type = domain_type

    def __repr__(self):
        return "#%d: %s" % (self.domain_num, self.client_name)


class IommuLib(object):
    def __init__(self, ramdump):
        self.ramdump = ramdump
        self.domain_list = []

        if self.find_iommu_domains_msm_iommu():
            pass
        elif self.find_iommu_domains_debug_attachments():
            pass
        elif self.find_iommu_domains_device_core():
            pass
        else:
            print_out_str("Unable to find any iommu domains")

    """
    legacy code - pre-8996/kernel 4.4?
    """
    def find_iommu_domains_msm_iommu(self):
        domains = list()
        root = self.ramdump.read_word('domain_root')
        if root is None:
            return False

        rb_walker = rb_tree.RbTreeWalker(self.ramdump)
        rb_walker.walk(root, self._iommu_domain_func, self.domain_list)
        return True

    def use_only_iommu_debug_attachments(self, debug_attachment):
        has_pgtbl_info = self.ramdump.read_structure_field(debug_attachment,
                         'struct iommu_debug_attachment', 'fmt')
        has_client_name = self.ramdump.read_structure_field(debug_attachment,
                         'struct iommu_debug_attachment', 'client_name')

        if has_pgtbl_info and has_client_name:
            return True;
        return False

    def find_iommu_domains_legacy(self, debug_attachment):
        domain_ptr = self.ramdump.read_structure_field( debug_attachment,
                     'struct iommu_debug_attachment', 'domain')

        if not domain_ptr:
            return

        ptr = self.ramdump.read_structure_field(
            debug_attachment, 'struct iommu_debug_attachment', 'group')
        if ptr is not None:
            dev_list = ptr + self.ramdump.field_offset(
                'struct iommu_group', 'devices')
            dev = self.ramdump.read_structure_field(
                dev_list, 'struct list_head', 'next')
            if self.ramdump.kernel_version >= (4, 14):
                client_name = self.ramdump.read_structure_cstring(
                    dev, 'struct group_device', 'name')
            else:
                client_name = self.ramdump.read_structure_cstring(
                    dev, 'struct iommu_device', 'name')
        else:
            """Older kernel versions have the field 'dev'
            instead of 'iommu_group'.
            """
            ptr = self.ramdump.read_structure_field(
                debug_attachment, 'struct iommu_debug_attachment', 'dev')
            kobj_ptr = ptr + self.ramdump.field_offset('struct device', 'kobj')
            client_name = self.ramdump.read_structure_cstring(
                kobj_ptr, 'struct kobject', 'name')


        has_pgtbl_info = self.ramdump.read_structure_field(debug_attachment,\
                         'struct iommu_debug_attachment', 'fmt') is not None
        if self.ramdump.kernel_version >= (5, 4, 0) and has_pgtbl_info:
            self._find_iommu_domains_debug_attachments(debug_attachment,\
                                            client_name, self.domain_list)
        else:
            self._find_iommu_domains_arm_smmu(domain_ptr, client_name,\
                                              self.domain_list)

    def find_iommu_domains(self, debug_attachment):
        client_name = self.ramdump.read_structure_cstring(debug_attachment,
                      'struct iommu_debug_attachment', 'client_name')
        self._find_iommu_domains_debug_attachments(debug_attachment,
                                                   client_name,
                                                   self.domain_list)

    """
    depends on CONFIG_IOMMU_DEBUG_TRACKING
    """
    def find_iommu_domains_debug_attachments(self):
        list_head_attachments = self.ramdump.address_of(
                                                    'iommu_debug_attachments')
        if list_head_attachments is None:
            return False

        offset = self.ramdump.field_offset('struct iommu_debug_attachment',
                                          'list')
        list_walker = llist.ListWalker(self.ramdump, list_head_attachments, offset)

        for debug_attachment in list_walker:
            if self.use_only_iommu_debug_attachments(debug_attachment):
                self.find_iommu_domains(debug_attachment)
            else:
                self.find_iommu_domains_legacy(debug_attachment)

        return True

    """
    will generate domains using only the information stored in the debug
    attachments structure.
    """
    def _find_iommu_domains_debug_attachments(self, debug_attachment,\
                                              client_name, domain_list):
        levels = self.ramdump.read_structure_field(debug_attachment,\
                                    'struct iommu_debug_attachment', 'levels')
        pg_table = self.ramdump.read_structure_field(debug_attachment,\
                                'struct iommu_debug_attachment', 'ttbr0')
        domain = Domain(pg_table, 0, [], client_name, ARM_SMMU_DOMAIN,
                        levels)
        domain_list.append(domain)

    """
    will only find active iommu domains. This means it will exclude most gpu domains.
    """
    def find_iommu_domains_device_core(self):
        domains = set()
        devices_kset = self.ramdump.read_pointer('devices_kset')
        if not devices_kset:
            return False

        list_head = devices_kset + self.ramdump.field_offset('struct kset',
                                                             'list')

        offset = self.ramdump.field_offset('struct device', 'kobj.entry')
        list_walker = llist.ListWalker(self.ramdump, list_head, offset)

        for dev in list_walker:
            iommu_group = self.ramdump.read_structure_field(dev, 'struct device', 'iommu_group')
            if not iommu_group:
                continue

            domain_ptr = self.ramdump.read_structure_field(iommu_group, 'struct iommu_group', 'domain')
            if not domain_ptr:
                continue

            if domain_ptr in domains:
                continue

            domains.add(domain_ptr)

            client_name_addr = self.ramdump.read_structure_field(dev, 'struct device', 'kobj.name')
            client_name = self.ramdump.read_cstring(client_name_addr)

            self._find_iommu_domains_arm_smmu(domain_ptr, client_name, self.domain_list)

        return True


    def _find_iommu_domains_arm_smmu(self, domain_ptr, client_name, domain_list):
        if self.ramdump.field_offset('struct iommu_domain', 'priv') \
                is not None:
            priv_ptr = self.ramdump.read_structure_field(
                domain_ptr, 'struct iommu_domain', 'priv')

            if not priv_ptr:
                return
        else:
            priv_ptr = None

        if self.ramdump.kernel_version >= (5, 4, 0):
            smmu_iommu_ops_offset = self.ramdump.field_offset('struct msm_iommu_ops','iommu_ops')
            arm_smmu_ops_data = self.ramdump.address_of('arm_smmu_ops')
            arm_smmu_ops = arm_smmu_ops_data + smmu_iommu_ops_offset
        else:
            arm_smmu_ops = self.ramdump.address_of('arm_smmu_ops')

        iommu_domain_ops = self.ramdump.read_structure_field(
            domain_ptr, 'struct iommu_domain', 'ops')
        if iommu_domain_ops is None or iommu_domain_ops == 0:
            return

        if iommu_domain_ops == arm_smmu_ops:
            if priv_ptr is not None:
                arm_smmu_domain_ptr = priv_ptr
            elif self.ramdump.kernel_version >= (5, 4, 0):
                arm_smmu_domain_ptr_wrapper = self.ramdump.container_of(
                        domain_ptr, 'struct msm_iommu_domain', 'iommu_domain')
                arm_smmu_domain_ptr = self.ramdump.container_of(
                        arm_smmu_domain_ptr_wrapper, 'struct arm_smmu_domain', 'domain')
            else:
                arm_smmu_domain_ptr = self.ramdump.container_of(
                    domain_ptr, 'struct arm_smmu_domain', 'domain')

            pgtbl_ops_ptr = self.ramdump.read_structure_field(
                arm_smmu_domain_ptr, 'struct arm_smmu_domain', 'pgtbl_ops')
            if pgtbl_ops_ptr is None or pgtbl_ops_ptr == 0:
                return

            level = 0
            fn = self.ramdump.read_structure_field(pgtbl_ops_ptr,
                    'struct io_pgtable_ops', 'map')
            if fn == self.ramdump.address_of('av8l_fast_map'):
                level = 3
            else:
                arm_lpae_io_pgtable_ptr = self.ramdump.container_of(
                    pgtbl_ops_ptr, 'struct arm_lpae_io_pgtable', 'iop.ops')

                level = self.ramdump.read_structure_field(
                    arm_lpae_io_pgtable_ptr, 'struct arm_lpae_io_pgtable',
                    'levels')

            if self.ramdump.kernel_version >= (5, 4, 0):
                pgtbl_info_offset = self.ramdump.field_offset('struct arm_smmu_domain','pgtbl_info')
                pgtbl_info_data = arm_smmu_domain_ptr + pgtbl_info_offset
                pg_table = self.ramdump.read_structure_field(pgtbl_info_data,'struct msm_io_pgtable_info','pgtbl_cfg.arm_lpae_s1_cfg.ttbr[0]')
            else:
                pg_table = self.ramdump.read_structure_field(
                    arm_smmu_domain_ptr, 'struct arm_smmu_domain',
                    'pgtbl_cfg.arm_lpae_s1_cfg.ttbr[0]')

            pg_table = phys_to_virt(self.ramdump, pg_table)

            domain_create = Domain(pg_table, 0, [], client_name,
                                   ARM_SMMU_DOMAIN, level)
            domain_list.append(domain_create)
        else:
            priv_pt_offset = self.ramdump.field_offset('struct msm_iommu_priv',
                                                       'pt')
            pgtable_offset = self.ramdump.field_offset('struct msm_iommu_pt',
                                                       'fl_table')
            redirect_offset = self.ramdump.field_offset('struct msm_iommu_pt',
                                                        'redirect')

            if priv_pt_offset is not None:
                pg_table = self.ramdump.read_u64(
                    priv_ptr + priv_pt_offset + pgtable_offset)
                redirect = self.ramdump.read_u64(
                   priv_ptr + priv_pt_offset + redirect_offset)

            if (self.ramdump.is_config_defined('CONFIG_IOMMU_AARCH64')):
                domain_create = Domain(pg_table, redirect, [], client_name,
                                       MSM_SMMU_AARCH64_DOMAIN)
            else:
                domain_create = Domain(pg_table, redirect, [], client_name,
                                       MSM_SMMU_DOMAIN)

            domain_list.append(domain_create)

    def _iommu_list_func(self, node, ctx_list):
        ctx_drvdata_name_ptr = self.ramdump.read_word(
            node + self.ramdump.field_offset('struct msm_iommu_ctx_drvdata',
                                             'name'))
        ctxdrvdata_num_offset = self.ramdump.field_offset(
            'struct msm_iommu_ctx_drvdata', 'num')
        num = self.ramdump.read_u32(node + ctxdrvdata_num_offset)
        if ctx_drvdata_name_ptr != 0:
            name = self.ramdump.read_cstring(ctx_drvdata_name_ptr, 100)
            ctx_list.append((num, name))

    def _iommu_domain_func(self, node, domain_list):
        domain_num = self.ramdump.read_u32(self.ramdump.sibling_field_addr(
            node, 'struct msm_iova_data', 'node', 'domain_num'))
        domain = self.ramdump.read_word(self.ramdump.sibling_field_addr(
            node, 'struct msm_iova_data', 'node', 'domain'))
        priv_ptr = self.ramdump.read_word(
            domain + self.ramdump.field_offset('struct iommu_domain', 'priv'))

        client_name_offset = self.ramdump.field_offset(
            'struct msm_iommu_priv', 'client_name')

        if client_name_offset is not None:
            client_name_ptr = self.ramdump.read_word(
                priv_ptr + self.ramdump.field_offset(
                    'struct msm_iommu_priv', 'client_name'))
            if client_name_ptr != 0:
                client_name = self.ramdump.read_cstring(client_name_ptr, 100)
            else:
                client_name = '(null)'
        else:
            client_name = 'unknown'

        list_attached_offset = self.ramdump.field_offset(
                'struct msm_iommu_priv', 'list_attached')

        if list_attached_offset is not None:
            list_attached = self.ramdump.read_word(priv_ptr +
                                                   list_attached_offset)
        else:
            list_attached = None

        priv_pt_offset = self.ramdump.field_offset('struct msm_iommu_priv',
                                                   'pt')
        pgtable_offset = self.ramdump.field_offset('struct msm_iommu_pt',
                                                   'fl_table')
        redirect_offset = self.ramdump.field_offset('struct msm_iommu_pt',
                                                    'redirect')

        if priv_pt_offset is not None:
            pg_table = self.ramdump.read_word(
                priv_ptr + priv_pt_offset + pgtable_offset)
            redirect = self.ramdump.read_u32(
                priv_ptr + priv_pt_offset + redirect_offset)
        else:
            # On some builds we are unable to look up the offsets so hardcode
            # the offsets.
            pg_table = self.ramdump.read_word(priv_ptr + 0)
            redirect = self.ramdump.read_u32(priv_ptr +
                                             self.ramdump.sizeof('void *'))

            # Note: On some code bases we don't have this pg_table and redirect
            # in the priv structure (see msm_iommu_sec.c). It only contains
            # list_attached. If this is the case we can detect that by checking
            # whether pg_table == redirect (prev == next pointers of the
            # attached list).
            if pg_table == redirect:
                # This is a secure domain. We don't have access to the page
                # tables.
                pg_table = 0
                redirect = None

        ctx_list = []
        if list_attached is not None and list_attached != 0:
            list_walker = llist.ListWalker(
                self.ramdump, list_attached,
                self.ramdump.field_offset('struct msm_iommu_ctx_drvdata',
                                          'attached_elm'))
            list_walker.walk(list_attached, self._iommu_list_func, ctx_list)

            if (self.ramdump.is_config_defined('CONFIG_IOMMU_AARCH64')):
                domain_create = Domain(pg_table, redirect, ctx_list, client_name,
                                       MSM_SMMU_AARCH64_DOMAIN, domain_num=domain_num)
            else:
                domain_create = Domain(pg_table, redirect, ctx_list, client_name,
                                       MSM_SMMU_DOMAIN, domain_num=domain_num)

            domain_list.append(domain_create)
