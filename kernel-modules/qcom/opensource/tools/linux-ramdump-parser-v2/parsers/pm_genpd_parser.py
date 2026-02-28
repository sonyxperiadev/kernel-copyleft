#SPDX-License-Identifier: GPL-2.0-only
#Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.

from print_out import print_out_str
from parser_util import register_parser, RamParser
from tabulate import tabulate


@register_parser('--pm_genpd-parser', 'Print pm genpd summary')
class Genpd(RamParser):

    def __init__(self, *args):
        super(Genpd, self).__init__(*args)

    def parse(self):
        try:
            start = self.ramdump.read_datatype("gpd_list")
        except Exception as e:
            print("genpd support is not available")
            return
        start = start.next - self.ramdump.field_offset("struct generic_pm_domain", "gpd_list_node")
        gpd_ll= self.ramdump.read_linkedlist("struct generic_pm_domain", "gpd_list_node.next", start)

        table = []

        for l in gpd_ll:
            c_start = l.parent_links.next - self.ramdump.field_offset("struct gpd_link", "parent_node")
            children = self.ramdump.read_linkedlist("struct gpd_link", "parent_node.next", c_start)
            c_list = []
            for c in children:
                if c:
                    ch = self.ramdump.read_datatype(c.child, "struct generic_pm_domain")
                    c_list.append(self.ramdump.read_cstring(ch.dev.kobj.name))
            table.append([self.ramdump.read_cstring(l.dev.kobj.name),l.status,c_list,l.performance_state])
            dev_list = self.ramdump.read_linkedlist("struct pm_domain_data", "list_node.next", l.dev_list.next)
            for d in dev_list:
                if d.dev != 0:
                    d = self.ramdump.read_datatype(d.dev, "struct device")
                    table.append([".    "+str(self.ramdump.read_cstring(d.kobj.name)), ".    "+str(d.power.runtime_status),"",""])

        head = ["domain (device)","status (runtime status)","children","performance"]

        self.output_file = self.ramdump.open_file('genpd_summary.txt')
        self.output_file.write(tabulate(table,head,tablefmt="github"))
        self.output_file.close()
        print_out_str("--- Wrote the output to genpd_summary.txt")
