# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

from parser_util import register_parser, RamParser


@register_parser('--icc-summary', 'script to dump the interconnect summary to get BW information')
class IccSummary(RamParser):
    def __init__(self, *args):
        super(IccSummary, self).__init__(*args)
        self.fop = self.ramdump.open_file("icc_summary.txt")
        self.fop_provider = self.ramdump.open_file("icc_providers.txt")
        self.external_link_data = []

    def __del__(self):
        self.fop.close()
        self.fop_provider.close()

    def get_internal_link(self, node):
        numlinks = node.num_links
        if node.links == 0x0:
            return
        for i in range(numlinks):
            dnode_addr = node.links + i * 8
            dnode = self.ramdump.read_pdatatype(dnode_addr, 'struct icc_node', attr_list=['name', 'id', 'provider'])
            did = dnode.id
            dname = self.ramdump.read_cstring(dnode.name)
            sname = self.ramdump.read_cstring(node.name)
            sid = node.id
            if node.provider == dnode.provider:
                self.fop_provider.write("    | {0:5}:{1:30} -> {2:5}:{3:30}\n".format(sid, sname, did, dname))
            else:
                data = "+ {0:5}:{1:30} -> {2:5}:{3:30}\n".format(sid, sname, did, dname)
                self.external_link_data.append(data)

    def extract_icc_summary(self):
        icc_providers_addr = self.ramdump.address_of("icc_providers")
        next_offset = self.ramdump.field_offset('struct list_head', 'next')
        icc_providers = self.ramdump.read_pointer(icc_providers_addr + next_offset)
        self.fop_provider.write("[providers]\n")
        self.fop.write("{0:65} {1:12} {2:12} {3:12}\n".format("         node", "tag", "avg", "peak"))
        line = "-" * 150 + "\n"
        formatProvider = "    |    {0:5}:{1:40}\t\t\t   [label={0:5}:{1:40}| avg_bw={2:<10} kBps\t| peak_bw={3:<10} kBps ]\n"
        format_req_str = "      |   {0:90} \t\t\t\t\t\t\t {1:<12}\t\t\t\t  {2:<12}\n"
        formatStr = "+ {0:5}:{1:40} \t\t\t\t{2:<12} {3:<12}\n"
        reqStr = "\t| {0:35} \t\t\t   {1:<12} {2:<12} {3:<12} \n"

        self.fop.write(line)
        provider = self.ramdump.read_linkedlist('struct icc_provider',
                                                "provider_list.next",
                                                icc_providers)
        no_of_provider = len(provider)
        for i in range(no_of_provider):
            self.fop_provider.write("\n\n+ subgraph cluster {}\n".format(i))
            label = self.ramdump.read_datatype(provider[i].dev,
                                               "struct device",
                                               attr_list=['kobj.name'])
            label_name = self.ramdump.read_cstring(label.kobj.name)
            self.fop_provider.write("  + label = {}\n".format(label_name))
            node = provider[i].nodes.next - 0x28
            nodes = self.ramdump.read_linkedlist('struct icc_node',
                                                 'node_list.next', node)
            no_of_nodes = len(nodes)
            for j in range(no_of_nodes):
                icc_node_name = self.ramdump.read_cstring(nodes[j].name)
                avg_bw = nodes[j].avg_bw
                peak_bw = nodes[j].peak_bw
                node_id = nodes[j].id
                self.fop_provider.write(
                    formatProvider.format(node_id, icc_node_name, avg_bw,
                                          peak_bw))
                self.fop.write(
                    formatStr.format(node_id, icc_node_name, avg_bw, peak_bw))
                req_list = nodes[j].req_list.first
                req = self.ramdump.read_linkedlist('struct icc_req',
                                                   'req_node.next', req_list,
                                                   attr_list=['req_node',
                                                              'avg_bw',
                                                              'peak_bw', 'tag',
                                                              'dev'])
                no_of_req = len(req)
                print_seq = 0
                for k in range(no_of_req):
                    dev = self.ramdump.read_datatype(req[k].dev,
                                                     'struct device',
                                                     attr_list=['kobj.name'])
                    dev_name = self.ramdump.read_cstring(dev.kobj.name)
                    req_avg = req[k].avg_bw
                    req_peak = req[k].peak_bw
                    req_tag = req[k].tag
                    if req_avg or req_peak:
                        if print_seq == 0:
                            self.fop_provider.write(
                                "    + reqs (active only)\n")
                        self.fop_provider.write(
                            format_req_str.format(dev_name, req_avg, req_peak))
                        print_seq = 1
                    self.fop.write(
                        reqStr.format(dev_name, req_tag, req_avg, req_peak))
                self.fop.write("\n\n")
                self.fop_provider.write("\n")
            self.fop_provider.write('  + intenral links\n')
            for j in range(no_of_nodes):
                self.get_internal_link(nodes[j])

        if self.external_link_data:
            self.fop_provider.write("\n\n[external links]\n")
            for i in range(len(self.external_link_data)):
                self.fop_provider.write(self.external_link_data[i])

    def parse(self):
        self.extract_icc_summary()
