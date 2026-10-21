# Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#
#   * Neither the name of The Linux Foundation nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import struct
import os
import sys
import subprocess
from print_out import print_out_str

"""dictionary mapping from (hw_id, client_id, version) to class CacheDump"""
lookuptable = {}


def lookup_tlb_type(hwid, client_id, version):
    """defaults to CacheDump() if no match found"""
    return lookuptable.get((hwid, client_id, version), TlbDump())


def formatwidth(string, limit):
    if len(string) >= limit:
        return string[0:limit]
    formatstr = '{{0:{0}}}'.format(limit)
    return formatstr.format(string)

class TableOutputFormat:
    """ Not sure if using PrettyTable (python lib) is a good idea, since people
    would need to install it"""

    def __init__(self):
        self.titlebar = []
        self.datafmt = []
        self.have_printed_title = False
        self.separator = ' '

    def addColumn(self, string, datafmt='{0}', width=0):
        width = max(len(string), width)
        string = formatwidth(string, width)
        self.titlebar.append(string)
        self.datafmt.append(datafmt)

    def printline(self, array, outfile):
        if (len(array) != len(self.titlebar)):
            raise Exception('BadTableDataSize', array, self.titlebar)

        if (not self.have_printed_title):
            self.have_printed_title = True
            outfile.write(self.separator.join(self.titlebar))
            outfile.write('\n')

        for item, title, fmt in zip(array, self.titlebar, self.datafmt):
            item = fmt.format(item)
            item = formatwidth(item, len(title))
            outfile.write(item)
            outfile.write(self.separator)

        outfile.write('\n')


class TlbDump(object):
    """ Class to describe a method to parse a particular type of tlbdump.
    Users should not make instances of this class."""
    def __init__(self):
        """do nothing"""

    def parse(self, start, end, ramdump, outfile):
        """Called from debug_image_v2.py. Overriden by child classes"""
        raise NotImplementedError

class TlbDumpType(TlbDump):
    def __init__(self):
        super(TlbDumpType, self).__init__()
        self.tableformat = TableOutputFormat()
        self.tableformat.addColumn('Way', '{0:01x}')
        self.tableformat.addColumn('Set', '{0:03x}')
        self.ramdump = None
        self.linefmt = None

    def add_table_data_columns(self):
        for i in range(0, self.LineSize):
            str = "DATA{0}".format(i)
            self.tableformat.addColumn(str, '{0:08x}', 8)

    def read_line(self, start):
        if self.linefmt is None:
            self.linefmt = '<'
            self.linefmt += 'I'* self.LineSize
        return self.ramdump.read_string(start, self.linefmt, virtual=False)


class TlbDumpType_v1(TlbDumpType):
    def __init__(self):
            self.infile_name = "scratch.bin"

    def parse(self, start, end, ramdump, outfile):
        self.ramdump = ramdump
        self.outfile = outfile
        """kryo tlb parser expects an input file with dump data for the tlb so
           temporarily create file with dump data"""
        infile_fd = ramdump.open_file(self.infile_name, mode='rb')
        core_dump_size = end - start
        buf = ramdump.read_physical(start, core_dump_size)
        infile_fd.write(buf)
        infile_fd.close()
        self.parse_dump()
        ramdump.remove_file(self.infile_name)

    def parse_dump(self):
        #child class should implement this method
        raise NotImplementedError

    def kryo_tlb_parse(self, cmd, offset, outfile_name):
        #expected cmdline arguments for kryo tlb parser.
        opts_flgs = ["-i", "-o", "-t", "-c", "-s"]
        infile_path = os.path.join(self.ramdump.outdir, self.infile_name)
        outfile_path = os.path.join(self.ramdump.outdir, outfile_name)
        cpu_name = self.cpu_name
        offset_str = str(offset)
        opts_params = [infile_path, outfile_path, cmd, cpu_name, offset_str]
        argv = [None] * (2 * len(opts_flgs))
        #get the path for the kryo_cache_tlb_parser.py in this directory
        exec_path = sys.argv[0].strip("ramparse.py")
        exec_path += "kryo_cache_tlb_json_parser.py"

        for i in xrange(len(opts_flgs)):
            argv[2 * i] = opts_flgs[i]
            argv[(2 * i) + 1] = opts_params[i]

        argv_str = " ".join(str(x) for x in argv)
        """Since the kryo tlb parser expects the data to be redirected to the
           outfile, and we're not calling it from the
           command line to redirect the output, we can do it like this"""
        outfile_fd = self.ramdump.open_file(outfile_name)
        sys.stdout.flush()
        sys.stdout = outfile_fd
        """We must do this, since the parser's main() function does not take
           any arguments; they must be passed through the command line"""
        os.system("python " + exec_path + " " + argv_str)
        sys.stdout.flush()
        sys.stdout = sys.__stdout__
        outfile_fd.close()

class TlbDumpType_v2(TlbDumpType):
    def __init__(self):
        super(TlbDumpType_v2, self).__init__()

    def parse(self, start, end, ramdump, outfile):
        self.ramdump = ramdump
        self.add_table_data_columns()

        ram = 0
        for nway in range(self.NumWaysRam0):
            offset = 0
            for nset in range(self.NumSetsRam0):
                if start > end:
                    raise Exception('past the end of array')

                output = [nway, nset]
                line = self.read_line(start)
                self.parse_tag_fn(output, line, nset, nway, ram, offset)
                output.extend(line)
                self.tableformat.printline(output, outfile)
                start = start + self.LineSize * 0x4
                offset = offset + 0x1000

        ram = 1
        for nway in range(self.NumWaysRam1):
            offset = 0
            for nset in range(self.NumSetsRam1):
                if start > end:
                    print([nway,nset])
                    raise Exception('past the end of array')

                output = [nway, nset]
                line = self.read_line(start)
                self.parse_tag_fn(output, line, nset, nway, ram, offset)
                output.extend(line)
                self.tableformat.printline(output, outfile)
                start = start + self.LineSize * 0x4
                offset = offset + 0x1000

class TlbDumpType_v3(object):
    def __init__(self):
        self.infile_name = "scratch.bin"

    def parse(self, start, end, ramdump, outfile):
        self.ramdump = ramdump
        self.outfile = outfile
        """kryo tlb parser expects an input file with dump data for the tlb so
           temporarily create file with dump data"""
        infile_fd = ramdump.open_file(self.infile_name, mode="wb")
        core_dump_size = end - start
        buf = ramdump.read_physical(start, core_dump_size)
        infile_fd.write(buf)
        infile_fd.close()
        self.parse_dump()
        ramdump.remove_file(self.infile_name)

    def parse_dump(self):
        #child class should implement this method
        raise NotImplementedError

    def kryo_tlb_parse(self, cmd, offset, outfile_name):
        #expected cmdline arguments for kryo tlb parser.
        opts_flgs = ["-i", "-o", "-t", "-c", "-s"]
        infile_path = os.path.join(self.ramdump.outdir, self.infile_name)
        outfile_path = os.path.join(self.ramdump.outdir, outfile_name)
        cpu_name = self.cpu_name
        offset_str = str(offset)
        opts_params = [infile_path, outfile_path, cmd, cpu_name, offset_str]
        argv = [None] * (2 * len(opts_flgs))
        #get the path for the kryo_cache_tlb_parser.py in this directory
        exec_path = sys.argv[0].strip("ramparse.py")
        exec_path += "kryo_cache_tlb_json_parser.py"

        for i in range(len(opts_flgs)):
            argv[2 * i] = opts_flgs[i]
            argv[(2 * i) + 1] = opts_params[i]

        """We must do this, since the parser's main() function does not take
           any arguments; they must be passed through the command line"""
        exec_name = "kryo_cache_tlb_json_parser.py"
        exec_abspath = os.path.join(self.ramdump.get_srcdir(), exec_name)
        args = ["py -2", exec_abspath]
        args.extend(argv)
        args_str = " ".join(str(x) for x in args)
        """We must do this, since the parser's main() function does not take
           any arguments; they must be passed through the command line"""
        p = subprocess.run(args_str, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True)
        if p.returncode < 0:
            print_out_str("{} Failed with error {}".format(p.args, p.returncode))
        print_out_str(p.stderr)
        self.outfile.close()
        os.remove(self.outfile.name)


class TlbDumpType_v4(object):
    def __init__(self):
        self.infile_name = "scratch.bin"

    def parse(self, start, end, ramdump, outfile):
        self.ramdump = ramdump
        self.outfile = outfile
        """kryo tlb parser expects an input file with dump data for the tlb so
           temporarily create file with dump data"""
        infile_fd = ramdump.open_file(self.infile_name, mode='wb')
        core_dump_size = end - start
        buf = ramdump.read_physical(start, core_dump_size)
        infile_fd.write(buf)
        infile_fd.close()
        self.parse_dump()
        ramdump.remove_file(self.infile_name)

    def parse_dump(self):
        #child class should implement this method
        raise NotImplementedError

    def kryo_tlb_parse(self, cmd, offset, outfile_name):
        #expected cmdline arguments for kryo tlb parser.
        opts_flgs = ["-i", "-o", "-t", "-c", "-s"]
        infile_path = os.path.join(self.ramdump.outdir, self.infile_name)
        outfile_path = os.path.join(self.ramdump.outdir, outfile_name)
        cpu_name = self.cpu_name
        offset_str = str(offset)
        opts_params = [infile_path, outfile_path, cmd, cpu_name, offset_str]
        argv = [None] * (2 * len(opts_flgs))
        #get the path for the kryo_cache_tlb_parser.py in this directory
        exec_path = sys.argv[0].strip("ramparse.py")
        exec_path += "kryo_cache_tlb_json_parser.py"

        for i in range(len(opts_flgs)):
            argv[2 * i] = opts_flgs[i]
            argv[(2 * i) + 1] = opts_params[i]

        argv_str = " ".join(str(x) for x in argv)

        """We must do this, since the parser's main() function does not take
           any arguments; they must be passed through the command line"""
        exec_name = "kryo_cache_tlb_json_parser.py"
        exec_abspath = os.path.join(self.ramdump.get_srcdir(), exec_name)
        args = ["py -2", exec_abspath]
        args.extend(argv)
        args_str = " ".join(str(x) for x in args)
        """We must do this, since the parser's main() function does not take
           any arguments; they must be passed through the command line"""
        p = subprocess.run(args_str, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True)
        if p.returncode < 0:
            print_out_str("{} Failed with error {}".format(p.args, p.returncode))
        print_out_str(p.stderr)
        self.outfile.close()
        os.remove(self.outfile.name)

class L1_TLB_KRYO2XX_GOLD(TlbDumpType_v2):
    def __init__(self):
        super(L1_TLB_KRYO2XX_GOLD, self).__init__()
        self.tableformat.addColumn('RAM')
        self.tableformat.addColumn('TYPE')
        self.tableformat.addColumn('PA', '{0:016x}', 16)
        self.tableformat.addColumn('VA', '{0:016x}', 16)
        self.tableformat.addColumn('VALID')
        self.tableformat.addColumn('VMID', '{0:02x}', 4)
        self.tableformat.addColumn('ASID', '{0:04x}', 4)
        self.tableformat.addColumn('S1_MODE', '{0:01x}', 7)
        self.tableformat.addColumn('S1_LEVEL', '{0:01x}', 8)
        self.tableformat.addColumn('SIZE')
        self.unsupported_header_offset = 0
        self.LineSize = 4
        self.NumSetsRam0 =  0x100
        self.NumSetsRam1 =  0x3c
        self.NumWaysRam0 = 4
        self.NumWaysRam1 = 2

    def parse_tag_fn(self, output, data, nset, nway, ram, offset):
        tlb_type = self.parse_tlb_type(data)

        s1_mode = (data[0] >> 2) & 0x3

        s1_level = data[0] & 0x3

        pa = (data[2] >> 4) * 0x1000
        pa = pa + offset

        va_l = (data[1] >> 2)
        va_h = (data[2]) & 0x7
        va = (va_h << 45) | (va_l << 16)
        va = va + offset

        if ((va >> 40) & 0xff )== 0xff:
            va = va + 0xffff000000000000

        valid = (data[2] >> 3) & 0x1

        vmid_1 = (data[0] >> 26) & 0x3f
        vmid_2 = data[1] & 0x3
        vmid = (vmid_2 << 6) | vmid_1

        asid = (data[0] >> 10) & 0xffff

        size = self.parse_size(data, ram, tlb_type)
        output.append(ram)
        output.append(tlb_type)
        output.append(pa)
        output.append(va)
        output.append(valid)
        output.append(vmid)
        output.append(asid)
        output.append(s1_mode)
        output.append(s1_level)
        output.append(size)

    def parse_tlb_type(self, data):
        type_num = (data[3] >> 20) & 0x1
        if type_num == 0x0:
            s1_level = data[0] & 0x3
            if s1_level == 0x3:
                return "IPA"
            else:
                return "REG"
        else:
            return "WALK"

    def parse_size(self, data, ram, tlb_type):
        size = (data[0] >> 6) & 0x7
        if tlb_type == "REG":
            if ram == 0:
                if size == 0x0:
                    return "4KB"
                elif size == 0x1:
                    return "16KB"
                else:
                    return "64KB"
            else:
                if size == 0x0:
                    return "1MB"
                elif size == 0x1:
                    return "2MB"
                elif size == 0x2:
                    return "16MB"
                elif size == 0x3:
                    return "32MB"
                elif size == 0x4:
                    return "512MB"
                else:
                    return "1GB"
        else:
                if size == 0x1:
                    return "4KB"
                elif size == 0x3:
                    return "16KB"
                elif size == 0x4:
                    return "64KB"

class L1_TLB_A53(TlbDumpType_v1):
    def __init__(self):
        super(L1_TLB_A53, self).__init__()
        self.unsupported_header_offset = 0
        self.LineSize = 4
        self.NumSets = 0x100
        self.NumWays = 4

class L2_TLB_KRYO3XX_GOLD(TlbDumpType_v2):
    def __init__(self):
        super(L2_TLB_KRYO3XX_GOLD, self).__init__()
        self.tableformat.addColumn('RAM')
        self.tableformat.addColumn('TYPE')
        self.tableformat.addColumn('VA/IPA[48:16]', '{0:016x}', 16)
        self.tableformat.addColumn('PA[43:12]', '{0:016x}', 16)
        self.tableformat.addColumn('VALID')
        self.tableformat.addColumn('NS')
        self.tableformat.addColumn('VMID', '{0:02x}', 4)
        self.tableformat.addColumn('ASID', '{0:04x}', 4)
        self.tableformat.addColumn('S1_MODE', '{0:01x}', 7)
        self.tableformat.addColumn('S1_LEVEL', '{0:01x}', 8)
        self.tableformat.addColumn('TRANS_REGIME', '{0:01x}', 2)
        self.tableformat.addColumn('SIZE')
        self.unsupported_header_offset = 0
        self.LineSize = 5
        self.NumSetsRam0 =  0x100
        self.NumSetsRam1 =  0x80
        self.NumWaysRam0 = 4
        self.NumWaysRam1 = 2

    def parse_tag_fn(self, output, data, nset, nway, ram, offset):
        tlb_type = self.parse_tlb_type(data)
        if ram == 0:
            tlb_type = "REG"
        s1_mode = (data[0] >> 2) & 0x3

        s1_level = data[0] & 0x3

        pa = data[3] & 0xffffffff
        pa = pa << 12

        va_l = (data[1] >> 10)
        va_h = data[2] & 0x3ff
        va = (va_h << 22) | va_l

        #Add set to the VA
        if ram == 0:
            va = va >> 4
            va = va << 8
            va = va + nset
            va = va << 12
        else:
            va = va >> 3
            va = va << 3
            va = va << 4
            va = va + nset
            va = va << 12
        if (va >> 40) == 0xff:
            va = va + 0xffff000000000000

        valid = (data[2] >> 11) & 0x1

        ns = data[4] & 0x1
        vmid_1 = (data[0] >> 26)
        vmid_2 = data[1] & 0x3ff
        vmid = (vmid_2 << 6) | vmid_1

        asid = (data[0] >> 10) & 0xffff

        regime = (data[0] >> 4) & 0x3

        size = self.parse_size(data, ram, tlb_type)
        output.append(ram)
        output.append(tlb_type)
        output.append(va)
        output.append(pa)
        output.append(valid)
        output.append(ns)
        output.append(vmid)
        output.append(asid)
        output.append(s1_mode)
        output.append(s1_level)
        output.append(regime)
        output.append(size)

    def parse_tlb_type(self, data):
        type_num = (data[4] >> 17) & 0x1
        if type_num == 0x0:
            s1_level = data[0] & 0x3
            s1_mode = (data[0] >> 2) & 0x3
            if s1_mode == 0x1 and s1_level == 0x2:
                return "IPA"
            else:
                return "REG"
        else:
            return "WALK"

    def parse_size(self, data, ram, tlb_type):
        size = (data[0] >> 6) & 0x7
        if ram == 0:
            if size == 0x0:
                return "4KB"
            elif size == 0x1:
                return "16KB"
            else:
                return "64KB"
        else:
            if size == 0x0:
                return "1MB"
            elif size == 0x1:
                return "2MB"
            elif size == 0x2:
                return "16MB"
            elif size == 0x3:
                return "32MB"
            elif size == 0x4:
                return "512MB"
            else:
                return "1GB"


class L2_TLB_KRYOBXX_SILVER(TlbDumpType_v1):
    def __init__(self):
        super(L2_TLB_KRYOBXX_SILVER, self).__init__()
        self.cpu_name = "KryoBSilver"

    def parse_dump(self):
        datafile_name = "data_scratch" + self.outfile.name[-4:]
        data_offset = 0
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_DATA", data_offset,\
                                datafile_name)

class L2_TLB_KRYO3XX_SILVER(TlbDumpType_v1):
    def __init__(self):
        super(L2_TLB_KRYO3XX_SILVER, self).__init__()
        self.cpu_name = "Kryo3Silver"
        self.NumWays = 4
        self.NumSets = 0x100
        self.NumTagRegs = 3
        self.RegSize = 4
        self.tlbsubcache = []
        self.datasubcache = []
        #represet subcache in this form ['name', set, ways, blocksize]
        self.tlbsubcache.append(["CACHEDUMP_CACHE_ID_L2_TLB_TAG_WALK",16, 4, 12])
        self.tlbsubcache.append([ "CACHEDUMP_CACHE_ID_L2_TLB_TAG_IPA",16 , 4, 12])
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L2_TLB_DATA_WALK", 16, 4, 8])
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L2_TLB_DATA_IPA",16, 4, 8])

    def parse_dump(self):
        tagfile_name = "tag_scratch" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_TAG", 0, tagfile_name)

        datafile_name = "data_scratch" + self.outfile.name[-4:]
        """the input file is the dump for this TLB, and this is divided into
           two parts: the tag contents for all of the TLB, followed by the data
           contents for all of the TLB. As such, you must calculate the size of
           the tag content for the TLB to get the offset into the dump where the
           data contents start."""
        data_offset = 0
        if self.tlbsubcache:
            for x in self.tlbsubcache:
                data_offset = data_offset + (int(x[1]) * int(x[2]) * int(x[3]))
            data_offset = data_offset + self.NumWays * self.NumSets * self.RegSize *\
                          self.NumTagRegs
            self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_DATA", data_offset,\
                                datafile_name)


class L1_ITLB_KRYO4XX_GOLD(TlbDumpType_v3):
    def __init__(self):
        super(L1_ITLB_KRYO4XX_GOLD, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "KRYO4GOLD"
        self.tableformat = TableOutputFormat()
        self.tableformat.addColumn('Set', '{0:03x}', 3)
        self.tableformat.addColumn('Way', '{0:01x}', 1)
        self.tableformat.addColumn('Valid', '{0:01x}', 1)
        self.tableformat.addColumn('Attr1', '{0:01x}', 1)
        self.tableformat.addColumn('TxlnRegime', '{0:01x}', 1)
        self.tableformat.addColumn('VMID', '{0:04x}', 4)
        self.tableformat.addColumn('ASID', '{0:04x}', 4)
        self.tableformat.addColumn('Attr2', '{0:02x}', 2)
        self.tableformat.addColumn('InnerShared', '{0:01x}', 1)
        self.tableformat.addColumn('OuterShared', '{0:01x}', 1)
        self.tableformat.addColumn('Attr3', '{0:01x}', 1)
        self.tableformat.addColumn('PageSize', '{0:01x}', 1)
        self.tableformat.addColumn('MemAttr', '{0:01x}', 1)
        self.tableformat.addColumn('Attr4', '{0:01x}', 1)
        self.tableformat.addColumn('PBHA', '{0:01x}', 1)
        self.tableformat.addColumn('VA', '{0:016x}', 16)
        self.tableformat.addColumn('PA', '{0:08x}', 8)
        self.tableformat.addColumn('NS', '{0:01x}', 1)

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_I_TLB", 0, datafile_name)


class L1_DTLB_KRYO4XX_GOLD(TlbDumpType_v3):
    def __init__(self):
        super(L1_DTLB_KRYO4XX_GOLD, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "KRYO4GOLD"
        self.tableformat = TableOutputFormat()
        self.tableformat.addColumn('Set', '{0:03x}', 3)
        self.tableformat.addColumn('Way', '{0:01x}', 1)
        self.tableformat.addColumn('Valid', '{0:01x}', 1)
        self.tableformat.addColumn('VMID', '{0:04x}', 4)
        self.tableformat.addColumn('ASID', '{0:04x}', 4)
        self.tableformat.addColumn('TxlnRegime', '{0:01x}', 1)
        self.tableformat.addColumn('NS', '{0:01x}', 1)
        self.tableformat.addColumn('PageSize', '{0:01x}', 1)
        self.tableformat.addColumn('MemAttr', '{0:01x}', 1)
        self.tableformat.addColumn('InnerShared', '{0:01x}', 1)
        self.tableformat.addColumn('OuterShared', '{0:01x}', 1)
        self.tableformat.addColumn('VA', '{0:016x}', 16)
        self.tableformat.addColumn('PA', '{0:08x}', 8)
        self.tableformat.addColumn('PBHA', '{0:01x}', 1)

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_D_TLB", 0, datafile_name)

class L2_TLB_KRYO4XX_SILVER(TlbDumpType_v3):
    def __init__(self):
        super(L2_TLB_KRYO4XX_SILVER, self).__init__()
        self.cpu_name = "KRYO4SILVER"
        self.tableformat = TableOutputFormat()
        self.tableformat.addColumn('Set', '{0:03x}', 3)
        self.tableformat.addColumn('Way', '{0:01x}', 1)
        self.tableformat.addColumn('Valid', '{0:01x}', 1)
        self.tableformat.addColumn('NS', '{0:01x}', 1)
        self.tableformat.addColumn('ASID', '{0:04x}', 4)
        self.tableformat.addColumn('VMID', '{0:04x}', 4)
        self.tableformat.addColumn('size', '{0:01x}', 1)
        self.tableformat.addColumn('nG', '{0:01x}', 1)
        self.tableformat.addColumn('APHyp', '{0:01x}', 1)
        self.tableformat.addColumn('S2AP', '{0:01x}', 1)
        self.tableformat.addColumn('Dom', '{0:01x}', 1)
        self.tableformat.addColumn('S1Size', '{0:01x}', 1)
        self.tableformat.addColumn('AddrSignBit', '{0:01x}', 1)
        self.tableformat.addColumn('VA', '{0:08x}', 8)
        self.tableformat.addColumn('DBM', '{0:01x}', 1)
        self.tableformat.addColumn('Parity', '{0:01x}', 1)
        self.tableformat.addColumn('XS1Usr', '{0:01x}', 1)
        self.tableformat.addColumn('XS1NonUsr', '{0:01x}', 1)
        self.tableformat.addColumn('XS2Usr', '{0:01x}', 1)
        self.tableformat.addColumn('XS2NonUsr', '{0:01x}', 1)
        self.tableformat.addColumn('MemTypeAndShareability', '{0:02x}', 2)
        self.tableformat.addColumn('S2Level', '{0:01x}', 1)
        self.tableformat.addColumn('NS', '{0:01x}', 1)
        self.tableformat.addColumn('PA', '{0:08x}', 8)
        self.tableformat.addColumn('Parity', '{0:01x}', 1)
        #refer to section A5.2.2 in TRM
        self.NumWays = 4
        self.NumSets = 0x100
        #refer to new src for dumping tag data to see number of tag entries
        self.NumTagRegs = 3
        #refer to new src for dumping tag data to see size. Use bytes
        self.RegSize = 4

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_TAG_DATA", 0, tagfile_name)


class L2_TLB_KRYO4XX_GOLD(TlbDumpType_v3):
    def __init__(self):
        super(L2_TLB_KRYO4XX_GOLD, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "KRYO4GOLD"
        self.tableformat = TableOutputFormat()
        self.tableformat.addColumn('Set', '{0:03x}', 3)
        self.tableformat.addColumn('Way', '{0:01x}', 1)
        self.tableformat.addColumn('Valid(0x)', '{0:01x}', 1)
        self.tableformat.addColumn('Coalesced', '{0:01x}', 1)
        self.tableformat.addColumn('PageSize(0x)', '{0:01x}', 1)
        self.tableformat.addColumn('PA(0x)', '{0:08x}', 8)
        self.tableformat.addColumn('MemAttr', '{0:01x}', 1)
        self.tableformat.addColumn('InnerShared', '{0:01x}', 1)
        self.tableformat.addColumn('OuterShared', '{0:01x}', 1)
        self.tableformat.addColumn('nonGlobal', '{0:01x}', 1)
        self.tableformat.addColumn('NS', '{0:01x}', 1)
        self.tableformat.addColumn('VA(0x)', '{0:08x}', 8)
        self.tableformat.addColumn('Prefetched(0x)', '{0:01x}', 1)
        self.tableformat.addColumn('walkCache(0x)', '{0:01x}', 1)
        self.tableformat.addColumn('PBHA(0x)', '{0:01x}', 1)
        self.tableformat.addColumn('ASID(0x)', '{0:04x}', 4)
        self.tableformat.addColumn('VMID(0x)', '{0:04x}', 4)
        self.tableformat.addColumn('TxlnRegime(0x)', '{0:01x}', 1)

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TAG_DATA", 0, datafile_name)


if os.path.exists(os.path.join(os.path.dirname(__file__), 'extensions', 'cache_tlb_config.py')):
    import extensions.cache_tlb_config

#msmnile
lookuptable[("msmnile", 0x24, 0x14)] = L1_ITLB_KRYO4XX_GOLD()
lookuptable[("msmnile", 0x25, 0x14)] = L1_ITLB_KRYO4XX_GOLD()
lookuptable[("msmnile", 0x26, 0x14)] = L1_ITLB_KRYO4XX_GOLD()
lookuptable[("msmnile", 0x27, 0x14)] = L1_ITLB_KRYO4XX_GOLD()

lookuptable[("msmnile", 0x44, 0x14)] = L1_DTLB_KRYO4XX_GOLD()
lookuptable[("msmnile", 0x45, 0x14)] = L1_DTLB_KRYO4XX_GOLD()
lookuptable[("msmnile", 0x46, 0x14)] = L1_DTLB_KRYO4XX_GOLD()
lookuptable[("msmnile", 0x47, 0x14)] = L1_DTLB_KRYO4XX_GOLD()

lookuptable[("msmnile", 0x120, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("msmnile", 0x121, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("msmnile", 0x122, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("msmnile", 0x123, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("msmnile", 0x124, 0x14)] = L2_TLB_KRYO4XX_GOLD();
lookuptable[("msmnile", 0x125, 0x14)] = L2_TLB_KRYO4XX_GOLD();
lookuptable[("msmnile", 0x126, 0x14)] = L2_TLB_KRYO4XX_GOLD();
lookuptable[("msmnile", 0x127, 0x14)] = L2_TLB_KRYO4XX_GOLD();

#steppe
lookuptable[("steppe", 0x26, 0x14)] = L1_ITLB_KRYO4XX_GOLD()
lookuptable[("steppe", 0x27, 0x14)] = L1_ITLB_KRYO4XX_GOLD()

lookuptable[("steppe", 0x46, 0x14)] = L1_DTLB_KRYO4XX_GOLD()
lookuptable[("steppe", 0x47, 0x14)] = L1_DTLB_KRYO4XX_GOLD()

lookuptable[("steppe", 0x120, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("steppe", 0x121, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("steppe", 0x122, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("steppe", 0x123, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("steppe", 0x124, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("steppe", 0x125, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("steppe", 0x126, 0x14)] = L2_TLB_KRYO4XX_GOLD()
lookuptable[("steppe", 0x127, 0x14)] = L2_TLB_KRYO4XX_GOLD()

#lito
lookuptable[("lito", 0x26, 0x14)] = L1_ITLB_KRYO4XX_GOLD()
lookuptable[("lito", 0x27, 0x14)] = L1_ITLB_KRYO4XX_GOLD()

lookuptable[("lito", 0x46, 0x14)] = L1_DTLB_KRYO4XX_GOLD()
lookuptable[("lito", 0x47, 0x14)] = L1_DTLB_KRYO4XX_GOLD()

lookuptable[("lito", 0x120, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("lito", 0x121, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("lito", 0x122, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("lito", 0x123, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("lito", 0x124, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("lito", 0x125, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("lito", 0x126, 0x14)] = L2_TLB_KRYO4XX_GOLD()
lookuptable[("lito", 0x127, 0x14)] = L2_TLB_KRYO4XX_GOLD()

#atoll
lookuptable[("atoll", 0x26, 0x14)] = L1_ITLB_KRYO4XX_GOLD()
lookuptable[("atoll", 0x27, 0x14)] = L1_ITLB_KRYO4XX_GOLD()

lookuptable[("atoll", 0x46, 0x14)] = L1_DTLB_KRYO4XX_GOLD()
lookuptable[("atoll", 0x47, 0x14)] = L1_DTLB_KRYO4XX_GOLD()

lookuptable[("atoll", 0x120, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("atoll", 0x121, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("atoll", 0x122, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("atoll", 0x123, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("atoll", 0x124, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("atoll", 0x125, 0x14)] = L2_TLB_KRYO4XX_SILVER()
lookuptable[("atoll", 0x126, 0x14)] = L2_TLB_KRYO4XX_GOLD()
lookuptable[("atoll", 0x127, 0x14)] = L2_TLB_KRYO4XX_GOLD()

# "sdm845"
lookuptable[("sdm845", 0x120, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm845", 0x121, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm845", 0x122, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm845", 0x123, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm845", 0x124, 0x14)] = L2_TLB_KRYO3XX_GOLD()
lookuptable[("sdm845", 0x125, 0x14)] = L2_TLB_KRYO3XX_GOLD()
lookuptable[("sdm845", 0x126, 0x14)] = L2_TLB_KRYO3XX_GOLD()
lookuptable[("sdm845", 0x127, 0x14)] = L2_TLB_KRYO3XX_GOLD()

# "sdm710"
lookuptable[("sdm710", 0x120, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm710", 0x121, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm710", 0x122, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm710", 0x123, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm710", 0x124, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm710", 0x125, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("sdm710", 0x126, 0x14)] = L2_TLB_KRYO3XX_GOLD()
lookuptable[("sdm710", 0x127, 0x14)] = L2_TLB_KRYO3XX_GOLD()

# "qcs605"
lookuptable[("qcs605", 0x120, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("qcs605", 0x121, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("qcs605", 0x122, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("qcs605", 0x123, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("qcs605", 0x124, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("qcs605", 0x125, 0x14)] = L2_TLB_KRYO3XX_SILVER()
lookuptable[("qcs605", 0x126, 0x14)] = L2_TLB_KRYO3XX_GOLD()
lookuptable[("qcs605", 0x127, 0x14)] = L2_TLB_KRYO3XX_GOLD()

# "trinket"
lookuptable[("trinket", 0x20, 0x14)] = L1_TLB_A53()
lookuptable[("trinket", 0x21, 0x14)] = L1_TLB_A53()
lookuptable[("trinket", 0x22, 0x14)] = L1_TLB_A53()
lookuptable[("trinket", 0x23, 0x14)] = L1_TLB_A53()
lookuptable[("trinket", 0x24, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("trinket", 0x25, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("trinket", 0x26, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("trinket", 0x27, 0x14)] = L1_TLB_KRYO2XX_GOLD()

# "msm8998"
lookuptable[("8998", 0x20, 0x14)] = L1_TLB_A53()
lookuptable[("8998", 0x21, 0x14)] = L1_TLB_A53()
lookuptable[("8998", 0x22, 0x14)] = L1_TLB_A53()
lookuptable[("8998", 0x23, 0x14)] = L1_TLB_A53()
lookuptable[("8998", 0x24, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("8998", 0x25, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("8998", 0x26, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("8998", 0x27, 0x14)] = L1_TLB_KRYO2XX_GOLD()


lookuptable[("8998", 0x40, 0x14)] = L1_TLB_A53()
lookuptable[("8998", 0x41, 0x14)] = L1_TLB_A53()
lookuptable[("8998", 0x42, 0x14)] = L1_TLB_A53()
lookuptable[("8998", 0x43, 0x14)] = L1_TLB_A53()
lookuptable[("8998", 0x44, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("8998", 0x45, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("8998", 0x46, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("8998", 0x47, 0x14)] = L1_TLB_KRYO2XX_GOLD()

# "bengal"
lookuptable[("bengal", 0x120, 0x14)] = L2_TLB_KRYOBXX_SILVER()
lookuptable[("bengal", 0x121, 0x14)] = L2_TLB_KRYOBXX_SILVER()
lookuptable[("bengal", 0x122, 0x14)] = L2_TLB_KRYOBXX_SILVER()
lookuptable[("bengal", 0x123, 0x14)] = L2_TLB_KRYOBXX_SILVER()
lookuptable[("bengal", 0x124, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("bengal", 0x125, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("bengal", 0x126, 0x14)] = L1_TLB_KRYO2XX_GOLD()
lookuptable[("bengal", 0x127, 0x14)] = L1_TLB_KRYO2XX_GOLD()

# "scuba"
lookuptable[("scuba", 0x120, 0x14)] = L2_TLB_KRYOBXX_SILVER()
lookuptable[("scuba", 0x121, 0x14)] = L2_TLB_KRYOBXX_SILVER()
lookuptable[("scuba", 0x122, 0x14)] = L2_TLB_KRYOBXX_SILVER()
lookuptable[("scuba", 0x123, 0x14)] = L2_TLB_KRYOBXX_SILVER()
