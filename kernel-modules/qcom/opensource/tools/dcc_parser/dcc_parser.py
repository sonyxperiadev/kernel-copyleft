# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
# Copyright (c) 2015, 2017, 2019-2021 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import datetime
import logging
import logging.handlers
import os
import struct
import sys
import re

from optparse import OptionParser

count = 0
address = []
data = []
dcc_sink = []
next_ll_offset = []
MAX_LOOP_COUNT = 4096

def bm(msb, lsb):
    'Creates a bitmask from msb to lsb'
    return int(('1' * (msb - lsb + 1)) + ('0' * lsb), 2)


def bvalsel(msb, lsb, val):
    'Masks and returns the bits from msb to lsb in val'
    return ((val & bm(msb, lsb)) >> lsb)

def log_init(name, path, filename):
    # Set up logger
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)

    # Add the log message handler to the logger
    if filename is not None:
        handler = logging.FileHandler(path + '/' + filename, mode='w')
    else:
        handler = logging.StreamHandler(sys.stdout)

    logger.addHandler(handler)
    return logger


def add_addr(base, offset, length):
    for i in range(0, length):
        addr = base + offset + (i * 4)
        address.append(addr)


def add_loop_addr(loop_nr, loop_count):
    for i in range(0, loop_count):
        for j in range(len(address) - loop_nr, len(address)):
            address.append(address[j])


def read_data(data_pt):
    nr = count
    while nr > 0:
        word = data_pt.read(4)
        if len(word) != 4:
            break

        val = struct.unpack('<L', word)[0]
        data.append(val)
        nr = nr - 1
    next_ll_offset.append(hex(data_pt.tell()))
    return nr


list_nr = []

def read_config(config_pt):
    offset = 0
    base = 0
    length = 1
    list_nr.append(0)
    count = 0
    if options.version is None:
        address_descriptor = 0x1 << 31
        link_descriptor = 0
        loop_descriptor = None
        rd_mod_wr_descriptor = None
        dcc_write_ind = None
        link_second_arg = 8
        # We return zero and fail
        on_zero_link_len = 0
        track_len = 0
        empty_ind = 0x0
    else:
        address_descriptor = 0
        link_descriptor = 0x3 << 30
        loop_descriptor = 0x1 << 30
        rd_mod_wr_descriptor = 0x1 << 31
        dcc_write_ind = 0x1 << 28
        link_second_arg = 7
        #indicates end of list
        on_zero_link_len = -1
        #word size
        track_len = 4
        #empty SRAM is filled with 0xdededede
        empty_ind = 0xdededede

    if options.config_loopoffset is not None:
        config_loopoffset = int(options.config_loopoffset)
    else:
        config_loopoffset = 13

    while True:
        word = config_pt.read(4)
        if len(word) != 4:
            break

        val = struct.unpack('<L', word)[0]
        if val == 0:
            break

        descriptor = val & (0x3 << 30)
        read_write_ind = val & (0x1 << 28)

        if val == empty_ind:
            continue
        elif descriptor == address_descriptor:
            if read_write_ind == dcc_write_ind:
                config_pt.seek(8, 1)
            else:
                base = ((val & 0x0FFFFFFF) << 4)
                offset = 0
                length = 1
                tmp_count = 0
        elif descriptor == link_descriptor:
            total_length = 0
            for i in range(0, 2):
                offset = offset + (val & 0xFF) * 4 + (length - 1) * track_len
                val = val >> 8

                length = (val & 0x7f)
                val = val >> link_second_arg
                if length != 0:
                    total_length += length
                    count = count + 1
                    tmp_count = tmp_count + 1
                    add_addr(base, offset, length)
                else:
                    if (i == 0 ):
                        return list_nr[on_zero_link_len]
                    else:
                        offset = 0
            if total_length > 0:
                list_nr.append(total_length + list_nr[- 1])
        elif descriptor == loop_descriptor:
            loop_offset = val & bm(config_loopoffset - 1, 0)
            loop_count = bvalsel(27, config_loopoffset, val)
            if loop_count > MAX_LOOP_COUNT:
                print ("loop offset is wrong or SRAM is corrupted\n")
                exit()

            if loop_offset == 0:
                continue
            try:
                loop_nr = list_nr[-1] - list_nr[-loop_offset//2 - 1]
                list_nr.append(loop_nr * loop_count + list_nr[-1])
                count = count + 1
                add_loop_addr(loop_nr, loop_count)
            except Exception as err:
                pass
        elif descriptor == rd_mod_wr_descriptor:
            '''
            Skip over mask and value of rd_mod_wr.
            There is no gaurantee of this being actually written
            and we never read the value back to confirm.
            Remove address added by previous entry since
            that's for rd_mod_wr instead of read operation.
            '''
            for i in range(0, tmp_count):
                if (val & 1 == 0): # check for Keep bit
                    last_length = list_nr[-1] - list_nr[-2]
                    list_nr.pop()
                    count = count - 1
                    for j in range(0, last_length):
                       address.pop()

            config_pt.seek(8, 1)

    return count


def new_linked_list(config_pt):
    word = config_pt.read(4)

    if len(word)!= 4:
        return False
    else:
        val = struct.unpack('<L', word)[0]
        if val != 0:
            config_pt.seek(-4, 1)
            return True
        else:
            return False


def dump_regs_json(options):
    log.info("Dumping regs in JSON format in \'{0}\' file.".format(options.outfile))
    parsed_data.info("{")
    parsed_data.info("\t\"version\": 1,")
    parsed_data.info("\t\"timestamp\": \"{0}\",".format(datetime.date.today().strftime('%m/%d/%y')))
    parsed_data.info("\t\"generator\": \"Linux DCC Parser\",")
    parsed_data.info("\t\"chip\": {")
    parsed_data.info("\t\t\"name\": \"{0}\",".format(options.chipname))
    parsed_data.info("\t\t\"version\": \"{0}\"".format(options.chipversion))
    parsed_data.info("\t},")
    parsed_data.info("\t\"registers\": [")
    for addr, val in zip(address, data):
        parsed_data.info("\t\t{{ \"address\": \"0x{0:08x}\", \"value\": \"0x{1:08x}\" }},".format(addr, val))
    parsed_data.info("\t]")
    parsed_data.info("}")
    return


def dump_regs_xml(options):
    log.info("Dumping regs in XML format in \'{0}\' file.".format(options.outfile))
    parsed_data.info("<?xml version=\"1.0\" encoding=\"utf-8\"?>")
    parsed_data.info("<hwioDump version=\"1\">")
    parsed_data.info("\t<timestamp>{0}</timestamp>".format(datetime.date.today().strftime('%m/%d/%y')))
    parsed_data.info("\t<generator>Linux DCC Parser</generator>")
    parsed_data.info("\t<chip name=\"{0}\" version=\"{1}\">".format(options.chipname, options.chipversion))
    for addr, val in zip(address, data):
        parsed_data.info("\t\t<register address=\"0x{0:08x}\" value=\"0x{1:08x}\" />".format(addr, val))
    parsed_data.info("\t</chip>")
    try:
        parsed_data.info("\t<next_ll_offset>next_ll_offset : {0} </next_ll_offset>".format(next_ll_offset[-1]))
    except:
        pass
    parsed_data.info("</hwioDump>")
    return


def dump_regs(options):
    if not address:
        log.error('No configuration found in SRAM!!')
        sys.exit(1)

    if options.json is True:
        dump_regs_json(options)
    else:
        dump_regs_xml(options)


def read_data_atb(atb_data_pt, count):
    atb_count = 0
    for line in atb_data_pt:
        if "ATID\" : 65, \"OpCode\" : \"D8\"" in line:
            data1 = ""
            i = 0
            while i < 4:
                data_byte_re = re.match(
                    "\{\"ATID\" : 65, \"OpCode\" : \"D8\", \"Payload\" : "
                    "\"0x([0-9A-Fa-f][0-9A-Fa-f])\"\}", line)
                if data_byte_re:
                    data1 = (data_byte_re.group(1)) + data1
                    i += 1
                else:
                    log.error("ATB file format wrong")
                    return atb_count
                if i < 4:
                    line = atb_data_pt.next()
            data.append(int(data1, 16))
            atb_count = atb_count + 1
            if atb_count >= count:
                break
        elif "ATID\" : 65, \"OpCode\" : \"D32\"" in line:
            #print line
            data_byte_re = re.match(
                "\{\"ATID\" : 65, \"OpCode\" : \"D32\", \"Payload\" : "
                "\"0x([0-9A-Fa-f]+)\"}", line)
            if data_byte_re:
                data1 = (data_byte_re.group(1))
                #print data1
                data.append(int(data1, 16))
                atb_count = atb_count + 1
                if atb_count >= count:
                    break
    return atb_count

if __name__ == '__main__':
    usage = 'usage: %prog [options to print]. Run with --help for more details'
    parser = OptionParser(usage)
    parser.add_option('-s', '--sram-file', dest='sramfile',
                      help='sram image path')
    parser.add_option('-a', '--atb-file', dest='atbfile', help='atb image path')
    parser.add_option('-j', '--json', action='store_true',
                      help='output in JSON format')
    parser.add_option('-o', '--out-dir', dest='outdir', help='output dir path')
    parser.add_option('-f', '--output-file', dest='outfile',
                      help='output filename')
    parser.add_option('-l', '--log-file', dest='logfile', help='Log filename')
    parser.add_option('', '--chip-name', dest='chipname', help='chip name')
    parser.add_option('', '--chip-version', dest='chipversion',
                      help='chip version')
    parser.add_option('--v2', dest='version', action="store_const", const='2',
                      help='DCC driver version 2')
    parser.add_option('--config-offset', dest='config_offset',
                      help='Start offset for DCC configuration')
    parser.add_option('--data-offset', dest='data_offset',
                      help='Start offset for DCC Data')
    parser.add_option('--config-loopoffset', dest='config_loopoffset',
                      help='Offset of loop value')
    parser.add_option('--dcc_sink', dest='dcc_sink',
                      help='DCC sink(SRAM/ATB).Comma seperated list if more than one list used')

    (options, args) = parser.parse_args()

    args = ''
    for arg in sys.argv:
        args = args + arg + ' '

    if options.outdir:
        if not os.path.exists(options.outdir):
            print ('!!! Out directory does not exist. Creating...')
            try:
                os.makedirs(options.outdir)
            except:
                print ("Failed to create %s. You probably don't have permissions there. Bailing." % options.outdir)
                sys.exit(1)
    else:
        options.outdir = '.'

    if options.json:
        ext = '.json'
    else:
        ext = '.xml'

    if options.dcc_sink is None:
        dcc_sink.append('SRAM')
    else:
        dcc_sink = options.dcc_sink.split(',')
        print(dcc_sink)

    if options.outfile is None:
        options.outfile = 'dcc_captured_data{0}'.format(ext)

    log = log_init('LOG', options.outdir, options.logfile)

    log.info("Data Capture and Compare(DCC) parser.")

    if options.sramfile is None:
        log.error("No SRAM image file given! Exiting...")
        parser.print_usage()
        sys.exit(1)

    try:
        sram_file = open(options.sramfile, 'rb')
    except:
        log.error("could not open path {0}".format(options.sramfile))
        log.error("Do you have read permissions on the path?")
        sys.exit(1)

    count = 0

    if options.config_offset is not None:
        sram_file.seek(int(options.config_offset, 16), 1)
    parsed_data = log_init('PARSED_DATA', options.outdir, options.outfile)
    if options.atbfile is not None and os.path.exists(options.atbfile):
        atb_file = open(options.atbfile, 'r')

    for sink in dcc_sink:
        count = read_config(sram_file)
        print("Number of registers in list:" , count)
        print("Sink used for the list:" ,  sink)
        if sink == 'SRAM':
            print('Read data from SRAM')
            if options.data_offset is not None:
                sram_file.seek(int(options.data_offset, 16))
            if read_data(sram_file):
                log.error('Couldn\'t read complete data.')
                sys.exit(1)
        elif sink == 'ATB':
            print('Read data from ATB file')
            if options.atbfile is not None:
                try:
                    atb_count = read_data_atb(atb_file, count)
                    if atb_count < count:
                        del address[-count:]
                        del data[-atb_count:]
                        log.error("ATB file don't have complete DCC data")
                except:
                    log.error("could not read ATB data {0}".format(options.atbfile))
                    del address[-count:]
            else:
                log.error('ATB file not given')
                del address[-count:]
        if not new_linked_list(sram_file):
            log.error("Next list not available")
            break
    if options.atbfile is not None and os.path.exists(options.atbfile):
        atb_file.close()
    dump_regs(options)
    sram_file.close()
    sys.stderr.flush()
