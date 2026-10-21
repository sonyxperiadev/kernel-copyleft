# Copyright (c) 2012-2014, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
import sys
import traceback
from contextlib import contextmanager

out_file = sys.stdout

def flush_outfile():
    if out_file is None:
        sys.stdout.flush()
    else:
        out_file.flush()

def set_outfile(path):
    global out_file
    try:
        out_file = open(path, 'wt')
    except:
        print_out_str("could not open path {0}".format(path))
        print_out_str("Do you have write/read permissions on the path?")
        sys.exit(1)


def print_out_str(string):
    if out_file is None:
        print (string)
    else:
        out_file.write(string + '\n')

def print_out_exception():
    if out_file is None:
        traceback.print_exc(file=sys.stdout)
    else:
        traceback.print_exc(file=out_file)

@contextmanager
def print_out_section(header):
    begin_header_string = '{0}begin {1}{0}'.format(
        10 * '-', header
    )
    end_header_string = '{0}end {1}{2}'.format(
        12 * '-',
        header,
        10 * '-',
    )
    print_out_str('\n' + begin_header_string)
    yield
    print_out_str(end_header_string + '\n')
