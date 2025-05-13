 # Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 # Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 #
 # This program is free software; you can redistribute it and/or modify it
 # under the terms of the GNU General Public License version 2 as published by
 # the Free Software Foundation.
 #
 # This program is distributed in the hope that it will be useful, but WITHOUT
 # ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 # FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 # more details.
 #
 # You should have received a copy of the GNU General Public License along with
 # this program.  If not, see <http://www.gnu.org/licenses/>.

import argparse
import filecmp
import os
import re
import subprocess
import sys

def run_headers_install(verbose, gen_dir, headers_install, unifdef, prefix, h):
    if not h.startswith(prefix):
        print('error: expected prefix [%s] on header [%s]' % (prefix, h))
        return False

    out_h = os.path.join(gen_dir, h[len(prefix):])
    (out_h_dirname, out_h_basename) = os.path.split(out_h)
    env = os.environ.copy()
    env["LOC_UNIFDEF"] = unifdef
    cmd = ["sh", headers_install, h, out_h]

    if True:
        print('run_headers_install: cmd is %s' % cmd)

    result = subprocess.call(cmd, env=env)

    if result != 0:
        print('error: run_headers_install: cmd %s failed %d' % (cmd, result))
        return False
    return True

def gen_mm_drivers_headers(verbose, gen_dir, headers_install, unifdef, mm_drivers_include_uapi):
    error_count = 0
    for h in mm_drivers_include_uapi:
        mm_drivers_uapi_include_prefix = os.path.join(h.split('sync_fence/include/uapi')[0],
			'sync_fence', 'include', 'uapi') + os.sep
        if not run_headers_install(
                verbose, gen_dir, headers_install, unifdef,
                mm_drivers_uapi_include_prefix, h): error_count += 1
    return error_count

def main():
    """Parse command line arguments and perform top level control."""
    parser = argparse.ArgumentParser(
            description=__doc__,
            formatter_class=argparse.RawDescriptionHelpFormatter)

    # Arguments that apply to every invocation of this script.
    parser.add_argument(
            '--verbose', action='store_true',
            help='Print output that describes the workings of this script.')
    parser.add_argument(
            '--header_arch', required=True,
            help='The arch for which to generate headers.')
    parser.add_argument(
            '--gen_dir', required=True,
            help='Where to place the generated files.')
    parser.add_argument(
            '--mm_drivers_include_uapi', required=True, nargs='*',
            help='The list of techpack/*/include/uapi header files.')
    parser.add_argument(
            '--headers_install', required=True,
            help='The headers_install tool to process input headers.')
    parser.add_argument(
              '--unifdef',
              required=True,
              help='The unifdef tool used by headers_install.')

    args = parser.parse_args()

    if args.verbose:
        print('header_arch [%s]' % args.header_arch)
        print('gen_dir [%s]' % args.gen_dir)
        print('mm_drivers_include_uapi [%s]' % args.mm_drivers_include_uapi)
        print('headers_install [%s]' % args.headers_install)
        print('unifdef [%s]' % args.unifdef)

    return gen_mm_drivers_headers(args.verbose, args.gen_dir,
            args.headers_install, args.unifdef, args.mm_drivers_include_uapi)

if __name__ == '__main__':
    sys.exit(main())

