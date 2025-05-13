# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.

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

    if verbose:
        print('run_headers_install: cmd is %s' % cmd)

    result = subprocess.call(cmd, env=env)

    if result != 0:
        print('error: run_headers_install: cmd %s failed %d' % (cmd, result))
        return False
    return True

def gen_ipa_headers(verbose, gen_dir, headers_install, unifdef, ipa_include_uapi):
    error_count = 0
    for h in ipa_include_uapi:
        ipa_uapi_include_prefix = os.path.join(h.split('/include/uapi/')[0],
                                                 'include',
                                                 'uapi') + os.sep

        if not run_headers_install(
                verbose, gen_dir, headers_install, unifdef,
                ipa_uapi_include_prefix, h): error_count += 1
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
            '--gen_dir', required=True,
            help='Where to place the generated files.')
    parser.add_argument(
            '--ipa_include_uapi', required=True, nargs='*',
            help='The list of header files.')
    parser.add_argument(
            '--headers_install', required=True,
            help='The headers_install tool to process input headers.')
    parser.add_argument(
            '--unifdef',
            required=True,
            help='The unifdef tool used by headers_install.')

    args = parser.parse_args()

    if args.verbose:
        print('gen_dir [%s]' % args.gen_dir)
        print('ipa_include_uapi [%s]' % args.ipa_include_uapi)
        print('headers_install [%s]' % args.headers_install)
        print('unifdef [%s]' % args.unifdef)

    return gen_ipa_headers(args.verbose, args.gen_dir,
            args.headers_install, args.unifdef, args.ipa_include_uapi)

if __name__ == '__main__':
    sys.exit(main())
