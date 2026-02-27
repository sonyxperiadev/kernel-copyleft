#!/usr/bin/env python3

# Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# this script requires python2. However, we'd like to be able to print
# an informative message to a user who might be unknowingly running
# python3 so we can't allow any python2 print statements to sneak in
# since they result in syntax errors in python3. By importing
# print_function we are requiring ourselves to use the python3 syntax.
from __future__ import print_function

import sys
import os
import re
import time
import platform
import textwrap
from optparse import OptionParser

import parser_util
from ramdump import RamDump
from print_out import print_out_str, set_outfile, print_out_section, print_out_exception, flush_outfile
from sched_info import verify_active_cpus
# Please update version when something is changed!'
VERSION = '2.0'
# Requires Python 3.5 or newer.
LRDP_PYTHON_VERSION_MAJOR = 3
LRDP_PYTHON_VERSION_MINOR = 5

# quick check of system requirements:
major, minor = sys.version_info[:2]

if (major, minor) < (LRDP_PYTHON_VERSION_MAJOR, LRDP_PYTHON_VERSION_MINOR):
    print("This script requires python >=3.5 to run!\n")
    print("You seem to be running: " + sys.version)
    print()
    sys.exit(1)

def parse_ram_file(option, opt_str, value, parser):
    a = getattr(parser.values, option.dest)
    if a is None:
        a = []
    temp = []
    for arg in parser.rargs:
        if arg[:2] == '--':
            break
        if arg[:1] == '-' and len(arg) > 1:
            break
        temp.append(arg)

    if len(temp) != 3:
        raise OptionValueError(
            "Ram files must be specified in 'name, start, end' format")

    a.append((temp[0], int(temp[1], 16), int(temp[2], 16)))
    setattr(parser.values, option.dest, a)


def get_ftrace_args(option, opt, value, parser):
    setattr(parser.values, option.dest, value.split(','))


if __name__ == '__main__':
    starttime = time.time()
    usage = 'usage: %prog [options to print]. Run with --help for more details'
    parser = OptionParser(usage)
    parser.add_option('', '--print-watchdog-time', action='store_true',
                      dest='watchdog_time', help='Print watchdog timing information', default=False)
    parser.add_option('', '--logcat_limit_time_sec',
                      dest='logcat_limit_time', type='int', default=0,
                      help='Defined the max time logcat parse running')
    parser.add_option('-e', '--ram-file', dest='ram_addr',
                      help='List of ram files (name, start, end)', action='callback', callback=parse_ram_file)
    parser.add_option('-v', '--vmlinux', dest='vmlinux', help='vmlinux path')
    parser.add_option('-n', '--nm-path', dest='nm', help='nm path')
    parser.add_option('-g', '--gdb-path', dest='gdb', help='gdb path')
    parser.add_option('-j', '--objdump-path', dest='objdump', help='objdump path')
    parser.add_option('-a', '--auto-dump', dest='autodump',
                      help='Auto find ram dumps from the path')
    parser.add_option('-o', '--outdir', dest='outdir', help='Output directory')
    parser.add_option('-s', '--t32launcher', action='store_true',
                      dest='t32launcher', help='Create T32 simulator launcher', default=False)
    parser.add_option('', '--t32-host-system', dest='t32_host_system',
                      metavar='HOST', choices=('Linux', 'Windows'),
                      help='T32 host system (for launcher script generation). Supported choices: "Linux", "Windows". Defaults to the system ramparse.py is running on.')
    parser.add_option('-x', '--everything', action='store_true',
                      dest='everything', help='Output everything (may be slow')
    parser.add_option('-f', '--output-file', dest='outfile',
                      help='Name of file to save output')
    parser.add_option('', '--stdout', action='store_true',
                      dest='stdout', help='Dump to stdout instead of the file')
    parser.add_option('', '--phys-offset', type='int',
                      dest='phys_offset', help='use custom phys offset')
    parser.add_option('', '--kaslr-offset', type='int',
                      dest='kaslr_offset',
                      help='Offset for address space layout randomization')
    parser.add_option('', '--page-offset', type='int',
                      dest='page_offset', help='use custom page offset')
    parser.add_option('', '--force-hardware',
                      dest='force_hardware', help='Force the hardware detection')
    parser.add_option(
        '', '--force-version', type='int', dest='force_hardware_version',
        help='Force the hardware detection to a specific hardware version')
    parser.add_option('-l', '--kernel-exception-level', type='int',
                      dest='currentEL', help='Current exception level for kernel')
    parser.add_option('', '--parse-qdss', action='store_true',
                      dest='qdss', help='Parse QDSS (deprecated)')
    parser.add_option('', '--64-bit', action='store_true', dest='arm64',
                      help='Parse dumps as 64-bit dumps (default)')
    parser.add_option('', '--32-bit', action='store_true', dest='arm32',
                      help='Parse dumps as 32-bit dumps')
    parser.add_option('', '--shell', action='store_true',
                      help='Run an interactive python interpreter with the ramdump loaded')
    parser.add_option('', '--classic-shell', action='store_true',
                      help='Like --shell, but forces the use of the classic python shell, even if ipython is installed')
    parser.add_option('', '--qtf', action='store_true', dest='ftrace_format',
                      help='Use QTF tool to parse and save QDSS trace data')
    parser.add_option('', '--ftrace-formats', action='store_true', dest='ftrace_format',
                      help='Extracts formats.txt used for generating ftrace')
    parser.add_option('', '--qtf-path', dest='qtf_path',
                      help='QTF tool executable')
    parser.add_option('', '--skip-qdss-bin', action='store_true',
                      dest='skip_qdss_bin', help='Skip QDSS ETF and ETR '
                      'binary data parsing from debug image (may save time '
                      'if large ETM and ETR buffers are present)')
    parser.add_option('', '--eval',
                      help='Evaluate some python code directly, or from stdin if "-" is passed. The "dump" variable will be available, as it is with the --shell option.')  # noqa
    parser.add_option('', '--wlan', dest='wlan', help='wlan.ko path')
    parser.add_option('', '--minidump', action='store_true', dest='minidump',
                      help='Parse minidump')
    # Adding option for reduced dump
    parser.add_option('', '--reduceddump', action='store_true', dest='reduceddump',
                  help='Parse reduceddump')
    parser.add_option('', '--svm', default='', dest='svm',action='store',type="string",
                      help='Parse svm')
    parser.add_option('', '--ram-elf', dest='ram_elf_addr',
                      help='pass ap_minidump.elf generated by crashscope')
    parser.add_option('-m', '--mod_path', action='append', dest='mod_path_list', help='Symbol path to all loadable module ".ko.unstripped" files. Files will be searched for recursively in sub-directories. Multiple paths can be provided using -m <path> -m <path> ...')
    parser.add_option('', '--timeout', dest='timeout', help='symbol path to all loadable modules')
    parser.add_option('', '--dump_mod_sym_tbl', action='store_true', dest='dump_module_symbol_table',
                      help='dump all symbols within a module whose symbol file is available.', default=False)
    parser.add_option('', '--dump_krnl_sym_tbl', action='store_true', dest='dump_kernel_symbol_table',
                      help='dump all symbols within the Linux kernel', default=False)
    parser.add_option('', '--dump_mod_kallsyms', action='store_true', dest='dump_module_kallsyms',
                      help='dump all symbols retrieved from kallsyms of a specific module', default=False)
    parser.add_option('', '--dump_glb_sym_tbl', action='store_true', dest='dump_global_symbol_table',
                      help='dump all symbols within the global symbol lookup table', default=False)
    parser.add_option('', '--hyp', dest='hyp', help='elf file containing hypervisor symbol information. Required for --svm')
    parser.add_option('--dbg', '--debug', action='store_true', dest='debug',
                      help=textwrap.dedent("""\
                      Enable debug.
                      Stop ignoring exceptions from running a parser. Intended to be used with:
                      python -m pdb ramparse.py <args for ramparse.py>
                      """))
    parser.add_option('', '--ftrace-args', type='string', action='callback',
                          callback=get_ftrace_args, dest = 'ftrace_args',
                          help="""
                          Comma separated trace subsystems to capture. Works with --dump-ftrace parser option
                          trace_names = [
                                         "binder", "bootreceiver", "clock_reg",
                                         "kgsl-fence", "memory", "mmc", "rproc_qcom",
                                         "suspend", "ufs", "usb", "wifi"
                                         ]
                          """,
                          default=[])
    parser.add_option('--fs', '--ftrace_buffer_size_kb', type='int', dest='ftrace_max_size',
                      help="""
                      This option indicates that ftrace trace buffer max size in KB.
                      It will be passed to ensure an early bailout from ftrace parser if size goes
                      beyond this specified value.
                      Example: --fs 4096
                      This specifies that max size is 4096 KB.
                      """)
    parser.add_option('', '--skip_TLB_Cache_parse', action='store_true', help='Skip parsing TLB Cache Dumps in parse_debug_image')
    parser.add_option('--iommu-pg-table-format', action='store', choices=['fastrpc', 'default'],
                      default='default')

    for p in parser_util.get_parsers():
        parser.add_option(p.shortopt or '',
                          p.longopt,
                          dest=p.cls.__name__,
                          help=p.desc,
                          action='store_true')

    (options, args) = parser.parse_args()
    if options.reduceddump:
        try:
            import ramreduction_util as elfutil
        except ImportError as ierr:
            print("Missing PyElftools library. Check README")
            sys.exit(1)
        except Exception as err:
            print("{}".format(str(err)))
            sys.exit(1)

    if options.minidump:
        default_list = []
        default_list.append("Schedinfo")
        default_list.append("Dmesg")
        default_list.append("RTB")
        default_list.append("DebugImage")
        default_list.append("Watchdog")
        default_list.append("KBootLog")
        default_list.append("PageTracking")
        default_list.append("Slabinfo")
        default_list.append("RunQueues")
        default_list.append("PStore")
        default_list.append("Kconfig")
        default_list.append("ThermalTemp")
        default_list.append("ipc_logging_cn")

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

    if options.outfile is None:
        # dmesg_TZ is a very non-descriptive name and should be changed
        # sometime in the future
        options.outfile = 'dmesg_TZ.txt'

    if not options.stdout:
        set_outfile(options.outdir + '/' + options.outfile)

    print_out_str('Linux Ram Dump Parser Version %s' % VERSION)

    args = ''
    for arg in sys.argv:
        args = args + arg + ' '

    print_out_str('Arguments: {0}'.format(args))

    system_type = parser_util.get_system_type()

    if options.autodump is not None:
        if os.path.exists(options.autodump):
            print_out_str(
                'Looking for Ram dumps in {0}'.format(options.autodump))
        else:
            print_out_str(
                'Path {0} does not exist for Ram dumps. Exiting...'.format(options.autodump))
            sys.exit(1)

    if options.vmlinux is None:
        if options.autodump is None:
            print_out_str("No vmlinux or autodump dir given. I can't proceed!")
            parser.print_usage()
            sys.exit(1)
        autovm = os.path.join(options.autodump, 'vmlinux')
        if os.path.isfile(autovm):
            options.vmlinux = autovm
        else:
            print_out_str("No vmlinux given or found in autodump dir. I can't proceed!")
            parser.print_usage()
            sys.exit(1)

    if not os.path.exists(options.vmlinux):
        print_out_str(
            '{0} does not exist. Cannot proceed without vmlinux. Exiting...'.format(options.vmlinux))
        sys.exit(1)
    elif not os.path.isfile(options.vmlinux):
        print_out_str(
            '{0} is not a file. Did you pass the ram file directory instead of the vmlinux?'.format(options.vmlinux))
        sys.exit(1)

    print_out_str('using vmlinux file {0}'.format(options.vmlinux))

    if options.ram_addr is None and options.autodump is None and not options.minidump and not options.reduceddump:
        print_out_str('Need one of --auto-dump or at least one --ram-file')
        sys.exit(1)

    if options.reduceddump and not options.autodump:
        print_out_str('Need autodump with --reduceddump option')
        sys.exit(1)

    if options.ram_addr is not None:
        for a in options.ram_addr:
            if os.path.exists(a[0]):
                print_out_str(
                    'Loading Ram file {0} from {1:x}--{2:x}'.format(a[0], a[1], a[2]))
            else:
                print_out_str(
                    'Ram file {0} does not exist. Exiting...'.format(a[0]))
                sys.exit(1)

    # offset 4 of vmlinux indcates it's 32 or 64 bits
    # for 64 bits it is 0x2, for 32 bits it is 0x1
    vm_file = open(options.vmlinux, "rb")
    vm_file.seek(4, 0)
    bin_bits = vm_file.read(1)
    vm_file.close()
    options.arm64 = (bin_bits == b'\x02')


    if options.wlan is None:
        options.wlan = "INTEGRATED"
    else:
        if not os.path.exists(options.wlan):
            print_out_str('{} does not exist.'.format(options.wlan))
            print_out_str('Cannot proceed without wlan.ko Exiting')
            sys.exit(1)

    gdb_path = options.gdb
    nm_path = options.nm
    objdump_path = options.objdump

    try:
        import local_settings
        try:
            gdb_ndk_path = None
            if options.arm64:
                gdb_path = gdb_path or local_settings.gdb64_path
                if hasattr(local_settings, 'gdb64_ndk_path'):
                    gdb_ndk_path = local_settings.gdb64_ndk_path
                nm_path = nm_path or local_settings.nm64_path
                objdump_path = objdump_path or local_settings.objdump64_path
            else:
                gdb_path = gdb_path or local_settings.gdb_path
                nm_path = nm_path or local_settings.nm_path
                objdump_path = objdump_path or local_settings.objdump_path
        except AttributeError as e:
            print_out_str("local_settings.py looks bogus. Please see README.txt")
            missing_attr = re.sub(".*has no attribute '(.*)'", '\\1', e.message)
            print_out_str("Specifically, looks like you're missing `%s'\n" % missing_attr)
            print_out_str("Full message: %s" % e.message)
            sys.exit(1)
    except ImportError:
        cross_compile = os.environ.get('CROSS_COMPILE')
        if cross_compile is not None:
            gdb_path = gdb_path or cross_compile+"gdb"
            nm_path = nm_path or cross_compile+"nm"

    if gdb_path is None or nm_path is None:
        print_out_str("!!! Incorrect path for toolchain specified.")
        print_out_str("!!! Please see the README for instructions on setting up local_settings.py or CROSS_COMPILE")
        sys.exit(1)

    print_out_str("Using gdb path {0}".format(gdb_path))
    print_out_str("Using nm path {0}".format(nm_path))

    if not os.path.exists(gdb_path):
        print_out_str("!!! gdb_path {0} does not exist! Check your settings!".format(gdb_path))
        sys.exit(1)

    if not os.access(gdb_path, os.X_OK):
        print_out_str("!!! No execute permissions on gdb path {0}".format(gdb_path))
        print_out_str("!!! Please check the path settings")
        print_out_str("!!! If this tool is being run from a shared location, contact the maintainer")
        sys.exit(1)

    if not os.path.exists(nm_path):
        print_out_str("!!! nm_path {0} does not exist! Check your settings!".format(nm_path))
        sys.exit(1)

    if not os.access(nm_path, os.X_OK):
        print_out_str("!!! No execute permissions on nm path {0}".format(nm_path))
        print_out_str("!!! Please check the path settings")
        print_out_str("!!! If this tool is being run from a shared location, contact the maintainer")
        sys.exit(1)

    if options.everything:
        options.qtf = True

    dump = RamDump(options, nm_path, gdb_path, objdump_path,gdb_ndk_path)

    if options.eval:
        if options.eval == '-':
            code = sys.stdin.read()
        else:
            code = options.eval
        exec(code)
        sys.exit(0)

    if options.shell or options.classic_shell:
        print("Entering interactive shell mode.")
        print("The RamDump instance is available in the `dump' variable\n")
        do_fallback = options.classic_shell
        if not do_fallback:
            try:
                from IPython import embed
                embed()
            except ImportError:
                do_fallback = True

        if do_fallback:
            import code
            vars = globals()
            vars.update(locals())

            #readline library not available as a standard python libary on windows
            if platform.system() == 'Linux':
                import readline
                import rlcompleter

                readline.set_completer(rlcompleter.Completer(vars).complete)
                readline.parse_and_bind("tab: complete")

            shell = code.InteractiveConsole(vars)
            shell.interact()
        sys.exit(0)

    if not options.minidump:
        if not dump.print_command_line():
            print_out_str('!!! Error printing saved command line.')
            print_out_str('!!! The vmlinux is probably wrong for the ramdumps')
            print_out_str('!!! Exiting now...')
            sys.exit(1)

    if options.minidump:
        try:
            if not dump.print_socinfo_minidump():
                print_out_str('!!! No serial number information '
                              'available for this minidump.')
        except Exception as err:
            print_out_str('Unable to extract serial number information')
    else:
        if not dump.print_socinfo():
            print_out_str('!!! No serial number information available.')

    try:
        epoch_ns = dump.read_u64('cd.read_data[0].epoch_ns')
        epoch_cyc = dump.read_u64('cd.read_data[0].epoch_cyc')
        print_out_str('\nepoch_ns: {0}ns  epoch_cyc: {1}\n'.format(epoch_ns,epoch_cyc))
    except Exception as e:
        print_out_str(str(e))
        pass

    if options.qdss:
        print_out_str('!!! --parse-qdss is now deprecated')
        print_out_str(
            '!!! Please just use --parse-debug-image to get QDSS information')

    if options.watchdog_time:
        print_out_str('\n--------- watchdog time -------')
        get_wdog_timing(dump)
        print_out_str('---------- end watchdog time-----')

    # Always verify Scheduler requirement for active_cpus on 64-bit platforms.
    if options.arm64:
        try:
            verify_active_cpus(dump)
        except Exception as err:
            print_out_str('Unable to extract active cpus  info')
    # we called parser.add_option with dest=p.cls.__name__ above,
    # so if the user passed that option then `options' will have a
    # p.cls.__name__ attribute.
    if options.minidump:
        parsers_to_run = [p for p in parser_util.get_parsers()
                          if getattr(options, p.cls.__name__)
                          or (options.everything and p.cls.__name__ in default_list)]
    else:
        parsers_to_run = [p for p in parser_util.get_parsers()
                      if getattr(options, p.cls.__name__)
                      or (options.everything and not p.optional)]
    if options.timeout:
        from func_timeout import func_timeout, FunctionTimedOut

    print_out_str("Time taken to setup the subparsers run : {}".format(time.time()-starttime))
    starttime = time.time()
    for i,p in enumerate(parsers_to_run):
        if i == 0:
            sys.stderr.write("\n")
        if options.minidump:
            if p.cls.__name__ not in default_list:
                print("    [%d/%d] %s ... not supported in minidump \n" %
                                 (i + 1, len(parsers_to_run), p.longopt))
                continue



        print("    [%d/%d] %s ... " %
                         (i + 1, len(parsers_to_run), p.longopt), end='', flush=True)
        before = time.time()
        print_out_str("start time {0}".format(before))
        with print_out_section(p.cls.__name__):
            try:
                if options.timeout:
                    try:
                        func_timeout(int(options.timeout), p.cls(dump).parse)
                    except Exception as e:
                        print_out_str(e)
                else:
                    p.cls(dump).parse()
            except:
                # log exceptions and continue by default
                if not options.debug:
                    print_out_str('!!! Exception while running {0}'.format(p.cls.__name__))
                    print_out_exception()
                    print("FAILED! ")
                else:
                    raise
        after = time.time()
        print_out_str("end time {0} time cost {1} for {2}".format(after, (after - before), p.cls.__name__))
        print("%fs" % (after - before),  flush=True)
        flush_outfile()

    sys.stderr.write("\n")

    if options.t32launcher or options.everything or options.minidump:
        dump.create_t32_launcher()

    dump.gdbmi.close()
    print_out_str("Time taken to complete ramparser subscripts : {}".format(time.time()-starttime))
    if options.reduceddump:
        print_out_str("Number of cache hits on full cache : {}".format(elfutil.cachehits))
        print_out_str("Number of cache misses on full cache : {}".format(elfutil.cachemiss))
