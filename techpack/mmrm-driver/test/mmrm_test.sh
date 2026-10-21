#!/bin/sh
#-----------------------------------------------------------------------------
# Copyright (c) 2020, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#-----------------------------------------------------------------------------

show_help(){
	echo "Usage: $0 "
	echo "		ex: ./mmrm_test.sh"
}

do_mmrm_test(){

	if [ -d /data/kernel-tests/modules ]; then
		modpath=/data/kernel-tests/modules
	else
		modpath=/kernel-tests/modules/lib/modules/$(uname -r)/extra
	fi

	# Check if module exists
	if [ -e "${modpath}/mmrm_test_module.ko" ]; then
		mmrm_test_mod=mmrm_test_module
	else
		echo "ERROR: Failed to get mmrm test module"
		return 1
	fi

	# Insert module
	insmod "${modpath}/${mmrm_test_mod}.ko"
	if [ $? -ne 0 ]; then
		echo "ERROR: Failed to insert mmrm test module"
		return 1
	fi

	# Remove module after the test
	lsmod | grep $mmrm_test_mod
	if [ $? -eq 0 ]; then
		rmmod $mmrm_test_mod > /dev/null 2>&1
		if [ $? -ne 0 ]; then
			echo "ERROR: Failed to remove mmrm test module"
			return 1
		fi
	else
		echo "ERROR: Failed to find mmrm test module"
	fi

	return 0
}


while [ $# -gt 0 ]
do
	case $1 in
	-h | --help | *)
		show_help
		exit 1
	;;
	esac
done

# run mmrm test
echo "=== Running MMRM Test ==="
do_mmrm_test
if [ $? -eq 0 ];then
	echo "MMRM Test Passed"
else
	echo "MMRM Test Failed"
	exit 1
fi

exit 0
