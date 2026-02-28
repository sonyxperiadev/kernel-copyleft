Test:

si_core_tests include tests to check if si_core kernel functionality is working well or not.
We have two sets of tests included in the same:
1. Direct Path
	Kernel test to directly interact with si-core API for direct call.
2. Loading TA/Sending command to TA
	Kernel tests to interact with si-core API for loading TA and sending command.

Usage:

insmod /vendor_dlkm/lib/modules/si_core_test.ko
mv /data/smcinvoke_example_ta64.mbn /vendor/firmware
echo > /dev/si_core_test_client
