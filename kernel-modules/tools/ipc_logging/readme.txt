IPC Logging Extraction Script
=============================

Quick Start
-----------
To extract IPC logging info from RAM Dumps, use the script as follows:

python ipc_logging.py parse <RAM dump 1> <RAM dump 2> <RAM dump N>

This will give you just the parsed output. If you would also like to
output each log page's data in binary format, along with more verbose
output, use the -v option:

python ipc_logging.py parse -v <RAM dump 1> <RAM dump 2> <RAM dump N>

Before running the script, consult the load.cmm file from the crash dumps
(if it is available) to make sure the dumps are contiguous and in the normal
order, and modify the above commands as necessary. Run the script with no
arguments for more usage information.

If the load.cmm file is not available and the script cannot parse the dumps
with the default command, contact asw.mp.linux for assistance.

Advanced Usage
--------------
To see more usage options, run the ipc_logging.py with no arguments.
