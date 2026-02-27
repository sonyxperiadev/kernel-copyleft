Prerequisites:
1. gcc compiler. (preferably latest)
2. java
Procedure to Run:
1.	Collect the bugreport by issuing the below command
$ adb bugreport | tee bugreport.txt
2.  Copy this directory to the local linux machine and run the below command
to give executable permission to all parsers.
$ sh MISC/change_permissions.sh
3.	Run the below command to parse the bug report.
$ sh Bugreport.sh bugreport.txt
It creates a directory in BugReport_YYYY.MM.DD.HH.mm.ss format and creates a
directory for each Bug dump/Alert dump in bugreport.txt in BugdumpX/AlertdumpX
format.
It updates binary files and corresponding text formatted files to the
corresponding BugdumpX/AlertdumpX directory.
Note:
Please compile the parsers using below command if there are any system
dependencies and to be compiled:
$ sh MISC/compile_src.sh
