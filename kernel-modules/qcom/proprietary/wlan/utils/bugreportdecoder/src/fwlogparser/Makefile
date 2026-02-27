CC := $(ATH_CROSS_COMPILE_TYPE)gcc
TARGET_TYPE ?= AR9888
TARGET_VERS ?= v2

all:
	$(CC) -g3 -Wall  \
		-I ../../../target/inc \
		-I ../../../uapi/linux \
		cld-diag-parser.c parser.c cld-fwlog-parser.c -o fwlogparser
clean:
	rm -f cld-fwlog-record
	rm -f cld-fwlog-parser
	rm -f cld-fwlog-netlink
