CC = gcc
PKTLOG_INCLUDE := -I ../uapi/linux \
		  -I ../target/inc

all: pktlogconf

pktlogconf:
	$(CC) pktlogconf.c $(PKTLOG_INCLUDE) -o pktlogconf

clean:
	rm -f pktlogconf
