#ifndef _BD7602_H
#define _BD7602_H

#define BD7602_MAGIC 'J'

/*
 * BD7602 MODE and VALUE via ioctl
 * BD7602_MODE_STORE: store target register address
 * BD7602_MODE_SHOW: retrieve current register address
 * BD7602_VALUE_STORE: store register value
 * BD7602_VALUE_SHOW: retrieve register value
 */
#define BD7602_MODE_STORE		_IOW(BD7602_MAGIC, 0x01, char *)
#define BD7602_MODE_SHOW		_IOR(BD7602_MAGIC, 0x02, char *)
#define BD7602_VALUE_STORE		_IOW(BD7602_MAGIC, 0x03, char *)
#define BD7602_VALUE_SHOW		_IOR(BD7602_MAGIC, 0x04, char *)

#endif
