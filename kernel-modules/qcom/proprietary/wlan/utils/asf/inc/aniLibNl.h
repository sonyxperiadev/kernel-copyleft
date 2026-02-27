/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


#ifndef _ANI_LIB_NL_H_
#define _ANI_LIB_NL_H_ 1

#include <asm/types.h>
#include <linux/netlink.h>
#include <aniNlMsg.h>

typedef struct sAniNlHandle
{
    int         fd;
    struct sockaddr_nl  local;
    struct sockaddr_nl  peer;
    __u32           seq;
    __u32           dump;
} tAniNlHandle ;

extern int aniNlOpen(tAniNlHandle *wh, unsigned subscriptions);
extern int aniNlWildDumpReq(tAniNlHandle *wh, int fam, int type);
extern int aniNlDumpReq(tAniNlHandle *wh, int type, void *req,
                int len);
extern int aniNlDumpFilter(tAniNlHandle *wh,
        int (*filter)(struct sockaddr_nl *, struct nlmsghdr *n, void *),
        void *arg1,
        int (*junk)(struct sockaddr_nl *,struct nlmsghdr *n, void *),
        void *arg2);

extern int aniNlTalk(tAniNlHandle *wninl, struct nlmsghdr *n, pid_t peer,
             unsigned groups, struct nlmsghdr *answer,
             int (*junk)(struct sockaddr_nl *,struct nlmsghdr *n, void *),
             void *jarg);
extern int aniNlSend(tAniNlHandle *wh, char *buf, int);


extern int addAttr32(struct nlmsghdr *n, int maxlen, int type, __u32 data);
extern int addAttrL(struct nlmsghdr *n, int maxlen, int type, void *data, int alen);
extern int aniAddAttr32(struct wniattr *rta, int maxlen, int type, __u32 data);
extern int aniAddAttrL(struct wniattr *rta, int maxlen, int type, void *data, int alen);

extern int aniNlAttrParse(struct wniattr *tb[], int max, struct wniattr *wattr, int len);

extern int aniNlListen(tAniNlHandle *, int (*handler)(struct sockaddr_nl *,
            struct nlmsghdr *n, void *), void *jarg);
extern int aniNlFromFile(FILE *, int (*handler)(struct sockaddr_nl *,
            struct nlmsghdr *n, void *), void *jarg);

#endif /* _ANI_LIB_NL_H_ */
