/* Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef DATARMNETf34a2d1928
#define DATARMNETf34a2d1928
#include "rmnet_shs.h"
#include <net/genetlink.h>
#define DATARMNET0fa03ac25b (0xd26+209-0xdf6)
#define DATARMNET0228d9f101 "\x52\x4d\x4e\x45\x54\x5f\x53\x48\x53"
#define DATARMNETe64295b6cb (0xbb7+853-0xc3c)
#define DATARMNET598eb03fad (0xbc7+788-0xc04)
#define DATARMNET7afb49ee3f      (0xd07+2129-0x121c)
#define DATARMNET3a69f7c4e5      (0xd07+2122-0x1214)
#define DATARMNET2d19c9b1ef (0xcc7+5761-0x221c)
#define DATARMNET96de786762 (0xcc7+5754-0x2214)
#define DATARMNETb5601817b1 (0xdb7+6677-0x263c)
#define DATARMNET00895c1601  (0xdc7+3085-0x163c)
#define DATARMNET0cb8735618   (0xdb7+3102-0x1634)
#define DATARMNET6e742895e1  (0xdb7+3095-0x162c)
#define DATARMNETc30f35c15f   (0xdb7+3088-0x1624)
#define DATARMNET19092afcc2        (0xec7+1152-0x131d)
extern int DATARMNETc252c204a8;enum{DATARMNET9491b185b7,DATARMNETc574b5cfba,
DATARMNET8e3adfc5dd,DATARMNETffb2945689,DATARMNET51b1ee5a68,
RMNET_SHS_GENL_CMD_LL_FLOW,DATARMNET93b3e11659,DATARMNET5b3796e25a,};enum{
DATARMNET603b776397,DATARMNETaa0fe5a855,DATARMNET7d289a7bfa,DATARMNET813a742587,
DATARMNET50e1cd26c7,DATARMNET6ab4513e45,DATARMNET627787b1dd,DATARMNET0158bf4d2b,
};
#define DATARMNETcecb35ee33 (DATARMNET0158bf4d2b - (0xd26+209-0xdf6))
struct DATARMNET6c41b886b2{uint32_t DATARMNET4da4612f1e;uint32_t 
DATARMNETa3f89581b5;uint16_t DATARMNETc790ff30fc;uint16_t DATARMNET208ea67e1d;};
struct DATARMNET837c876a22{uint32_t DATARMNET8c11bd9466;uint32_t 
DATARMNET87636d0152;};struct DATARMNET1ac24ff95c{uint32_t DATARMNET8c11bd9466;
uint32_t ack_thresh;};struct DATARMNET80e227e008{uint8_t DATARMNET035f475d5c;
uint8_t DATARMNETcfb5dc7296;};
#define DATARMNETa35687f809 "RMNET_SHS_MSG"
enum{DATARMNETeaa13301a0,DATARMNETafee1e9070,DATARMNET943966c53e,};enum{
DATARMNET5f0371060e,DATARMNETc08daf87d4,DATARMNET8070cc0bdc,DATARMNETc2be398ed4,
};
#define DATARMNET3b631aeccb ((0xeb7+712-0x111d))
#define DATARMNET8a917ef593     ((0xd26+209-0xdf6))
struct DATARMNETe5f1cf1a69{uint32_t DATARMNETaf3d356342;uint8_t 
DATARMNET43a8300dfd;};enum{DATARMNET68b3f1699c=(0xd2d+202-0xdf7),
DATARMNETfce267cbe9=(0xd26+209-0xdf6),DATARMNETf41c724abf=(0xd1f+216-0xdf5),};
struct DATARMNET4a3b3209dd{char DATARMNETdf2dbc641f[DATARMNET3b631aeccb];
uint16_t msg_type;};struct DATARMNET25187800fe{int valid;};struct 
DATARMNET177911299b{struct DATARMNET4a3b3209dd list[DATARMNET8a917ef593];
uint64_t timestamp;uint16_t list_len;uint8_t valid;};struct DATARMNETbf4d34b241{
union{__be32 daddr;struct in6_addr v6_daddr;}DATARMNETea422561ef;union{__be32 
saddr;struct in6_addr v6_saddr;}DATARMNET53d5f671f0;u16 src_port;u16 
DATARMNET5a5907dd87;u16 DATARMNET1e49bc75c8;u16 DATARMNET1c959e10ca;u8 
DATARMNET1819cae4a3;u8 DATARMNETb035edcfb9;u8 proto;u8 DATARMNET8b5ace4a98;u8 
DATARMNET602389fe52;u8 DATARMNETc2d5c71ce1;u8 seq;u8 DATARMNET969cfb9094;};enum 
DATARMNET4c422f5fb1{DATARMNET9a035137c0,DATARMNET8d88a2f3f5,DATARMNET755fb8f198,
};struct DATARMNET0331d6732d{struct list_head DATARMNET32e10c59ce;struct 
hlist_node list;struct DATARMNETbf4d34b241 DATARMNET54338da2ff;};int 
DATARMNET740f3b34b3(struct sk_buff*DATARMNETaafc1d9519,struct genl_info*
DATARMNET54338da2ff);int DATARMNET29175fb5fc(struct sk_buff*DATARMNETaafc1d9519,
struct genl_info*DATARMNET54338da2ff);int DATARMNETd81d2866ba(struct sk_buff*
DATARMNETaafc1d9519,struct genl_info*DATARMNET54338da2ff);int 
DATARMNETc850634243(struct sk_buff*DATARMNETaafc1d9519,struct genl_info*
DATARMNET54338da2ff);int DATARMNET283f08f439(struct sk_buff*DATARMNETaafc1d9519,
struct genl_info*DATARMNET54338da2ff);int DATARMNET9bbfc822c2(struct sk_buff*
DATARMNETaafc1d9519,struct genl_info*DATARMNET54338da2ff);int 
DATARMNET5d4ca1da1c(struct genl_info*DATARMNET54338da2ff,int val);int 
DATARMNET5945236cd3(int val);int DATARMNETa9a7fa898c(void);void 
DATARMNET8d0d510d45(uint32_t DATARMNETaf3d356342,struct DATARMNET177911299b*
DATARMNET60b6e12cfd);int DATARMNETb5d58adbe7(struct DATARMNET177911299b*msg_ptr)
;int DATARMNETd65d1351b9(struct sk_buff*DATARMNETaafc1d9519,struct genl_info*
DATARMNET54338da2ff);void DATARMNET1d4b1eff85(struct DATARMNET177911299b*
DATARMNET60b6e12cfd,uint8_t DATARMNET907a90c6af,uint8_t DATARMNET9a4544e068);int
 DATARMNET0dbc627e8f(void);int DATARMNETeabd69d1ab(void);
#endif 

