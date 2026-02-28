/* Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "rmnet_mem_nl.h"
#include "rmnet_mem_priv.h"
#define DATARMNETc8aadbe769 (0xdf7+6169-0x241c)
#define DATARMNET831d60a2b1 (0xd18+223-0xdf4)
#define DATARMNETa967925c7a (0xdb7+6677-0x263c)
#define DATARMNET19337c1bbf (0xdb7+6670-0x2634)
extern struct delayed_work pool_adjust_work;extern struct workqueue_struct*
mem_wq;int DATARMNET291f036d31(struct sk_buff*skb,struct genl_info*
DATARMNET54338da2ff){u8 mode=(0xd2d+202-0xdf7);struct DATARMNET5d6175c98d 
mem_info;struct nlattr*na;if(DATARMNET54338da2ff->attrs[DATARMNETe5184c7a76]){na
=DATARMNET54338da2ff->attrs[DATARMNETe5184c7a76];if(nla_memcpy(&mem_info,na,
sizeof(mem_info))>(0xd2d+202-0xdf7)){rm_err("%s(): modeinfo %u\n",__func__,
mem_info.DATARMNET3a4d9ad400);}rm_err(
"\x25\x73\x28\x29\x3a\x20\x6d\x6f\x64\x65\x20\x25\x75" "\n",__func__,mode);
DATARMNETe85d734d4f(DATARMNETa967925c7a,DATARMNET54338da2ff);}else{
DATARMNETe85d734d4f(DATARMNET19337c1bbf,DATARMNET54338da2ff);}return
(0xd2d+202-0xdf7);}int DATARMNET8e48a951e4(struct sk_buff*skb,struct genl_info*
DATARMNET54338da2ff){struct DATARMNET5d23779a8f mem_info;struct nlattr*na;int i;
unsigned long DATARMNET28085cfd14;u8 DATARMNET205e85dea0=(0xd2d+202-0xdf7);u8 
DATARMNET4f9cb7ce34=(0xd2d+202-0xdf7);DATARMNETa293261aea[DATARMNET95fc2e84cc]++
;if(DATARMNET54338da2ff->attrs[DATARMNETb0428b7575]){na=DATARMNET54338da2ff->
attrs[DATARMNETb0428b7575];if(nla_memcpy(&mem_info,na,sizeof(mem_info))>
(0xd2d+202-0xdf7)){rm_err(
"\x25\x73\x28\x29\x3a\x20\x6d\x6f\x64\x65\x69\x6e\x66\x6f\x20\x25\x75" "\n",
__func__,mem_info.DATARMNET855b934a37);}for(i=(0xd2d+202-0xdf7);i<POOL_LEN;i++){
if(mem_info.DATARMNET855b934a37&(0xd26+209-0xdf6)<<i&&mem_info.
DATARMNETe87b937bb6[i]>(0xd2d+202-0xdf7)&&mem_info.DATARMNETe87b937bb6[i]<=
MAX_STATIC_POOL){DATARMNET15e53a8338[i]=mem_info.DATARMNETe87b937bb6[i];
max_pool_size[i]=mem_info.DATARMNETe87b937bb6[i];DATARMNET205e85dea0=
(0xd26+209-0xdf6);if(!DATARMNET4f9cb7ce34&&mem_info.DATARMNETe87b937bb6[i]>
static_pool_size[i]){DATARMNET4f9cb7ce34=(0xd26+209-0xdf6);}}}rm_err(
"\x20\x70\x6f\x6f\x6c\x73\x69\x7a\x65\x20\x25\x64\x20\x25\x64" "\n",mem_info.
DATARMNETe87b937bb6[(0xd1f+216-0xdf5)],mem_info.DATARMNETe87b937bb6[
(0xd18+223-0xdf4)]);if(DATARMNET205e85dea0&&mem_wq){DATARMNET28085cfd14=
msecs_to_jiffies(DATARMNET675090896c);cancel_delayed_work_sync(&pool_adjust_work
);queue_delayed_work(mem_wq,&pool_adjust_work,(DATARMNET4f9cb7ce34)?
(0xd2d+202-0xdf7):DATARMNET28085cfd14);}DATARMNETe85d734d4f(DATARMNETa967925c7a,
DATARMNET54338da2ff);}else{DATARMNETe85d734d4f(DATARMNET19337c1bbf,
DATARMNET54338da2ff);}return(0xd2d+202-0xdf7);}int DATARMNET803d42739e(struct 
sk_buff*skb,struct genl_info*DATARMNET54338da2ff){struct DATARMNET5d23779a8f 
mem_info;struct nlattr*na;int i;DATARMNETa293261aea[DATARMNETe581523c0b]++;if(
DATARMNET54338da2ff->attrs[DATARMNETb0428b7575]){na=DATARMNET54338da2ff->attrs[
DATARMNETb0428b7575];if(nla_memcpy(&mem_info,na,sizeof(mem_info))>
(0xd2d+202-0xdf7)){rm_err(
"\x25\x73\x28\x29\x3a\x20\x6d\x6f\x64\x65\x69\x6e\x66\x6f\x20\x25\x75" "\n",
__func__,mem_info.DATARMNET855b934a37);}rm_err(
"\x25\x73\x28\x29\x3a\x20\x70\x62\x69\x6e\x64\x20\x70\x6f\x6f\x6c\x5f\x73\x69\x7a\x65\x20\x25\x75" "\n"
,__func__,mem_info.DATARMNETe87b937bb6[(0xd18+223-0xdf4)]);for(i=
(0xd2d+202-0xdf7);i<POOL_LEN;i++){if(mem_info.DATARMNET855b934a37&
(0xd26+209-0xdf6)<<i){if(mem_info.DATARMNETe87b937bb6[i]>(0xd2d+202-0xdf7)&&
mem_info.DATARMNETe87b937bb6[i]<=MAX_STATIC_POOL)DATARMNETf85ebffa7a[i]=mem_info
.DATARMNETe87b937bb6[i];}}DATARMNETe85d734d4f(DATARMNETa967925c7a,
DATARMNET54338da2ff);}else{DATARMNETe85d734d4f(DATARMNET19337c1bbf,
DATARMNET54338da2ff);}return(0xd2d+202-0xdf7);}
