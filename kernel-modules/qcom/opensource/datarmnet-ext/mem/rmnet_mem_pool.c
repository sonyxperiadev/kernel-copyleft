/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "rmnet_mem_nl.h"
#include "rmnet_mem_priv.h"
#define DATARMNETc8aadbe769 (0xdf7+6169-0x241c)
#define DATARMNET831d60a2b1 (0xd18+223-0xdf4)
#define DATARMNETa967925c7a (0xdb7+6677-0x263c)
#define DATARMNET19337c1bbf (0xdb7+6670-0x2634)
extern struct work_struct pool_adjust_work;extern struct workqueue_struct*mem_wq
;int DATARMNET291f036d31(struct sk_buff*skb,struct genl_info*DATARMNET54338da2ff
){u8 mode=(0xd2d+202-0xdf7);struct sk_buff*DATARMNETa13fcf9070;struct 
DATARMNET5d6175c98d mem_info;struct nlattr*na;if(DATARMNET54338da2ff->attrs[
DATARMNETe5184c7a76]){na=DATARMNET54338da2ff->attrs[DATARMNETe5184c7a76];if(
nla_memcpy(&mem_info,na,sizeof(mem_info))>(0xd2d+202-0xdf7)){rm_err(
"%s(): modeinfo %u\n",__func__,mem_info.DATARMNET3a4d9ad400);}rm_err(
"\x25\x73\x28\x29\x3a\x20\x6d\x6f\x64\x65\x20\x25\x75" "\n",__func__,mode);
DATARMNETa13fcf9070=nlmsg_new(NLMSG_DEFAULT_SIZE,GFP_KERNEL);if(!
DATARMNETa13fcf9070)return-ENOMEM;DATARMNETe85d734d4f(DATARMNETa967925c7a,
DATARMNET54338da2ff);}else{DATARMNETe85d734d4f(DATARMNET19337c1bbf,
DATARMNET54338da2ff);}return(0xd2d+202-0xdf7);}int DATARMNET8e48a951e4(struct 
sk_buff*skb,struct genl_info*DATARMNET54338da2ff){struct sk_buff*
DATARMNETa13fcf9070;struct DATARMNET5d23779a8f mem_info;struct nlattr*na;int i;
u8 DATARMNET205e85dea0=(0xd2d+202-0xdf7);if(DATARMNET54338da2ff->attrs[
DATARMNETb0428b7575]){na=DATARMNET54338da2ff->attrs[DATARMNETb0428b7575];if(
nla_memcpy(&mem_info,na,sizeof(mem_info))>(0xd2d+202-0xdf7)){pr_err(
"\x25\x73\x28\x29\x3a\x20\x6d\x6f\x64\x65\x69\x6e\x66\x6f\x20\x25\x75" "\n",
__func__,mem_info.DATARMNET855b934a37);}rm_err(
"\x25\x73\x28\x29\x3a\x20\x70\x6f\x6f\x6c\x5f\x73\x69\x7a\x65\x20\x25\x75" "\n",
__func__,mem_info.DATARMNETe87b937bb6[(0xd2d+202-0xdf7)]);for(i=
(0xd2d+202-0xdf7);i<POOL_LEN;i++){if(mem_info.DATARMNET855b934a37&
(0xd26+209-0xdf6)<<i&&mem_info.DATARMNETe87b937bb6[i]!=static_pool_size[i]){
target_static_pool_size[i]=mem_info.DATARMNETe87b937bb6[i];DATARMNET205e85dea0=
(0xd26+209-0xdf6);}}if(DATARMNET205e85dea0&&mem_wq)queue_work(mem_wq,&
pool_adjust_work);DATARMNETa13fcf9070=nlmsg_new(NLMSG_DEFAULT_SIZE,GFP_KERNEL);
if(!DATARMNETa13fcf9070)return-ENOMEM;DATARMNETe85d734d4f(DATARMNETa967925c7a,
DATARMNET54338da2ff);}else{DATARMNETe85d734d4f(DATARMNET19337c1bbf,
DATARMNET54338da2ff);}return(0xd2d+202-0xdf7);}
