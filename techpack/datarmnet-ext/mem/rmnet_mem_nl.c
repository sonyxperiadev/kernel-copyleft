/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "rmnet_mem_nl.h"
#include "rmnet_mem_priv.h"
#define DATARMNETb005a78b72 "\x52\x4d\x4e\x45\x54\x5f\x4d\x45\x4d"
#define DATARMNET39e021cd6f (0xd26+209-0xdf6)
enum{DATARMNET5277047270,DATARMNET654ec9d727,DATARMNET579b73b6a1,
DATARMNET99bbc5ae70,};
#define DATARMNETb2539ccff0 (DATARMNETc820b7a9dc - (0xd26+209-0xdf6))
uint32_t DATARMNET7c4038843f;static struct nla_policy DATARMNET93ad46699e[
DATARMNETb2539ccff0+(0xd26+209-0xdf6)]={[DATARMNETe5184c7a76]=
NLA_POLICY_EXACT_LEN(sizeof(struct DATARMNET5d6175c98d)),[DATARMNETb0428b7575]=
NLA_POLICY_EXACT_LEN(sizeof(struct DATARMNET5d23779a8f)),};static const struct 
genl_ops DATARMNETb68b0ed922[]={{.cmd=DATARMNET654ec9d727,.doit=
DATARMNET291f036d31,},{.cmd=DATARMNET579b73b6a1,.doit=DATARMNET8e48a951e4,},};
struct genl_family DATARMNET595b5c3a9e __ro_after_init={.hdrsize=
(0xd2d+202-0xdf7),.name=DATARMNETb005a78b72,.version=DATARMNET39e021cd6f,.
maxattr=DATARMNETb2539ccff0,.policy=DATARMNET93ad46699e,.ops=DATARMNETb68b0ed922
,.n_ops=ARRAY_SIZE(DATARMNETb68b0ed922),};int DATARMNETe85d734d4f(int val,struct
 genl_info*DATARMNET54338da2ff){struct sk_buff*skb;void*msg_head;int rc;skb=
genlmsg_new(NLMSG_GOODSIZE,GFP_ATOMIC);if(skb==NULL)goto DATARMNETbf4095f79e;
msg_head=genlmsg_put(skb,(0xd2d+202-0xdf7),(0xd2d+202-0xdf7),&
DATARMNET595b5c3a9e,(0xd2d+202-0xdf7),DATARMNET654ec9d727);if(msg_head==NULL){rc
=-ENOMEM;rm_err(
"\x4d\x45\x4d\x5f\x47\x4e\x4c\x3a\x20\x46\x41\x49\x4c\x45\x44\x20\x74\x6f\x20\x6d\x73\x67\x5f\x68\x65\x61\x64\x20\x25\x64" "\n"
,rc);kfree(skb);goto DATARMNETbf4095f79e;}rc=nla_put_u32(skb,DATARMNETbd57d2442f
,val);if(rc!=(0xd2d+202-0xdf7)){rm_err(
"\x4d\x45\x4d\x5f\x47\x4e\x4c\x3a\x20\x46\x41\x49\x4c\x45\x44\x20\x6e\x6c\x61\x5f\x70\x75\x74\x20\x25\x64" "\n"
,rc);kfree(skb);goto DATARMNETbf4095f79e;}genlmsg_end(skb,msg_head);rc=
genlmsg_reply(skb,DATARMNET54338da2ff);if(rc!=(0xd2d+202-0xdf7))goto 
DATARMNETbf4095f79e;rm_err(
"\x4d\x45\x4d\x5f\x47\x4e\x4c\x3a\x20\x53\x75\x63\x63\x65\x73\x73\x66\x75\x6c\x6c\x79\x20\x73\x65\x6e\x74\x20\x69\x6e\x74\x20\x25\x64" "\n"
,val);return(0xd2d+202-0xdf7);DATARMNETbf4095f79e:rm_err(
"\x4d\x45\x4d\x5f\x47\x4e\x4c\x3a\x20\x46\x41\x49\x4c\x45\x44\x20\x74\x6f\x20\x73\x65\x6e\x64\x20\x69\x6e\x74\x20\x25\x64" "\n"
,val);return-(0xd26+209-0xdf6);}int rmnet_mem_nl_register(void){return 
genl_register_family(&DATARMNET595b5c3a9e);}void rmnet_mem_nl_unregister(void){
genl_unregister_family(&DATARMNET595b5c3a9e);}
