/* Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/ip.h>
#include <linux/cpu.h>
#include <net/ip.h>
#include <linux/cpu.h>
#include <linux/bitmap.h>
#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/ipv6.h>
#include <linux/netdevice.h>
#include <linux/percpu-defs.h>
#include "rmnet_shs.h"
#include "rmnet_shs_wq_genl.h"
#include "rmnet_shs_config.h"
#include "rmnet_shs_wq.h"
#include "rmnet_shs_modules.h"
#include "rmnet_shs_common.h"
#include "rmnet_trace.h"
#include <linux/icmp.h>
#include <linux/inet.h>
DEFINE_HASHTABLE(DATARMNET58fe8ac797,DATARMNET25437d35fd);DEFINE_HASHTABLE(
DATARMNET5750992efb,DATARMNET25437d35fd);DEFINE_SPINLOCK(DATARMNETd83ee17944);
struct DATARMNETe600c5b727 DATARMNET148e775ece[DATARMNETc6782fed88];
#define DATARMNET82243f712c (0xeb7+698-0x110d)
#define DATARMNET832731a933(CPU) (per_cpu(softnet_data, CPU).input_pkt_queue)
#define DATARMNET3de2536baf(CPU) (per_cpu(softnet_data, CPU).process_queue)
#define DATARMNETd2d15e7f67(CPU) (DATARMNET832731a933(CPU).qlen + \
DATARMNET3de2536baf(CPU).qlen)
#define DATARMNET78ec5e8746(DATARMNETfbfdc7f27e, CPU) (per_cpu(\
DATARMNETfbfdc7f27e, CPU).input_queue_tail)
#define DATARMNET9c1c20df35(DATARMNETfbfdc7f27e, CPU) (per_cpu(\
DATARMNETfbfdc7f27e, CPU).input_queue_head)
#define DATARMNET66ead9195a(CPU) (per_cpu(softnet_data, CPU).input_queue_head)
#define DATARMNETbb1541e65b(CPU) (per_cpu(softnet_data, CPU).input_queue_tail)
#define DATARMNET8053357aa4 (0xeb7+1130-0x130d)
unsigned int DATARMNETefe3dcad0a=(0xd2d+202-0xdf7);module_param(
DATARMNETefe3dcad0a,uint,(0xdb7+6665-0x261c));MODULE_PARM_DESC(
DATARMNETefe3dcad0a,
"\x4c\x4c\x20\x70\x61\x63\x6b\x65\x74\x73\x20\x73\x65\x65\x6e\x20\x69\x6e\x20\x6c\x6c\x20\x72\x6d\x6e\x65\x74\x5f\x73\x68\x73"
);unsigned int DATARMNET75ae82094a=(0xd2d+202-0xdf7);module_param(
DATARMNET75ae82094a,uint,(0xdb7+6665-0x261c));MODULE_PARM_DESC(
DATARMNET75ae82094a,
"\x4c\x4c\x20\x66\x69\x6c\x74\x65\x72\x20\x63\x6f\x75\x6e\x74\x20\x73\x65\x65\x6e\x20\x69\x6e\x20\x6c\x6c\x20\x72\x6d\x6e\x65\x74\x5f\x73\x68\x73"
);int DATARMNETf4aefff4c2(struct sk_buff*skb){int DATARMNETbd864aa442=
(0xd2d+202-0xdf7);struct iphdr*ip4h,DATARMNETc00baf31c3;struct ipv6hdr*ip6h,
DATARMNETcf1d9e2c1e;const struct ipv6_opt_hdr*DATARMNET7b34b7b5be;struct 
ipv6_opt_hdr DATARMNET1688a97aa4;switch(skb->protocol){case htons(ETH_P_IP):ip4h
=DATARMNETefcaf5fbe9(skb,(0xd2d+202-0xdf7),sizeof(*ip4h),&DATARMNETc00baf31c3);
if(!ip4h)break;if(!ip_is_fragment(ip4h)&&(ip4h->protocol==IPPROTO_TCP||ip4h->
protocol==IPPROTO_UDP)){DATARMNETbd864aa442=(0xd26+209-0xdf6);break;}if(ip4h->
protocol==IPPROTO_ICMP){skb->hash=(0xd2d+202-0xdf7);skb->sw_hash=
(0xd26+209-0xdf6);if(trace_print_icmp_rx_enabled()){char saddr[INET6_ADDRSTRLEN]
,daddr[INET6_ADDRSTRLEN];u16 ip_proto=(0xd2d+202-0xdf7);__be16 sequence=
(0xd2d+202-0xdf7);u8 type=(0xd2d+202-0xdf7);struct icmphdr*icmphdr,
DATARMNET5aa29a2264;memset(saddr,(0xd2d+202-0xdf7),INET6_ADDRSTRLEN);memset(
daddr,(0xd2d+202-0xdf7),INET6_ADDRSTRLEN);icmphdr=DATARMNETefcaf5fbe9(skb,ip4h->
ihl*(0xd11+230-0xdf3),sizeof(*icmphdr),&DATARMNET5aa29a2264);if(!icmphdr)goto 
DATARMNET03fd0cd6e6;if(icmphdr->type!=ICMP_ECHOREPLY&&icmphdr->type!=ICMP_ECHO)
goto DATARMNET03fd0cd6e6;ip_proto=htons(ETH_P_IP);type=icmphdr->type;sequence=
icmphdr->un.echo.sequence;snprintf(saddr,INET6_ADDRSTRLEN,"\x25\x70\x49\x34",&
ip4h->saddr);snprintf(daddr,INET6_ADDRSTRLEN,"\x25\x70\x49\x34",&ip4h->daddr);
trace_print_icmp_rx(skb,ip_proto,type,sequence,saddr,daddr);}}else if(ip4h->
protocol==IPPROTO_ESP){skb->hash=DATARMNETaa0602144b;skb->sw_hash=
(0xd26+209-0xdf6);}DATARMNET03fd0cd6e6:break;case htons(ETH_P_IPV6):ip6h=
DATARMNETefcaf5fbe9(skb,(0xd2d+202-0xdf7),sizeof(*ip6h),&DATARMNETcf1d9e2c1e);if
(!ip6h)break;if(!(ip6h->nexthdr==NEXTHDR_FRAGMENT)&&(ip6h->nexthdr==IPPROTO_TCP
||ip6h->nexthdr==IPPROTO_UDP)){DATARMNETbd864aa442=(0xd26+209-0xdf6);break;}if(
ip6h->nexthdr==NEXTHDR_ICMP){skb->hash=(0xd2d+202-0xdf7);skb->sw_hash=
(0xd26+209-0xdf6);if(trace_print_icmp_rx_enabled()){char saddr[INET6_ADDRSTRLEN]
,daddr[INET6_ADDRSTRLEN];u16 ip_proto=(0xd2d+202-0xdf7);__be16 sequence=
(0xd2d+202-0xdf7);u8 type=(0xd2d+202-0xdf7);struct icmp6hdr*icmp6hdr,
DATARMNETaa41336581;memset(saddr,(0xd2d+202-0xdf7),INET6_ADDRSTRLEN);memset(
daddr,(0xd2d+202-0xdf7),INET6_ADDRSTRLEN);icmp6hdr=DATARMNETefcaf5fbe9(skb,
sizeof(*ip6h),sizeof(*icmp6hdr),&DATARMNETaa41336581);if(!icmp6hdr)goto 
DATARMNETf623862dd4;if(icmp6hdr->icmp6_type!=ICMPV6_ECHO_REQUEST&&icmp6hdr->
icmp6_type!=ICMPV6_ECHO_REPLY)goto DATARMNETf623862dd4;ip_proto=htons(ETH_P_IPV6
);type=icmp6hdr->icmp6_type;sequence=icmp6hdr->icmp6_sequence;snprintf(saddr,
INET6_ADDRSTRLEN,"\x25\x70\x49\x36",&ip6h->saddr);snprintf(daddr,
INET6_ADDRSTRLEN,"\x25\x70\x49\x36",&ip6h->daddr);trace_print_icmp_rx(skb,
ip_proto,type,sequence,saddr,daddr);}}else if(ip6h->nexthdr==NEXTHDR_ESP){skb->
hash=DATARMNETaa0602144b;skb->sw_hash=(0xd26+209-0xdf6);}else if(ip6h->nexthdr==
NEXTHDR_FRAGMENT){if(skb->len-sizeof(struct ipv6hdr)<(int)sizeof(struct 
ipv6_opt_hdr)){break;}DATARMNET7b34b7b5be=skb_header_pointer(skb,sizeof(struct 
ipv6hdr),sizeof(DATARMNET1688a97aa4),&DATARMNET1688a97aa4);if(
DATARMNET7b34b7b5be&&DATARMNET7b34b7b5be->nexthdr==NEXTHDR_ESP){skb->hash=
DATARMNETaa0602144b;skb->sw_hash=(0xd26+209-0xdf6);}}DATARMNETf623862dd4:break;
default:break;}DATARMNETda96251102(DATARMNETcd24fca747,DATARMNET116c96c236,
DATARMNETbd864aa442,(0x16e8+787-0xc0c),(0x16e8+787-0xc0c),(0x16e8+787-0xc0c),skb
,NULL);return DATARMNETbd864aa442;}int DATARMNET71b0abb49e(struct sk_buff*skb,
struct ipv6hdr*DATARMNET5d09fca14e,struct DATARMNETbf4d34b241*
DATARMNET54338da2ff){struct tcphdr*tp,DATARMNETd1ff6cd568;struct udphdr*up,
DATARMNETc82d2f4e16;int DATARMNETb3ce3d0107,DATARMNETb16a9be210,
DATARMNET5a94751027,DATARMNETad7ab41bc8,DATARMNETf13f5dee10=false;int ret=false;
int DATARMNETd6e492a659=(0xd2d+202-0xdf7);u8 protocol;__be16 frag_off;
DATARMNETb3ce3d0107=!DATARMNET54338da2ff->DATARMNET1819cae4a3||ipv6_addr_equal(&
DATARMNET5d09fca14e->saddr,&DATARMNET54338da2ff->DATARMNET53d5f671f0.v6_saddr);
DATARMNETb16a9be210=!DATARMNET54338da2ff->DATARMNETb035edcfb9||ipv6_addr_equal(&
DATARMNET5d09fca14e->daddr,&DATARMNET54338da2ff->DATARMNETea422561ef.v6_daddr);
DATARMNET5a94751027=!DATARMNET54338da2ff->DATARMNET8b5ace4a98||
DATARMNET54338da2ff->proto==DATARMNET5d09fca14e->nexthdr;DATARMNETad7ab41bc8=!
DATARMNET54338da2ff->DATARMNET5a5907dd87;DATARMNETf13f5dee10=!
DATARMNET54338da2ff->DATARMNET1c959e10ca;protocol=DATARMNET5d09fca14e->nexthdr;
if(DATARMNET54338da2ff->DATARMNET5a5907dd87||DATARMNET54338da2ff->
DATARMNET1c959e10ca){DATARMNETd6e492a659=ipv6_skip_exthdr(skb,sizeof(*
DATARMNET5d09fca14e),&protocol,&frag_off);if(DATARMNETd6e492a659<
(0xd2d+202-0xdf7)){return false;}if(DATARMNET5d09fca14e->nexthdr==IPPROTO_TCP){
tp=DATARMNETefcaf5fbe9(skb,DATARMNETd6e492a659,sizeof(*tp),&DATARMNETd1ff6cd568)
;if(!tp){DATARMNETad7ab41bc8=false;DATARMNETf13f5dee10=false;}else{
DATARMNETad7ab41bc8=!DATARMNET54338da2ff->DATARMNET5a5907dd87||tp->source==(
DATARMNET54338da2ff->src_port);DATARMNETf13f5dee10=!DATARMNET54338da2ff->
DATARMNET1c959e10ca||tp->dest==(DATARMNET54338da2ff->DATARMNET1e49bc75c8);}}else
 if(DATARMNET5d09fca14e->nexthdr==IPPROTO_UDP){up=DATARMNETefcaf5fbe9(skb,
DATARMNETd6e492a659,sizeof(*up),&DATARMNETc82d2f4e16);if(!up){
DATARMNETad7ab41bc8=false;DATARMNETf13f5dee10=false;}else{DATARMNETad7ab41bc8=!
DATARMNET54338da2ff->DATARMNET5a5907dd87||up->source==(DATARMNET54338da2ff->
src_port);DATARMNETf13f5dee10=!DATARMNET54338da2ff->DATARMNET1c959e10ca||up->
dest==(DATARMNET54338da2ff->DATARMNET1e49bc75c8);}}}if((DATARMNETb3ce3d0107)&&(
DATARMNETb16a9be210)&&(DATARMNET5a94751027)&&(DATARMNETad7ab41bc8)&&(
DATARMNETf13f5dee10))ret=true;return ret;}int DATARMNETb9e0ebf153(struct sk_buff
*skb,struct iphdr*skb_ip4h,struct DATARMNETbf4d34b241*DATARMNET54338da2ff){int 
ret=false;struct tcphdr*tp,DATARMNETd1ff6cd568;struct udphdr*up,
DATARMNETc82d2f4e16;u16 DATARMNET43b01ff41b=skb_ip4h->ihl*(0xd11+230-0xdf3);int 
DATARMNETb3ce3d0107=!DATARMNET54338da2ff->DATARMNET1819cae4a3||skb_ip4h->saddr==
DATARMNET54338da2ff->DATARMNET53d5f671f0.saddr;int DATARMNETb16a9be210=!
DATARMNET54338da2ff->DATARMNETb035edcfb9||skb_ip4h->daddr==(DATARMNET54338da2ff
->DATARMNETea422561ef.daddr);int DATARMNET5a94751027=!DATARMNET54338da2ff->
DATARMNET8b5ace4a98||skb_ip4h->protocol==DATARMNET54338da2ff->proto;int 
DATARMNETad7ab41bc8=!DATARMNET54338da2ff->DATARMNET5a5907dd87;int 
DATARMNETf13f5dee10=!DATARMNET54338da2ff->DATARMNET1c959e10ca;if(
DATARMNET54338da2ff->DATARMNET5a5907dd87||DATARMNET54338da2ff->
DATARMNET1c959e10ca){if(skb_ip4h->protocol==IPPROTO_TCP){tp=DATARMNETefcaf5fbe9(
skb,DATARMNET43b01ff41b,sizeof(*tp),&DATARMNETd1ff6cd568);if(!tp){
DATARMNETad7ab41bc8=false;DATARMNETf13f5dee10=false;}else{DATARMNETad7ab41bc8=!
DATARMNET54338da2ff->DATARMNET5a5907dd87||tp->source==(DATARMNET54338da2ff->
src_port);DATARMNETf13f5dee10=!DATARMNET54338da2ff->DATARMNET1c959e10ca||tp->
dest==(DATARMNET54338da2ff->DATARMNET1e49bc75c8);}}else if(skb_ip4h->protocol==
IPPROTO_UDP){up=DATARMNETefcaf5fbe9(skb,DATARMNET43b01ff41b,sizeof(*up),&
DATARMNETc82d2f4e16);if(!up){DATARMNETad7ab41bc8=false;DATARMNETf13f5dee10=false
;}else{DATARMNETad7ab41bc8=!DATARMNET54338da2ff->DATARMNET5a5907dd87||up->source
==(DATARMNET54338da2ff->src_port);DATARMNETf13f5dee10=!DATARMNET54338da2ff->
DATARMNET1c959e10ca||up->dest==(DATARMNET54338da2ff->DATARMNET1e49bc75c8);}}}if(
(DATARMNETb3ce3d0107)&&(DATARMNETb16a9be210)&&(DATARMNET5a94751027)&&(
DATARMNETad7ab41bc8)&&(DATARMNETf13f5dee10))ret=true;rm_err(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x56\x34\x20\x73\x61\x64\x64\x72\x20\x6d\x61\x74\x63\x68\x20\x25\x75\x20\x64\x61\x64\x64\x72\x20\x6d\x61\x74\x63\x68\x20\x25\x75\x2c\x20\x70\x72\x6f\x74\x6f\x20\x6d\x61\x74\x63\x68\x20\x25\x75\x2c\x20\x73\x72\x63\x20\x70\x6f\x72\x74\x20\x25\x75\x2c\x20\x64\x65\x73\x74\x20\x70\x6f\x72\x74\x20\x6d\x61\x74\x63\x68\x20\x25\x75\x20\x20\x70\x72\x6f\x74\x6f\x20\x25\x75" "\n"
,DATARMNETb3ce3d0107,DATARMNETb16a9be210,DATARMNET5a94751027,DATARMNETad7ab41bc8
,DATARMNETf13f5dee10,skb_ip4h->protocol);return ret;}int DATARMNET8fc45abdc4(
struct DATARMNET0331d6732d*DATARMNET63b1a086d5,struct DATARMNET0331d6732d*
DATARMNET82190d5ee0){struct DATARMNETbf4d34b241*DATARMNET54338da2ff=&
DATARMNET63b1a086d5->DATARMNET54338da2ff;struct DATARMNETbf4d34b241*
DATARMNET099bdc749a=&DATARMNET82190d5ee0->DATARMNET54338da2ff;int 
DATARMNET10f1c9f8db=DATARMNET54338da2ff->DATARMNET602389fe52==
DATARMNET099bdc749a->DATARMNET602389fe52;int DATARMNETb3ce3d0107,
DATARMNETb16a9be210,DATARMNET5a94751027,DATARMNETad7ab41bc8,DATARMNETf13f5dee10=
false;if(DATARMNET54338da2ff->seq&&DATARMNET099bdc749a->seq&&(
DATARMNET54338da2ff->seq==DATARMNET099bdc749a->seq)){return true;}
DATARMNETb3ce3d0107=(!DATARMNET54338da2ff->DATARMNET1819cae4a3&&!
DATARMNET099bdc749a->DATARMNET1819cae4a3)||(DATARMNET54338da2ff->
DATARMNET1819cae4a3&&DATARMNET099bdc749a->DATARMNET1819cae4a3&&(
DATARMNET54338da2ff->DATARMNET602389fe52==(0xd11+230-0xdf3))?(
DATARMNET54338da2ff->DATARMNET53d5f671f0.saddr==DATARMNET099bdc749a->
DATARMNET53d5f671f0.saddr):ipv6_addr_equal(&DATARMNET099bdc749a->
DATARMNET53d5f671f0.v6_saddr,&DATARMNET54338da2ff->DATARMNET53d5f671f0.v6_saddr)
);DATARMNETb16a9be210=(!DATARMNET54338da2ff->DATARMNETb035edcfb9&&!
DATARMNET099bdc749a->DATARMNETb035edcfb9)||(DATARMNET54338da2ff->
DATARMNETb035edcfb9&&DATARMNET099bdc749a->DATARMNETb035edcfb9&&(
DATARMNET54338da2ff->DATARMNET602389fe52==(0xd11+230-0xdf3))?(
DATARMNET54338da2ff->DATARMNETea422561ef.daddr==DATARMNET099bdc749a->
DATARMNETea422561ef.daddr):ipv6_addr_equal(&DATARMNET099bdc749a->
DATARMNETea422561ef.v6_daddr,&DATARMNET54338da2ff->DATARMNETea422561ef.v6_daddr)
);DATARMNET5a94751027=(!DATARMNET54338da2ff->DATARMNET8b5ace4a98&&!
DATARMNET099bdc749a->DATARMNET8b5ace4a98)||(DATARMNET54338da2ff->
DATARMNET8b5ace4a98&&DATARMNET099bdc749a->DATARMNET8b5ace4a98&&
DATARMNET54338da2ff->proto==DATARMNET099bdc749a->proto);DATARMNETad7ab41bc8=(!
DATARMNET54338da2ff->DATARMNET5a5907dd87&&!DATARMNET099bdc749a->
DATARMNET5a5907dd87)||(DATARMNET54338da2ff->DATARMNET5a5907dd87&&
DATARMNET099bdc749a->DATARMNET5a5907dd87&&DATARMNET54338da2ff->src_port==
DATARMNET099bdc749a->src_port);DATARMNETf13f5dee10=(!DATARMNET54338da2ff->
DATARMNET1c959e10ca&&!DATARMNET099bdc749a->DATARMNET1c959e10ca)||(
DATARMNET54338da2ff->DATARMNET1c959e10ca&&DATARMNET099bdc749a->
DATARMNET1c959e10ca&&DATARMNET54338da2ff->DATARMNET1e49bc75c8==
DATARMNET099bdc749a->DATARMNET1e49bc75c8);rm_err(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x6d\x61\x74\x63\x68\x20\x72\x65\x73\x75\x6c\x74\x20\x73\x61\x64\x72\x20\x6d\x61\x74\x63\x68\x20\x25\x75\x20\x64\x61\x64\x64\x72\x20\x6d\x61\x74\x63\x68\x20\x25\x75\x2c\x20\x70\x72\x6f\x74\x6f\x20\x6d\x61\x74\x63\x68\x20\x25\x75\x2c\x20\x73\x72\x63\x20\x70\x6f\x72\x74\x20\x25\x75\x2c\x20\x64\x65\x73\x74\x20\x70\x6f\x72\x74\x20\x6d\x61\x74\x63\x68\x20\x25\x75\x20\x20\x76\x65\x72\x73\x69\x6f\x6e\x6d\x61\x74\x63\x68\x20\x25\x75" "\n"
,DATARMNETb3ce3d0107,DATARMNETb16a9be210,DATARMNET5a94751027,DATARMNETad7ab41bc8
,DATARMNETf13f5dee10,DATARMNET10f1c9f8db);return(DATARMNET10f1c9f8db&&
DATARMNETb3ce3d0107&&DATARMNETb16a9be210&&DATARMNET5a94751027&&
DATARMNETad7ab41bc8&&DATARMNETf13f5dee10);}int DATARMNETe24386452c(struct 
sk_buff*skb){struct iphdr*ip4h,DATARMNETc00baf31c3;struct ipv6hdr*ip6h,
DATARMNETcf1d9e2c1e;struct DATARMNET0331d6732d*node_p;struct hlist_node*tmp;int 
ret=false;spin_lock_bh(&DATARMNETd83ee17944);switch(skb->protocol){case htons(
ETH_P_IP):ip4h=DATARMNETefcaf5fbe9(skb,(0xd2d+202-0xdf7),sizeof(*ip4h),&
DATARMNETc00baf31c3);if(!ip4h){break;}hash_for_each_possible_safe(
DATARMNET5750992efb,node_p,tmp,list,ip4h->saddr){if(DATARMNETb9e0ebf153(skb,ip4h
,&node_p->DATARMNET54338da2ff)){ret=true;break;}}break;case htons(ETH_P_IPV6):
ip6h=DATARMNETefcaf5fbe9(skb,(0xd2d+202-0xdf7),sizeof(*ip6h),&
DATARMNETcf1d9e2c1e);if(!ip6h){break;}hash_for_each_possible_safe(
DATARMNET5750992efb,node_p,tmp,list,ip6h->saddr.in6_u.u6_addr32[
(0xd2d+202-0xdf7)]){if(DATARMNET71b0abb49e(skb,ip6h,&node_p->DATARMNET54338da2ff
)){ret=true;break;}}break;default:break;}spin_unlock_bh(&DATARMNETd83ee17944);
rm_err(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x50\x61\x63\x6b\x65\x74\x20\x46\x69\x6c\x74\x65\x72\x20\x63\x68\x65\x63\x6b\x65\x64\x20\x61\x6e\x61\x6c\x79\x7a\x65\x64\x20\x72\x65\x74\x3a\x20\x25\x64"
,ret);return ret;}void DATARMNETd52d50282d(struct DATARMNET0331d6732d*
DATARMNET63b1a086d5){struct DATARMNET0331d6732d*DATARMNET9a739c7d8b;struct 
hlist_node*tmp;struct DATARMNET6c78e47d24*DATARMNETd2a694d52a=NULL;unsigned long
 bkt;int i=(0xd2d+202-0xdf7);spin_lock_bh(&DATARMNETd83ee17944);
hash_for_each_safe(DATARMNET5750992efb,bkt,tmp,DATARMNET9a739c7d8b,list){i++;if(
DATARMNET8fc45abdc4(DATARMNET9a739c7d8b,DATARMNET63b1a086d5)){rm_err(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x25\x73" "\n",
"\x46\x69\x6c\x74\x65\x72\x20\x61\x6c\x72\x65\x61\x64\x79\x20\x69\x6e\x73\x74\x61\x6c\x6c\x65\x64\x2c\x20\x44\x75\x70\x20\x46\x69\x6c\x74\x65\x72"
);hash_del_rcu(&DATARMNET9a739c7d8b->list);kfree(DATARMNET9a739c7d8b);break;}}
spin_unlock_bh(&DATARMNETd83ee17944);spin_lock_bh(&DATARMNETfbdbab2ef6);
list_for_each_entry(DATARMNETd2a694d52a,&DATARMNET9825511866,DATARMNET6de26f0feb
){if(DATARMNETd2a694d52a->DATARMNET63b1a086d5&&!DATARMNETd2a694d52a->
DATARMNET63b1a086d5->DATARMNET80eb31d7b8){DATARMNETd2a694d52a->
DATARMNET63b1a086d5->DATARMNET80eb31d7b8=DATARMNET64165df74d;}}spin_unlock_bh(&
DATARMNETfbdbab2ef6);kfree(DATARMNET63b1a086d5);DATARMNETecc0627c70.
DATARMNET110549da6f--;DATARMNET75ae82094a--;rm_err(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x25\x73",
"\x20\x55\x6e\x69\x6e\x73\x74\x61\x6c\x6c\x65\x64\x20\x4c\x4c\x20\x66\x69\x6c\x74\x65\x72"
);}void DATARMNET1e918c8e0d(struct DATARMNET0331d6732d*DATARMNET63b1a086d5){
struct DATARMNETbf4d34b241*DATARMNET54338da2ff=&DATARMNET63b1a086d5->
DATARMNET54338da2ff;pr_info(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x70\x72\x6f\x74\x6f\x20\x76\x61\x6c\x69\x64\x20\x25\x75\x20\x73\x72\x63\x20\x61\x64\x64\x72\x20\x76\x61\x6c\x69\x64\x20\x25\x75\x2c\x20\x64\x65\x73\x74\x20\x61\x64\x64\x72\x20\x76\x61\x6c\x69\x64\x20\x25\x75\x2c\x20\x64\x65\x73\x74\x20\x70\x6f\x72\x74\x20\x76\x61\x6c\x69\x64\x20\x25\x75\x2c\x20\x73\x72\x63\x70\x6f\x72\x74\x20\x76\x61\x6c\x69\x64\x20\x25\x75\x2c\x20\x69\x70\x20\x76\x65\x72\x73\x69\x6f\x6e\x20\x25\x75\x20\x73\x65\x71\x20\x25\x75"
,DATARMNET54338da2ff->DATARMNET8b5ace4a98,DATARMNET54338da2ff->
DATARMNET1819cae4a3,DATARMNET54338da2ff->DATARMNETb035edcfb9,DATARMNET54338da2ff
->DATARMNET1c959e10ca,DATARMNET54338da2ff->DATARMNET5a5907dd87,
DATARMNET54338da2ff->DATARMNET602389fe52,DATARMNET54338da2ff->seq);pr_info(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x69\x6e\x66\x6f\x2d\x3e\x69\x70\x5f\x76\x65\x72\x73\x69\x6f\x6e\x20\x25\x75"
,DATARMNET54338da2ff->DATARMNET602389fe52);pr_info(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x69\x6e\x66\x6f\x2d\x3e\x70\x72\x6f\x74\x6f\x20\x25\x75"
,DATARMNET54338da2ff->proto);pr_info(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x69\x6e\x66\x6f\x2d\x3e\x64\x65\x73\x74\x5f\x70\x6f\x72\x74\x20\x25\x75"
,DATARMNET54338da2ff->DATARMNET1e49bc75c8);pr_info(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x69\x6e\x66\x6f\x2d\x3e\x73\x72\x63\x5f\x70\x6f\x72\x74\x20\x25\x75"
,DATARMNET54338da2ff->src_port);pr_info(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x69\x6e\x66\x6f\x2d\x3e\x64\x65\x73\x74\x5f\x61\x64\x64\x72\x5f\x76\x61\x6c\x69\x64\x20\x25\x75"
,DATARMNET54338da2ff->DATARMNETb035edcfb9);pr_info(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x69\x6e\x66\x6f\x2d\x3e\x73\x72\x63\x5f\x61\x64\x64\x72\x5f\x76\x61\x6c\x69\x64\x20\x25\x75"
,DATARMNET54338da2ff->DATARMNET1819cae4a3);pr_info(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x69\x6e\x66\x6f\x2d\x3e\x73\x65\x71\x20\x25\x75"
,DATARMNET54338da2ff->seq);if(DATARMNET54338da2ff->DATARMNET602389fe52==
(0xd11+230-0xdf3)&&(DATARMNET54338da2ff->DATARMNETb035edcfb9)&&(
DATARMNET54338da2ff->DATARMNET1819cae4a3)){pr_info(
"\x4e\x65\x77\x20\x66\x6c\x6f\x77\x20\x69\x6e\x66\x6f\x2d\x3e\x64\x65\x73\x74\x5f\x61\x64\x64\x72\x5f\x76\x61\x6c\x69\x64\x20\x25\x75\x20"
,DATARMNET54338da2ff->DATARMNETea422561ef.daddr);pr_info(
"\x4e\x65\x77\x20\x66\x6c\x6f\x77\x20\x69\x6e\x66\x6f\x2d\x3e\x73\x72\x63\x5f\x61\x64\x64\x72\x5f\x76\x61\x6c\x69\x64\x20\x25\x75"
,DATARMNET54338da2ff->DATARMNET53d5f671f0.saddr);}if(DATARMNET54338da2ff->
DATARMNET602389fe52==(0xd03+244-0xdf1)&&(DATARMNET54338da2ff->
DATARMNETb035edcfb9)&&(DATARMNET54338da2ff->DATARMNET1819cae4a3)){pr_info(
"\x4e\x65\x77\x20\x66\x6c\x6f\x77\x20\x69\x6e\x66\x6f\x2d\x3e\x64\x65\x73\x74\x5f\x61\x64\x64\x72\x5f\x76\x61\x6c\x69\x64\x20\x25\x75\x20\x25\x75\x20\x25\x75\x20\x25\x75\x20"
,DATARMNET63b1a086d5->DATARMNET54338da2ff.DATARMNETea422561ef.v6_daddr.in6_u.
u6_addr32[(0xd18+223-0xdf4)],DATARMNET63b1a086d5->DATARMNET54338da2ff.
DATARMNETea422561ef.v6_daddr.in6_u.u6_addr32[(0xd1f+216-0xdf5)],
DATARMNET63b1a086d5->DATARMNET54338da2ff.DATARMNETea422561ef.v6_daddr.in6_u.
u6_addr32[(0xd26+209-0xdf6)],DATARMNET63b1a086d5->DATARMNET54338da2ff.
DATARMNETea422561ef.v6_daddr.in6_u.u6_addr32[(0xd2d+202-0xdf7)]);pr_info(
"\x4e\x65\x77\x20\x66\x6c\x6f\x77\x20\x69\x6e\x66\x6f\x2d\x3e\x73\x72\x63\x5f\x61\x64\x64\x72\x5f\x76\x61\x6c\x69\x64\x20\x20\x25\x75\x20\x25\x75\x20\x25\x75\x20\x25\x75"
,DATARMNET63b1a086d5->DATARMNET54338da2ff.DATARMNET53d5f671f0.v6_saddr.in6_u.
u6_addr32[(0xd18+223-0xdf4)],DATARMNET63b1a086d5->DATARMNET54338da2ff.
DATARMNET53d5f671f0.v6_saddr.in6_u.u6_addr32[(0xd1f+216-0xdf5)],
DATARMNET63b1a086d5->DATARMNET54338da2ff.DATARMNET53d5f671f0.v6_saddr.in6_u.
u6_addr32[(0xd26+209-0xdf6)],DATARMNET63b1a086d5->DATARMNET54338da2ff.
DATARMNET53d5f671f0.v6_saddr.in6_u.u6_addr32[(0xd2d+202-0xdf7)]);}}void 
DATARMNET2ac305d296(struct DATARMNET0331d6732d*DATARMNET63b1a086d5){struct 
DATARMNET0331d6732d*DATARMNET9a739c7d8b;struct DATARMNET6c78e47d24*
DATARMNETd2a694d52a=NULL;unsigned long bkt;int i=(0xd2d+202-0xdf7);spin_lock_bh(
&DATARMNETd83ee17944);hash_for_each(DATARMNET5750992efb,bkt,DATARMNET9a739c7d8b,
list){i++;if(DATARMNET8fc45abdc4(DATARMNET9a739c7d8b,DATARMNET63b1a086d5)){kfree
(DATARMNET63b1a086d5);spin_unlock_bh(&DATARMNETd83ee17944);rm_err(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x25\x73",
"\x20\x44\x75\x70\x20\x66\x69\x6c\x74\x65\x72\x20\x73\x65\x65\x6e\x20\x6d\x61\x74\x63\x68\x20\x73\x65\x65\x6e\x2c\x20\x6e\x6f\x20\x69\x6e\x73\x74\x61\x6c\x6c"
);return;}}if(DATARMNETecc0627c70.DATARMNET110549da6f>=DATARMNET82243f712c){
kfree(DATARMNET63b1a086d5);spin_unlock_bh(&DATARMNETd83ee17944);
DATARMNET68d84e7b98[DATARMNET62807647a4]++;rm_err(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x25\x73" "\n",
"\x49\x6e\x73\x74\x61\x6c\x6c\x65\x64\x20\x4c\x4c\x20\x66\x69\x6c\x74\x65\x72\x20\x66\x61\x69\x6c\x65\x64\x3a\x20\x4d\x61\x78\x20\x72\x65\x61\x63\x68\x65\x64"
);return;}DATARMNETecc0627c70.DATARMNET110549da6f++;DATARMNET75ae82094a++;if(
DATARMNET756bdd424a)DATARMNET1e918c8e0d(DATARMNET63b1a086d5);hash_add(
DATARMNET5750992efb,&DATARMNET63b1a086d5->list,DATARMNET63b1a086d5->
DATARMNET54338da2ff.DATARMNET53d5f671f0.saddr);spin_unlock_bh(&
DATARMNETd83ee17944);rm_err("\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x25\x73" "\n",
"\x53\x65\x74\x74\x69\x6e\x67\x20\x6c\x6f\x77\x20\x6c\x61\x74\x65\x6e\x63\x79\x20\x66\x6c\x6f\x77\x20\x63\x68\x65\x63\x6b\x20\x66\x6f\x72\x20\x61\x6c\x6c\x20\x66\x6c\x6f\x77\x73"
);spin_lock_bh(&DATARMNETfbdbab2ef6);list_for_each_entry(DATARMNETd2a694d52a,&
DATARMNET9825511866,DATARMNET6de26f0feb){if(DATARMNETd2a694d52a->
DATARMNET63b1a086d5&&!DATARMNETd2a694d52a->DATARMNET63b1a086d5->
DATARMNET80eb31d7b8){DATARMNETd2a694d52a->DATARMNET63b1a086d5->
DATARMNET80eb31d7b8=DATARMNET64165df74d;}}spin_unlock_bh(&DATARMNETfbdbab2ef6);
rm_err("\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x25\x73" "\n",
"\x49\x6e\x73\x74\x61\x6c\x6c\x65\x64\x20\x4c\x4c\x20\x66\x69\x6c\x74\x65\x72");
}void DATARMNET337eca0417(struct sk_buff*skb,struct DATARMNET63d7680df2*
DATARMNET63b1a086d5){u32 DATARMNET1fcbf046ec=(0xd2d+202-0xdf7);u8 map=
(0xd2d+202-0xdf7),DATARMNET0258668025=(0xd2d+202-0xdf7);u16 index;if(!
DATARMNET63b1a086d5->DATARMNETfbbec4c537){map=DATARMNETecc0627c70.map_mask;
DATARMNET0258668025=DATARMNETecc0627c70.map_len;index=DATARMNET63b1a086d5->
map_index;}else{map=DATARMNET63b1a086d5->DATARMNETfbbec4c537;DATARMNET0258668025
=DATARMNET63b1a086d5->DATARMNETa59ce1fd2d;index=DATARMNET63b1a086d5->map_index;}
if(map){DATARMNET1fcbf046ec=DATARMNET8532ab3089(index,DATARMNET0258668025,
DATARMNET63b1a086d5->hash,(0xd2d+202-0xdf7));skb->hash=DATARMNET1fcbf046ec;}}int
 DATARMNETf5821256ad(struct sk_buff*skb,struct rmnet_shs_clnt_s*
DATARMNET0bf01e7c6f){struct DATARMNET63d7680df2*node_p;struct hlist_node*tmp;int
 map=DATARMNETecc0627c70.map_mask;int DATARMNETb925972e2a=DATARMNET3874292c18;
int map_cpu;u32 hash;u8 is_match_found=(0xd2d+202-0xdf7);struct 
DATARMNETe600c5b727*DATARMNETa4055affd5;struct rmnet_priv*priv;
DATARMNETefe3dcad0a++;hash=skb_get_hash(skb);if(!(DATARMNET0bf01e7c6f->config&
RMNET_SHS_STMP_ALL)&&!DATARMNETf4aefff4c2(skb)){DATARMNETe767554e6e(skb);return
(0xd2d+202-0xdf7);}spin_lock_bh(&DATARMNETd83ee17944);do{
hash_for_each_possible_safe(DATARMNET58fe8ac797,node_p,tmp,list,hash){if(hash!=
node_p->hash)continue;is_match_found=(0xd26+209-0xdf6);node_p->map_cpu=
DATARMNET3874292c18;node_p->map_index=DATARMNET04e8d1b862(node_p->map_cpu,map);
break;}if(is_match_found)break;if(DATARMNETb925972e2a<(0xd2d+202-0xdf7)){
DATARMNET68d84e7b98[DATARMNETa1f9420686]++;break;}if(atomic_long_read(&
DATARMNETecc0627c70.DATARMNET64bb8a8f57)>DATARMNETbfe31ef643){
DATARMNET68d84e7b98[DATARMNETe6e77f9f03]++;break;}node_p=kzalloc(sizeof(*node_p)
,GFP_ATOMIC);if(!node_p){DATARMNET68d84e7b98[DATARMNET394acaf558]++;break;}
atomic_long_inc(&DATARMNETecc0627c70.DATARMNET64bb8a8f57);node_p->
DATARMNETfbbec4c537=DATARMNET0bf01e7c6f->map_mask;node_p->DATARMNETa59ce1fd2d=
DATARMNETecc0627c70.map_mask;node_p->dev=skb->dev;node_p->hash=skb->hash;node_p
->map_cpu=DATARMNETb925972e2a;node_p->DATARMNET80eb31d7b8=(0xd26+209-0xdf6);
node_p->map_index=DATARMNET04e8d1b862(node_p->map_cpu,map);node_p->map_cpu=
raw_smp_processor_id();node_p->map_index=DATARMNET04e8d1b862(node_p->map_cpu,map
);INIT_LIST_HEAD(&node_p->DATARMNET04c88b8191);DATARMNET44459105b4(skb,node_p);
DATARMNET350f55bfca(node_p);map_cpu=node_p->map_cpu;DATARMNETa4055affd5=&
DATARMNET148e775ece[map_cpu];priv=netdev_priv(node_p->dev);if(!priv){rm_err(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x70\x72\x69\x76\x20\x66\x6f\x72\x20\x6e\x65\x74\x64\x65\x76\x20\x69\x73\x20\x6e\x75\x6c\x6c\x20\x66\x6f\x72\x20\x68\x61\x73\x68\x20\x30\x78\x25\x78"
,node_p->hash);DATARMNET68d84e7b98[DATARMNETf5157a9b85]++;}else{node_p->
DATARMNET341ea38662->mux_id=priv->mux_id;rm_err(
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x6d\x75\x78\x20\x69\x64\x20\x66\x6f\x72\x20\x68\x61\x73\x68\x20\x30\x78\x25\x78\x20\x69\x73\x20\x25\x64"
,node_p->hash,node_p->DATARMNET341ea38662->mux_id);}DATARMNET3e37ad2816(node_p,&
DATARMNETa4055affd5->DATARMNET3dc4262f53);hash_add_rcu(DATARMNET58fe8ac797,&
node_p->list,skb->hash);is_match_found=(0xd26+209-0xdf6);break;}while(
(0xd2d+202-0xdf7));spin_unlock_bh(&DATARMNETd83ee17944);if(is_match_found){
DATARMNET337eca0417(skb,node_p);if(!node_p->DATARMNETd986107d55&&
DATARMNET362b15f941(raw_smp_processor_id())){if(DATARMNETd2d15e7f67(
raw_smp_processor_id())<DATARMNET8053357aa4&&DATARMNETdc7bead533(
raw_smp_processor_id(),(0x2495+531-0x1708))){skb->hash=(0xd2d+202-0xdf7);skb->
sw_hash=(0xd26+209-0xdf6);}else if(!node_p->DATARMNETd986107d55){node_p->
DATARMNETd986107d55=DATARMNETbb52958049;node_p->map_cpu=DATARMNETb925972e2a;
node_p->map_index=DATARMNET04e8d1b862(node_p->map_cpu,map);}}else if(node_p->
DATARMNETd986107d55!=DATARMNET46a17e3ec5){if(!DATARMNET362b15f941(
raw_smp_processor_id())){if(DATARMNETd2d15e7f67(raw_smp_processor_id())<
DATARMNET8053357aa4&&DATARMNETdc7bead533(raw_smp_processor_id(),12000)){skb->
hash=(0xd2d+202-0xdf7);skb->sw_hash=(0xd26+209-0xdf6);}else{node_p->
DATARMNETd986107d55=DATARMNET46a17e3ec5;node_p->map_cpu=DATARMNETb925972e2a;
node_p->map_index=DATARMNET04e8d1b862(node_p->map_cpu,map);}}}if(skb_shinfo(skb)
->gso_segs){node_p->DATARMNET11930c5df8+=skb_shinfo(skb)->gso_segs;
DATARMNET0997c5650d[node_p->map_cpu].DATARMNET4133fc9428++;node_p->
DATARMNETae4b27456e.DATARMNET35234676d4+=skb_shinfo(skb)->gso_segs;}else{node_p
->DATARMNET11930c5df8+=(0xd26+209-0xdf6);DATARMNET0997c5650d[node_p->map_cpu].
DATARMNET4133fc9428++;node_p->DATARMNETae4b27456e.DATARMNET35234676d4++;}node_p
->DATARMNETa8940e4a7b+=(0xd26+209-0xdf6);node_p->DATARMNET3ecedac168+=
RMNET_SKB_CB(skb)->coal_bytes;node_p->DATARMNETce5f56eab9+=RMNET_SKB_CB(skb)->
coal_bufsize;if(skb->priority==55834)node_p->DATARMNET1743c92e66++;node_p->
DATARMNET2594c418db+=skb->len;}DATARMNETe767554e6e(skb);return(0xd2d+202-0xdf7);
}void DATARMNET44499733f2(void){u8 DATARMNET0e4304d903;for(DATARMNET0e4304d903=
(0xd2d+202-0xdf7);DATARMNET0e4304d903<DATARMNETc6782fed88;DATARMNET0e4304d903++)
INIT_LIST_HEAD(&DATARMNET148e775ece[DATARMNET0e4304d903].DATARMNET3dc4262f53);}
void DATARMNET90fe3a4b56(void){struct DATARMNET0331d6732d*DATARMNET63b1a086d5;
struct hlist_node*tmp;unsigned long bkt;rm_err("\x25\x73",
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x44\x65\x2d\x69\x6e\x69\x74\x20\x4c\x4c\x20\x62\x6f\x6f\x6b\x2d\x6b\x65\x65\x70\x69\x6e\x67"
);spin_lock_bh(&DATARMNETd83ee17944);hash_for_each_safe(DATARMNET58fe8ac797,bkt,
tmp,DATARMNET63b1a086d5,list){hash_del_rcu(&DATARMNET63b1a086d5->list);}
hash_for_each_safe(DATARMNET5750992efb,bkt,tmp,DATARMNET63b1a086d5,list){
hash_del_rcu(&DATARMNET63b1a086d5->list);kfree(DATARMNET63b1a086d5);
DATARMNETecc0627c70.DATARMNET110549da6f--;DATARMNET75ae82094a--;}spin_unlock_bh(
&DATARMNETd83ee17944);rm_err("\x25\x73",
"\x53\x48\x53\x5f\x4c\x4c\x3a\x20\x44\x65\x2d\x69\x6e\x69\x74\x20\x4c\x4c\x20\x62\x6f\x6f\x6b\x2d\x6b\x65\x65\x70\x69\x6e\x67\x20\x65\x78\x69\x74"
);}
