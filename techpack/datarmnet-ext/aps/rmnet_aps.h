/* Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef DATARMNETa70542332d
#define DATARMNETa70542332d
#include <linux/skbuff.h>
#include <net/genetlink.h>
#ifdef DATARMNETd7ef88d6df
#define DATARMNET112d724eff(...) pr_err(__VA_ARGS__)
#else
#define DATARMNET112d724eff(...)
#endif
struct DATARMNET5d4139d9d7{u32 cmd;u32 DATARMNETbb588401ec;u32 
DATARMNET655f822a62;u32 ifindex;u8 DATARMNET6c2cba039d;u8 DATARMNET626c626b74;u8
 DATARMNET40bb1d945b;u8 DATARMNETecca9147fd;};struct DATARMNETca79857d4a{u32 cmd
;u32 DATARMNETe65883bfce;u32 DATARMNETbb588401ec;};
#define DATARMNET53f589a196 (0xd26+209-0xdf6)
#define DATARMNET3cff03c531 (0xd1f+216-0xdf5)
struct DATARMNETd51e57e3f4{u32 cmd;u32 DATARMNETbb588401ec;u32 ifindex;s32 
DATARMNET61b4abcc9b;__be32 saddr[(0xd11+230-0xdf3)];__be32 daddr[
(0xd11+230-0xdf3)];u16 sport;u16 dport;u32 DATARMNETe9aad463ce;u8 tos;u8 
DATARMNETa400ad4f72;u8 DATARMNET06d2413ad2;u8 DATARMNET0711bbda6c;u8 
DATARMNETecca9147fd[(0xef7+1114-0x130d)];};struct DATARMNET15bcb4844b{u32 cmd;
u32 DATARMNETe65883bfce;u32 DATARMNETbb588401ec;};struct DATARMNET797a5b1493{u32
 ifindex;u64 DATARMNET9c9a589dce;u32 DATARMNETc277c62678;u32 DATARMNETecca9147fd
[(0xd35+210-0xdff)];};struct DATARMNET9a727f81bc{u32 ifindex;u32 
DATARMNETecca9147fd[(0xcfc+267-0xe00)];};struct DATARMNET7fb3ee4333{u8 mux_id;u8
 DATARMNET6f031e7934;u8 DATARMNET08e1628d23;u8 len;u32 DATARMNETa960d37cad[
(0xd35+210-0xdff)];};int DATARMNET37a9efbbcb(struct sk_buff*DATARMNETaafc1d9519,
struct genl_info*DATARMNET54338da2ff);int DATARMNET1998d09852(struct sk_buff*
DATARMNETaafc1d9519,struct genl_info*DATARMNET54338da2ff);int 
DATARMNETae6b282c61(struct sk_buff*DATARMNETaafc1d9519,struct genl_info*
DATARMNET54338da2ff);int DATARMNET568dffe281(struct sk_buff*DATARMNETaafc1d9519,
struct genl_info*DATARMNET54338da2ff);
#endif 

