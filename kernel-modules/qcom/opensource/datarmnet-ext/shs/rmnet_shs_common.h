/* Copyright (c) 2020 The Linux Foundation. All rights reserved.
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

#ifndef DATARMNET4e9dcb0338
#define DATARMNET4e9dcb0338
#undef TRACE_INCLUDE_PATH
#include <trace/hooks/sched.h>
#include <linux/tracepoint.h>
#include <trace/hooks/vendor_hooks.h>
#define DATARMNET94fa0a43a2 489335
void DATARMNET4095253347(struct DATARMNET9b44b71ee9*ep);void DATARMNETf7d317ed55
(struct DATARMNET9b44b71ee9*ep);void DATARMNET36e5e526fa(struct 
DATARMNET6c78e47d24*DATARMNETd2a694d52a);void DATARMNET2fe780019f(struct 
DATARMNET6c78e47d24*DATARMNETd2a694d52a);void DATARMNETd8a6375e9c(struct 
DATARMNET6c78e47d24*DATARMNETd2a694d52a,struct list_head*head);void 
DATARMNET9914e9761e(struct DATARMNET6c78e47d24*DATARMNETd2a694d52a,struct 
list_head*head);void DATARMNET6bf538fa23(void);void DATARMNETaea4c85748(void);
void DATARMNETe46c480d71(void);void DATARMNETde8ee16f92(struct 
DATARMNET63d7680df2*DATARMNET63b1a086d5);void DATARMNET3e37ad2816(struct 
DATARMNET63d7680df2*DATARMNET63b1a086d5,struct list_head*DATARMNET4d030eb9b5);
void DATARMNETe102b3a798(struct DATARMNET63d7680df2*DATARMNET63b1a086d5,struct 
list_head*DATARMNET4d030eb9b5,int DATARMNETa7a5705ab0);void DATARMNET4bde88919f(
u8 cpu,int count);inline int DATARMNET362b15f941(u16 cpu);u64 
DATARMNETfc888b4d3e(u16 cpu);u32 DATARMNETeb3978575d(u8 DATARMNET42a992465f);u32
 DATARMNETeea3cef5b6(u8 DATARMNET42a992465f);u8 DATARMNET928c931df9(struct 
rps_map*map);int DATARMNET4e292977da(void*priv,const struct list_head*
DATARMNET9cf7d31274,const struct list_head*DATARMNET5444bd3b6f);int 
DATARMNET3c489db64a(void*priv,const struct list_head*DATARMNET9cf7d31274,const 
struct list_head*DATARMNET5444bd3b6f);int DATARMNETd5c15f1ff3(void*priv,const 
struct list_head*DATARMNET9cf7d31274,const struct list_head*DATARMNET5444bd3b6f)
;int DATARMNET85af86a36d(void*priv,const struct list_head*DATARMNET9cf7d31274,
const struct list_head*DATARMNET5444bd3b6f);int DATARMNETf181a18009(struct 
net_device*dev);int DATARMNET98b2a0ce62(struct net_device*dev);void 
DATARMNET7f1d9480cb(void*port);void DATARMNETa4bd2ef52c(void*port);int 
DATARMNET310c3eb16e(u8 mask);void DATARMNETb4a6870b3b(struct DATARMNET6c78e47d24
*DATARMNETd2a694d52a);void DATARMNETd8a6375e9c(struct DATARMNET6c78e47d24*
DATARMNETd2a694d52a,struct list_head*head);void DATARMNET9914e9761e(struct 
DATARMNET6c78e47d24*DATARMNETd2a694d52a,struct list_head*head);int 
DATARMNET04e8d1b862(u8 cpu,u8 mask);void DATARMNET44459105b4(struct sk_buff*skb,
struct DATARMNET63d7680df2*node_p);int DATARMNET217fe38119(u64 
DATARMNETab155dfd5d,struct net_device*dev);void*DATARMNETefcaf5fbe9(struct 
sk_buff*skb,u32 offset,u32 DATARMNET567bdc7221,void*buf);u32 DATARMNET8532ab3089
(u32 index,u32 DATARMNET0258668025,u32 hash,u8 DATARMNET778962e2c2);extern 
struct list_head DATARMNET9825511866;static inline void DATARMNET3e88a91b63(void
*unused,struct wait_queue_head*DATARMNETa08427f746,int*done,struct sock*sk){if((
sk->sk_protocol==IPPROTO_TCP||sk->sk_protocol==IPPROTO_UDP)&&(sk->sk_rxhash&
1048575)==DATARMNET94fa0a43a2){(*done)=(0xd26+209-0xdf6);
wake_up_interruptible_poll(DATARMNETa08427f746,EPOLLIN|EPOLLPRI|EPOLLRDNORM|
EPOLLRDBAND);}}static inline int DATARMNETed3cac41ac(void){int rc=
(0xd2d+202-0xdf7);rc=register_trace_android_vh_do_wake_up_sync(
DATARMNET3e88a91b63,NULL);return rc;}static inline int DATARMNET7fcf8c178f(void)
{int rc=(0xd2d+202-0xdf7);rc=unregister_trace_android_vh_do_wake_up_sync(
DATARMNET3e88a91b63,NULL);return rc;}
#undef TRACE_INCLUDE_PATH
#endif

