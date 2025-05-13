/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: GPL-2.0-only
*/

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/mm.h>
#include "rmnet_mem_nl.h"
#include "rmnet_mem.h"
#include "rmnet_mem_priv.h"
MODULE_LICENSE("\x47\x50\x4c\x20\x76\x32");DEFINE_SPINLOCK(rmnet_mem_lock);int 
DATARMNETfb2a1a4560[POOL_LEN];module_param_array(DATARMNETfb2a1a4560,int,NULL,
(0xcb7+5769-0x221c));MODULE_PARM_DESC(DATARMNETfb2a1a4560,
"\x67\x61\x76\x65\x75\x70\x20\x70\x65\x72\x20\x69\x64");int max_pool_size[
POOL_LEN]={(0xd2d+202-0xdf7),(0xd2d+202-0xdf7),MAX_POOL_O2,MAX_POOL_O3};
module_param_array(max_pool_size,int,NULL,(0xdb7+6665-0x261c));MODULE_PARM_DESC(
max_pool_size,
"\x4d\x61\x78\x20\x50\x6f\x6f\x6c\x20\x73\x69\x7a\x65\x20\x70\x65\x72\x20\x6f\x72\x64\x65\x72"
);int static_pool_size[POOL_LEN];module_param_array(static_pool_size,int,NULL,
(0xcb7+5769-0x221c));MODULE_PARM_DESC(static_pool_size,
"\x50\x6f\x6f\x6c\x20\x73\x69\x7a\x65\x20\x70\x65\x72\x20\x6f\x72\x64\x65\x72");
int pool_unbound_feature[POOL_LEN]={(0xd2d+202-0xdf7),(0xd2d+202-0xdf7),
(0xd26+209-0xdf6),(0xd26+209-0xdf6)};module_param_array(pool_unbound_feature,int
,NULL,(0xdb7+6665-0x261c));MODULE_PARM_DESC(pool_unbound_featue,
"\x50\x6f\x6f\x6c\x20\x62\x6f\x75\x6e\x64\x20\x67\x61\x74\x65");int 
rmnet_mem_order_requests[POOL_LEN];module_param_array(rmnet_mem_order_requests,
int,NULL,(0xcb7+5769-0x221c));MODULE_PARM_DESC(rmnet_mem_order_requests,
"\x52\x65\x71\x75\x65\x73\x74\x20\x70\x65\x72\x20\x6f\x72\x64\x65\x72");int 
rmnet_mem_id_req[POOL_LEN];module_param_array(rmnet_mem_id_req,int,NULL,
(0xcb7+5769-0x221c));MODULE_PARM_DESC(rmnet_mem_id_req,
"\x52\x65\x71\x75\x65\x73\x74\x20\x70\x65\x72\x20\x69\x64");int 
rmnet_mem_id_recycled[POOL_LEN];module_param_array(rmnet_mem_id_recycled,int,
NULL,(0xcb7+5769-0x221c));MODULE_PARM_DESC(rmnet_mem_id_recycled,
"\x52\x65\x63\x79\x63\x6c\x65\x64\x20\x70\x65\x72\x20\x69\x64");struct 
workqueue_struct*mem_wq;int target_static_pool_size[POOL_LEN];module_param_array
(target_static_pool_size,int,NULL,(0xcb7+5769-0x221c));MODULE_PARM_DESC(
target_static_pool_size,
"\x50\x6f\x6f\x6c\x20\x73\x69\x7a\x65\x20\x70\x65\x72\x20\x6f\x72\x64\x65\x72");
struct work_struct pool_adjust_work;struct list_head rmnet_mem_pool[POOL_LEN];
struct mem_info{struct page*addr;struct list_head mem_head;u8 order;};void 
rmnet_mem_page_ref_inc_entry(struct page*page,unsigned id){page_ref_inc(page);}
EXPORT_SYMBOL(rmnet_mem_page_ref_inc_entry);struct rmnet_mem_notif_s{struct 
raw_notifier_head chain;spinlock_t lock;};struct rmnet_mem_notif_s 
rmnet_mem_notifier={.chain=RAW_NOTIFIER_INIT(rmnet_mem_notifier.chain),.lock=
__SPIN_LOCK_UNLOCKED(rmnet_mem_notifier.lock),};EXPORT_SYMBOL(rmnet_mem_notifier
);int rmnet_mem_mode_notify(unsigned pool_size){unsigned long flags;
spin_lock_irqsave(&rmnet_mem_notifier.lock,flags);raw_notifier_call_chain(&
rmnet_mem_notifier.chain,pool_size,NULL);spin_unlock_irqrestore(&
rmnet_mem_notifier.lock,flags);return NOTIFY_OK;}int rmnet_mem_register_notifier
(struct notifier_block*nb){unsigned long flags;int ret;spin_lock_irqsave(&
rmnet_mem_notifier.lock,flags);ret=raw_notifier_chain_register(&
rmnet_mem_notifier.chain,nb);spin_unlock_irqrestore(&rmnet_mem_notifier.lock,
flags);return ret;}EXPORT_SYMBOL_GPL(rmnet_mem_register_notifier);int 
rmnet_mem_unregister_notifier(struct notifier_block*nb){unsigned long flags;int 
ret;spin_lock_irqsave(&rmnet_mem_notifier.lock,flags);ret=
raw_notifier_chain_unregister(&rmnet_mem_notifier.chain,nb);
spin_unlock_irqrestore(&rmnet_mem_notifier.lock,flags);return ret;}
EXPORT_SYMBOL_GPL(rmnet_mem_unregister_notifier);struct mem_info*
rmnet_mem_add_page(struct page*page,u8 pageorder){struct mem_info*mem_slot;
mem_slot=(struct mem_info*)kzalloc(sizeof(*mem_slot),GFP_ATOMIC);if(!mem_slot)
return NULL;static_pool_size[pageorder]++;mem_slot->order=pageorder;mem_slot->
addr=(void*)page;INIT_LIST_HEAD(&mem_slot->mem_head);if(pageorder<POOL_LEN){
list_add_rcu(&mem_slot->mem_head,&(rmnet_mem_pool[pageorder]));}return mem_slot;
}void rmnet_mem_free_all(void){unsigned long flags;struct mem_info*mem_slot;
struct list_head*ptr=NULL,*next=NULL;int i;spin_lock_irqsave(&rmnet_mem_lock,
flags);for(i=(0xd2d+202-0xdf7);i<POOL_LEN;i++){list_for_each_safe(ptr,next,&
rmnet_mem_pool[i]){mem_slot=list_entry(ptr,struct mem_info,mem_head);list_del(&
mem_slot->mem_head);put_page(mem_slot->addr);static_pool_size[mem_slot->order]--
;kfree(mem_slot);}}spin_unlock_irqrestore(&rmnet_mem_lock,flags);}struct page*
rmnet_mem_get_pages_entry(gfp_t gfp_mask,unsigned int order,int*code,int*
pageorder,unsigned id){unsigned long flags;struct mem_info*mem_page;struct page*
page=NULL;int i=(0xd2d+202-0xdf7);int j=(0xd2d+202-0xdf7);spin_lock_irqsave(&
rmnet_mem_lock,flags);if(order<POOL_LEN){rmnet_mem_id_req[id]++;
rmnet_mem_order_requests[order]++;for(j=order;j>(0xd2d+202-0xdf7)&&j<POOL_LEN;j
++){do{mem_page=list_first_entry_or_null(&rmnet_mem_pool[j],struct mem_info,
mem_head);if(!mem_page){break;}if(page_ref_count(mem_page->addr)==
(0xd26+209-0xdf6)){rmnet_mem_id_recycled[j]++;page=mem_page->addr;page_ref_inc(
mem_page->addr);list_rotate_left(&rmnet_mem_pool[j]);break;}list_rotate_left(&
rmnet_mem_pool[j]);i++;}while(i<=(0xd0a+237-0xdf2));if(page&&pageorder){*
pageorder=j;break;}i=(0xd2d+202-0xdf7);}}if(!page){DATARMNETfb2a1a4560[id]++;if(
order<(0xd18+223-0xdf4)){page=__dev_alloc_pages(GFP_ATOMIC,order);if(page){if(
static_pool_size[order]<max_pool_size[order]&&pool_unbound_feature[order]){
rmnet_mem_add_page(page,order);page_ref_inc(page);}if(pageorder){*pageorder=
order;}}}else{if(static_pool_size[order]<max_pool_size[order]&&
pool_unbound_feature[order]){page=__dev_alloc_pages(GFP_ATOMIC,order);if(page){
rmnet_mem_add_page(page,order);page_ref_inc(page);}if(pageorder){*pageorder=
order;}}}}spin_unlock_irqrestore(&rmnet_mem_lock,flags);if(pageorder&&code&&page
){if(*pageorder==order)*code=RMNET_MEM_SUCCESS;else if(*pageorder>order)*code=
RMNET_MEM_UPGRADE;else if(*pageorder<order)*code=RMNET_MEM_DOWNGRADE;}else if(
pageorder&&code){*code=RMNET_MEM_FAIL;*pageorder=(0xd2d+202-0xdf7);}return page;
}EXPORT_SYMBOL(rmnet_mem_get_pages_entry);void rmnet_mem_put_page_entry(struct 
page*page){put_page(page);}EXPORT_SYMBOL(rmnet_mem_put_page_entry);static void 
mem_update_pool_work(struct work_struct*work){int i;for(i=(0xd2d+202-0xdf7);i<
POOL_LEN;i++){local_bh_disable();rmnet_mem_adjust(target_static_pool_size[i],i);
if(i==POOL_NOTIF){rmnet_mem_mode_notify(target_static_pool_size[i]);}
local_bh_enable();}}void rmnet_mem_adjust(unsigned perm_size,u8 pageorder){
struct list_head*entry,*next;struct mem_info*mem_slot;int i;struct page*newpage=
NULL;int adjustment;unsigned long flags;if(pageorder>=POOL_LEN||perm_size>
MAX_STATIC_POOL)return;adjustment=perm_size-static_pool_size[pageorder];if(
perm_size==static_pool_size[pageorder])return;spin_lock_irqsave(&rmnet_mem_lock,
flags);if(perm_size>static_pool_size[pageorder]){for(i=(0xd2d+202-0xdf7);i<(
adjustment);i++){newpage=__dev_alloc_pages(GFP_ATOMIC,pageorder);if(!newpage){
continue;}rmnet_mem_add_page(newpage,pageorder);}}else{list_for_each_safe(entry,
next,&(rmnet_mem_pool[pageorder])){mem_slot=list_entry(entry,struct mem_info,
mem_head);list_del(&mem_slot->mem_head);put_page(mem_slot->addr);kfree(mem_slot)
;static_pool_size[pageorder]--;if(static_pool_size[pageorder]==perm_size)break;}
}spin_unlock_irqrestore(&rmnet_mem_lock,flags);}int __init rmnet_mem_module_init
(void){int rc=(0xd2d+202-0xdf7);int i=(0xd2d+202-0xdf7);pr_info(
"\x25\x73\x28\x29\x3a\x20\x53\x74\x61\x72\x74\x69\x6e\x67\x20\x72\x6d\x6e\x65\x74\x20\x6d\x65\x6d\x20\x6d\x6f\x64\x75\x6c\x65" "\n"
,__func__);for(i=(0xd2d+202-0xdf7);i<POOL_LEN;i++){INIT_LIST_HEAD(&(
rmnet_mem_pool[i]));}mem_wq=alloc_workqueue("\x6d\x65\x6d\x5f\x77\x71",
WQ_HIGHPRI,(0xd2d+202-0xdf7));if(!mem_wq){pr_err(
"\x25\x73\x28\x29\x3a\x20\x46\x61\x69\x6c\x65\x64\x20\x74\x6f\x20\x61\x6c\x6c\x6f\x63\x20\x77\x6f\x72\x6b\x71\x75\x65\x75\x65\x20" "\n"
,__func__);return-ENOMEM;}INIT_WORK(&pool_adjust_work,mem_update_pool_work);rc=
rmnet_mem_nl_register();if(rc){pr_err(
"\x25\x73\x28\x29\x3a\x20\x46\x61\x69\x6c\x65\x64\x20\x74\x6f\x20\x72\x65\x67\x69\x73\x74\x65\x72\x20\x67\x65\x6e\x65\x72\x69\x63\x20\x6e\x65\x74\x6c\x69\x6e\x6b\x20\x66\x61\x6d\x69\x6c\x79" "\n"
,__func__);return-ENOMEM;}return(0xd2d+202-0xdf7);}void __exit 
rmnet_mem_module_exit(void){rmnet_mem_nl_unregister();if(mem_wq){
cancel_work_sync(&pool_adjust_work);drain_workqueue(mem_wq);destroy_workqueue(
mem_wq);mem_wq=NULL;}rmnet_mem_free_all();}module_init(rmnet_mem_module_init);
module_exit(rmnet_mem_module_exit);
