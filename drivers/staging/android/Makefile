ccflags-y += -I$(src)			# needed for trace events

obj-y					+= ion/

obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_VSOC)		+= vsoc.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
lowmemorykiller_tng := lowmemorykiller_tng.o lowmemorykiller_tasks.o lowmemorykiller_oom.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER_TNG) += lowmemorykiller_tng
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER_STATS)	+= lowmemorykiller_stats.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER_TNG_VMPRESSURE) += lowmemorykiller_vmpressure.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER_TNG_PSI) += lowmemorykiller_psi.o

