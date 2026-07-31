def register_modules(registry):
    registry.register(
        name = "drivers/misc/lkdtm/lkdtm",
        out = "lkdtm.ko",
        config = "CONFIG_LKDTM",
        srcs = [
            # do not sort
            "drivers/misc/lkdtm/core.c",
            "drivers/misc/lkdtm/bugs.c",
            "drivers/misc/lkdtm/heap.c",
            "drivers/misc/lkdtm/perms.c",
            "drivers/misc/lkdtm/rodata.c",
            "drivers/misc/lkdtm/refcount.c",
            "drivers/misc/lkdtm/usercopy.c",
            "drivers/misc/lkdtm/stackleak.c",
            "drivers/misc/lkdtm/cfi.c",
            "drivers/misc/lkdtm/fortify.c",
            "drivers/misc/lkdtm/lkdtm.h",
        ],
        conditional_srcs = {
            "CONFIG_PPC_64S_HASH_MMU": {
                True: [
                    # do not sort
                    "drivers/misc/lkdtm/powerpc.c",
                    "drivers/misc/lkdtm/lkdtm.h",
                ],
            },
        },
    )
