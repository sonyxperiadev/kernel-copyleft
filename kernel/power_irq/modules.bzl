def register_modules(registry):
    registry.register(
        name = "kernel/power_irq/wakeup_irq_debug",
        out = "wakeup_irq_debug.ko",
        config = "CONFIG_WAKEUP_IRQ_DEBUG",
        srcs = [
            # do not sort
            "kernel/power_irq/wakeup_irq_debug.c",
        ],
    )
