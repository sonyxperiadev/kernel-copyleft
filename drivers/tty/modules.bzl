load(":drivers/tty/hvc/modules.bzl", register_hvc = "register_modules")
load(":drivers/tty/serial/modules.bzl", register_serial = "register_modules")

def register_modules(registry):
    register_hvc(registry)
    register_serial(registry)
