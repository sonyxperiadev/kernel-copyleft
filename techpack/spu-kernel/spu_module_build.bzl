load("//build/kernel/kleaf:kernel.bzl", "kernel_module",
                                        "kernel_modules_install",
                                        "ddk_module",
                                        "ddk_submodule")
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")

def _register_module_to_map(module_map, name, path, config_option, srcs, config_srcs, deps):
    processed_config_srcs = {}

    for config_src_name in config_srcs:
        config_src = config_srcs[config_src_name]

        if type(config_src) == "list":
            processed_config_srcs[config_src_name] = { True: config_src }
        else:
            processed_config_srcs[config_src_name] = config_src

    module = struct(
        name = name,
        path = path if path else ".",
        srcs = srcs,
        config_srcs = processed_config_srcs,
        config_option = config_option,
        deps = deps,
    )

    module_map[name] = module

def _get_kernel_build_module_srcs(kernel_build, module, formatter):
    src_formatter = lambda srcs: native.glob(formatter(["{}/{}".format(module.path, src) for src in srcs]))
    srcs = [] + src_formatter(module.srcs)

    return srcs

def _get_kernel_build_options(modules, config_options):
    all_options = {option: True for option in config_options}
    all_options = all_options | {module.config_option: True for module in modules if module.config_option}
    return all_options

def _get_kernel_build_module_deps(module, options, formatter):
    deps = [formatter(dep) for dep in deps]

    return deps

def spu_driver_module_entry(hdrs = []):
    module_map = {}

    def register(name, path = None, config_option = None, srcs = [], config_srcs = {}, deps = []):
        _register_module_to_map(module_map, name, path, config_option, srcs, config_srcs, deps)

    return struct(
        register = register,
        get = module_map.get,
        hdrs = hdrs,
        module_map = module_map
    )

def define_target_variant_modules(target, variant, registry, modules, config_options = []):
    kernel_build = "{}_{}".format(target, variant)
    kernel_build_label = "//msm-kernel:{}".format(kernel_build)
    modules = [registry.get(module_name) for module_name in modules]
    options = _get_kernel_build_options(modules, config_options)
    formatter = lambda strs : [s.replace("%b", kernel_build).replace("%t", target) for s in strs]
    headers = ["//msm-kernel:all_headers"] + registry.hdrs
    all_module_rules = []

    for module in modules:
        rule_name = "{}_{}".format(kernel_build, module.name)
        srcs = _get_kernel_build_module_srcs(kernel_build, module, formatter)

        ddk_submodule(
            name = rule_name,
            srcs = srcs,
            out = "{}.ko".format(module.name),
            deps = headers + formatter(module.deps),
            local_defines = options.keys()
        )

        all_module_rules.append(rule_name)

    ddk_module(
        name = "{}_spu-drivers".format(kernel_build),
        kernel_build = kernel_build_label,
        deps = all_module_rules
    )

    copy_to_dist_dir(
        name = "{}_spu-drivers_dist".format(kernel_build),
        data = [":{}_spu-drivers".format(kernel_build)],
        dist_dir = "../vendor/qcom/opensource/spu-drivers/out", ## TODO
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info",
    )

def define_consolidate_gki_modules(target, registry, modules, config_options = []):
    define_target_variant_modules(target, "consolidate", registry, modules, config_options)
    define_target_variant_modules(target, "gki", registry, modules, config_options)
