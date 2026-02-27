load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//build/kernel/kleaf:kernel.bzl", "ddk_module",
                                        "ddk_submodule")

def _create_module_conditional_src_map(conditional_srcs):
    processed_conditional_srcs = {}

    for conditional_src_name in conditional_srcs:
        conditional_src = conditional_srcs[conditional_src_name]

        if type(conditional_src) == "list":
            processed_conditional_srcs[conditional_src_name] = { True: conditional_src }
        else:
            processed_conditional_srcs[conditional_src_name] = conditional_src

    return processed_conditional_srcs

def _get_enabled_module_objs(registry, modules):
    undefined_modules = []
    enabled_module_objs = []

    for module_name in modules:
        module_obj = registry.get(module_name)

        if not module_obj:
            undefined_modules.append(module_name)
        else:
            enabled_module_objs.append(module_obj)

    if undefined_modules:
        fail("FAILED. Tried to enable the following undefined modules: \n{}".format("\n".join(undefined_modules)))
    else:
        return enabled_module_objs

def _get_module_srcs(module, options):
    srcs = [] + module.srcs
    module_path = "{}/".format(module.path) if module.path else ""

    for option in module.conditional_srcs:
        is_option_enabled = option in options
        srcs.extend(module.conditional_srcs[option].get(is_option_enabled, []))

    return ["{}{}".format(module_path, src) for src in srcs]

def _combine_target_module_options(enabled_modules, config_options):
    all_options = {option: True for option in config_options}
    modules_options = {module.config_option: True for module in enabled_modules if module.config_option}

    return all_options | modules_options

def _define_target_modules(target, variant, registry, modules, product = None, config_options = []):
    dep_formatter = lambda s : s.replace("%t", target)\
                                .replace("%v", variant)\
                                .replace("%p", product if product else "")\
                                .replace("%b", "{}_{}".format(target, variant))
    rule_prefix = "{}_{}_{}".format(target, variant, product) if product else "{}_{}".format(target, variant)
    enabled_modules = _get_enabled_module_objs(registry, modules)
    options = _combine_target_module_options(enabled_modules, config_options)
    headers = ["//msm-kernel:all_headers"] + registry.hdrs
    submodule_rules = []

    for module in enabled_modules:
        rule_name = "{}_{}".format(rule_prefix, module.name)
        srcs = _get_module_srcs(module, options)
        deps = headers + [dep_formatter(dep) for dep in module.deps]

        if not srcs:
            continue

        ddk_submodule(
            name = rule_name,
            srcs = srcs,
            out = "{}.ko".format(module.name),
            deps = deps,
            local_defines = options.keys(),
        )

        submodule_rules.append(rule_name)

    ddk_module(
        name = "{}_modules".format(rule_prefix),
        kernel_build = "//msm-kernel:{}_{}".format(target, variant),
        deps = submodule_rules
    )

    copy_to_dist_dir(
        name = "{}_modules_dist".format(rule_prefix),
        data = [":{}_modules".format(rule_prefix)],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info"
    )

def create_module_registry(hdrs = []):
    module_map = {}

    def register(name, path = None, config_option = None, srcs = [], conditional_srcs = {}, deps = []):
        module_map[name] = struct(
            name = name,
            path = path,
            srcs = srcs,
            conditional_srcs = _create_module_conditional_src_map(conditional_srcs),
            config_option = config_option,
            deps = deps,
        )

    return struct(
        module_map = module_map,
        hdrs = hdrs,
        register = register,
        get = module_map.get,
    )

def define_target_modules(target, variants, registry, modules, config_options = [], products = []):
    for variant in variants:
        if products:
            for product in products:
                _define_target_modules(target = target,
                                       variant = variant,
                                       registry = registry,
                                       modules = modules,
                                       product = product,
                                       config_options = config_options)
        else:
            _define_target_modules(target = target,
                                   variant = variant,
                                   registry = registry,
                                   modules = modules,
                                   config_options = config_options)
