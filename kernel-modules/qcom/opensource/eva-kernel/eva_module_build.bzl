load(
    "//build/kernel/kleaf:kernel.bzl",
    "ddk_module",
    "ddk_submodule",
    "kernel_module",
    "kernel_modules_install",
)
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")

def _register_module_to_map(module_map, name, path, config_option, srcs, config_srcs, deps, config_deps):
    processed_config_srcs = {}
    processed_config_deps = {}

    for config_src_name in config_srcs:
        config_src = config_srcs[config_src_name]

        if type(config_src) == "list":
            processed_config_srcs[config_src_name] = {True: config_src}
        else:
            processed_config_srcs[config_src_name] = config_src

    for config_deps_name in config_deps:
        config_dep = config_deps[config_deps_name]

        if type(config_dep) == "list":
            processed_config_deps[config_deps_name] = {True: config_dep}
        else:
            processed_config_deps[config_deps_name] = config_dep

    module = struct(
        name = name,
        path = path,
        srcs = srcs,
        config_srcs = processed_config_srcs,
        config_option = config_option,
        deps = deps,
        config_deps = processed_config_deps,
    )

    module_map[name] = module

def _get_config_choices(map, options):
    choices = []

    for option in map:
        choices.extend(map[option].get(option in options, []))

    return choices

def _get_kernel_build_options(modules, config_options):
    all_options = {option: True for option in config_options}
    all_options = all_options | {module.config_option: True for module in modules if module.config_option}

    return all_options

def _get_kernel_build_module_srcs(module, options, formatter):
    srcs = module.srcs + _get_config_choices(module.config_srcs, options)
    module_path = "{}/".format(module.path) if module.path else ""
    globbed_srcs = native.glob(["{}{}".format(module_path, formatter(src)) for src in srcs])

    return globbed_srcs

def _get_kernel_build_module_deps(module, options, formatter):
    deps = module.deps + _get_config_choices(module.config_deps, options)
    deps = [formatter(dep) for dep in deps]

    return deps

def create_module_registry(hdrs = []):
    module_map = {}

    def register(name, path = None, config_option = None, srcs = [], config_srcs = {}, deps = [], config_deps = {}):
        _register_module_to_map(module_map, name, path, config_option, srcs, config_srcs, deps, config_deps)

    return struct(
        register = register,
        get = module_map.get,
        hdrs = hdrs,
        module_map = module_map,
    )

def define_target_variant_modules(target, variant, registry, modules, config_options = []):
    kernel_build = "{}_{}".format(target, variant)
    kernel_build_label = "//msm-kernel:{}".format(kernel_build)
    modules = [registry.get(module_name) for module_name in modules]
    options = _get_kernel_build_options(modules, config_options)
    build_print = lambda message: print("{}: {}".format(kernel_build, message))
    formatter = lambda s: s.replace("%b", kernel_build).replace("%t", target)

    headers = ["//msm-kernel:all_headers"] + registry.hdrs
    all_module_rules = []

    for module in modules:
        rule_name = "{}_{}".format(kernel_build, module.name)
        module_srcs = _get_kernel_build_module_srcs(module, options, formatter)

        if not module_srcs:
            continue

        ddk_submodule(
            name = rule_name,
            srcs = module_srcs,
            out = "{}.ko".format(module.name),
            copts = ["-Wno-format"],
            deps = headers + _get_kernel_build_module_deps(module, options, formatter),
            local_defines = options.keys(),
        )

        all_module_rules.append(rule_name)

    ddk_module(
        name = "{}_modules".format(kernel_build),
        kernel_build = kernel_build_label,
        deps = all_module_rules,
    )

    copy_to_dist_dir(
        name = "{}_modules_dist".format(kernel_build),
        data = [":{}_modules".format(kernel_build)],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(kernel_build),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
    )


def define_consolidate_gki_modules(target, registry, modules, config_options = []):
    define_target_variant_modules(target, "consolidate", registry, modules, config_options)
    define_target_variant_modules(target, "gki", registry, modules, config_options)
