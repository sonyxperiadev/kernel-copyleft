load("//msm-kernel:target_variants.bzl", "get_all_variants")
load("//build/kernel/kleaf:kernel.bzl", "ddk_module")
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load(":bt_modules.bzl", "bt_modules")

def _get_config_choices(config_srcs, options):
    choices = []

    for option in config_srcs:
        choices.extend(config_srcs[option].get(option in options, []))

    return choices

def _get_module_srcs(module, options):
    """
    Gets all the module sources, formats them with the path for that module
    and then groups them together
    It also includes all the headers within the `include` directory
    `native.glob()` returns a new list with every file need for the current package
    """
    srcs = module.srcs + _get_config_choices(module.config_srcs, options)
    return native.glob(
        ["{}/{}".format(module.path, src) for src in srcs] + ["include/*.h"]
    )

def _get_module_deps(module, options, formatter):
    """
    Formats the dependent targets with the necessary prefix
    Args:
        module: kernel module
        options: dependencies that rely on a config option
        formatter: function that will replace the format string within `deps`
    Example:
        kernel build = "pineapple_gki"
        dep = "%b_btpower"
        The formatted string will look as follow
        formatted_dep = formatter(dep) = "pineapple_gki_btpower"
    """
    deps = module.deps + _get_config_choices(module.config_deps, options)
    return [formatter(dep) for dep in deps]

def _get_build_options(modules, config_options):
    all_options = {option: True for option in config_options}
    all_options = all_options | {module.config_opt: True for module in modules if module.config_opt}

    return all_options

def define_target_variant_modules(target, variant, modules, config_options = []):
    """
    Generates the ddk_module for each of our kernel modules
    Args:
        target: either `pineapple` or `kalama`
        variant: either `gki` or `consolidate`
        modules: bt_modules dictionary defined in `bt_modules.bzl`
        config_options: decides which kernel modules to build
    """
    kernel_build = "{}_{}".format(target, variant)
    kernel_build_label = "//msm-kernel:{}".format(kernel_build)
    modules = [bt_modules.get(module_name) for module_name in modules]
    options = _get_build_options(modules, config_options)
    formatter = lambda s : s.replace("%b", kernel_build)

    all_modules = []
    for module in modules:
        rule_name = "{}_{}".format(kernel_build, module.name)
        module_srcs = _get_module_srcs(module, options)

        ddk_module(
            name = rule_name,
            kernel_build = kernel_build_label,
            srcs = module_srcs,
            out = "{}.ko".format(module.name),
            deps = ["//msm-kernel:all_headers"] + _get_module_deps(module, options, formatter),
            includes = ["include"],
            local_defines = options.keys(),
            visibility = ["//visibility:public"],
        )

        all_modules.append(rule_name)

    copy_to_dist_dir(
        name = "{}_bt-kernel_dist".format(kernel_build),
        data = all_modules,
        dist_dir = "out/target/product/{}/dlkm/lib/modules".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info",
    )

def define_bt_modules(target, modules, config_options = []):
    for (t, v) in get_all_variants():
        if t == target:
            define_target_variant_modules(t, v, modules, config_options)
