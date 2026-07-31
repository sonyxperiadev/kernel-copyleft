def register_modules(registry):
    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-canoe",
        out = "pinctrl-canoe.ko",
        config = "CONFIG_PINCTRL_CANOE",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-canoe.c",
            "drivers/pinctrl/qcom/pinctrl-msm.h",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-msm",
        out = "pinctrl-msm.ko",
        config = "CONFIG_PINCTRL_MSM",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.c",
            "drivers/pinctrl/qcom/pinctrl-msm.h",
        ],
        deps = [
            # do not sort
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-spmi-gpio",
        out = "pinctrl-spmi-gpio.ko",
        config = "CONFIG_PINCTRL_QCOM_SPMI_PMIC",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-spmi-gpio.c",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-spmi-mpp",
        out = "pinctrl-spmi-mpp.ko",
        config = "CONFIG_PINCTRL_QCOM_SPMI_PMIC",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-spmi-mpp.c",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-sun",
        out = "pinctrl-sun.ko",
        config = "CONFIG_PINCTRL_SUN",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-sun.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-vienna",
        out = "pinctrl-vienna.ko",
        config = "CONFIG_PINCTRL_VIENNA",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-vienna.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-alor",
        out = "pinctrl-alor.ko",
        config = "CONFIG_PINCTRL_ALOR",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-alor.c",
            "drivers/pinctrl/qcom/pinctrl-msm.h",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-chora",
        out = "pinctrl-chora.ko",
        config = "CONFIG_PINCTRL_CHORA",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-chora.c",
            "drivers/pinctrl/qcom/pinctrl-msm.h",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-yupik",
        out = "pinctrl-yupik.ko",
        config = "CONFIG_PINCTRL_YUPIK",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-yupik.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-khaje",
        out = "pinctrl-khaje.ko",
        config = "CONFIG_PINCTRL_KHAJE",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-khaje.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-bengal",
        out = "pinctrl-bengal.ko",
        config = "CONFIG_PINCTRL_BENGAL",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-bengal.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-monaco",
        out = "pinctrl-monaco.ko",
        config = "CONFIG_PINCTRL_MONACO",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-monaco.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-malabar",
        out = "pinctrl-malabar.ko",
        config = "CONFIG_PINCTRL_MALABAR",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-malabar.c",
            "drivers/pinctrl/qcom/pinctrl-msm.h",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-direwolf",
        out = "pinctrl-direwolf.ko",
        config = "CONFIG_PINCTRL_DIREWOLF",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-direwolf.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-lemans",
        out = "pinctrl-lemans.ko",
        config = "CONFIG_PINCTRL_LEMANS",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-lemans.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-x1e80100",
        out = "pinctrl-x1e80100.ko",
        config = "CONFIG_PINCTRL_X1E80100",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-x1e80100.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-monaco_auto",
        out = "pinctrl-monaco_auto.ko",
        config = "CONFIG_PINCTRL_MONACO_AUTO",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-monaco_auto.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-sa8797p",
        out = "pinctrl-sa8797p.ko",
        config = "CONFIG_PINCTRL_SA8797P",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-sa8797p.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-pikachu",
        out = "pinctrl-pikachu.ko",
        config = "CONFIG_PINCTRL_PIKACHU",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-pikachu.c",
            "drivers/pinctrl/qcom/pinctrl-msm.h",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-shikra",
        out = "pinctrl-shikra.ko",
        config = "CONFIG_PINCTRL_SHIKRA",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm.h",
            "drivers/pinctrl/qcom/pinctrl-shikra.c",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/pinctrl/qcom/pinctrl-seraph",
        out = "pinctrl-seraph.ko",
        config = "CONFIG_PINCTRL_SERAPH",
        srcs = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-seraph.c",
            "drivers/pinctrl/qcom/pinctrl-msm.h",
        ],
        deps = [
            # do not sort
            "drivers/pinctrl/qcom/pinctrl-msm",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )
