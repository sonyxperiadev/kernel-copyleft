/* kernel/arch/arm/mach-msm/board-viskan-display.h
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_SONY_VISKAN_DISPLAY_H
#define __ARCH_ARM_MACH_MSM_BOARD_SONY_VISKAN_DISPLAY_H

#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SY01
extern const struct panel_id sharp_ls046k3sy01_panel_id_1a;
extern const struct panel_id sharp_ls046k3sy01_panel_id;
extern const struct panel_id sharp_ls046k3sy01_panel_default;
#endif /*CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SY01*/
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04
extern const struct panel_id sharp_ls043k3sx04_panel_default;
extern const struct panel_id sharp_ls043k3sx04_panel_id_1a;
extern const struct panel_id sharp_ls043k3sx04_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04 */
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01
extern const struct panel_id sharp_ls043k3sx01_panel_default_old;
extern const struct panel_id sharp_ls043k3sx01_panel_default;
extern const struct panel_id sharp_ls043k3sx01_panel_id_1a;
extern const struct panel_id sharp_ls043k3sx01_panel_id;
extern const struct panel_id sharp_ls043k3sx01_panel_id_old;
#endif /*CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01*/
#ifdef CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_AUO_H455TVN01
extern const struct panel_id auo_h455tvn01_panel_default;
extern const struct panel_id auo_h455tvn01_panel_id_1a;
extern const struct panel_id auo_h455tvn01_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_AUO_H455TVN01 */

#endif
