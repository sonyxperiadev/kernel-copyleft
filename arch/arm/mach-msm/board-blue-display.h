/* kernel/arch/arm/mach-msm/board-sony_blue-display.h
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_SONY_BLUE_DISPLAY_H
#define __ARCH_ARM_MACH_MSM_BOARD_SONY_BLUE_DISPLAY_H

#ifdef CONFIG_FB_MSM_MIPI_DSI_RENESAS_R63306
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_TMD_MDW30
extern const struct panel_id tmd_mdw30_panel_default_no_id;
extern const struct panel_id tmd_mdw30_panel_default_id_old;
extern const struct panel_id tmd_mdw30_panel_default;
extern const struct panel_id tmd_mdw30_panel_id_old;
extern const struct panel_id tmd_mdw30_panel_id_1a;
extern const struct panel_id tmd_mdw30_panel_id_1c;
extern const struct panel_id tmd_mdw30_panel_id_1e;
extern const struct panel_id tmd_mdw30_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_TMD_MDW30 */
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_TMD_MDX80
extern const struct panel_id tmd_mdx80_panel_default;
extern const struct panel_id tmd_mdx80_panel_id_1a;
extern const struct panel_id tmd_mdx80_panel_id;
extern const struct panel_id tmd_mdx80_panel_id_att;
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_TMD_MDX80 */
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_HITACHI_DX12D900VM
extern const struct panel_id hitachi_dx12d900vm_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_HITACHI_DX12D900VM */
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SX01
extern const struct panel_id sharp_ls046k3sx01_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SX01 */
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01
extern const struct panel_id sharp_ls043k3sx01_panel_default_old;
extern const struct panel_id sharp_ls043k3sx01_panel_default;
extern const struct panel_id sharp_ls043k3sx01_panel_id_old;
extern const struct panel_id sharp_ls043k3sx01_panel_id_1a;
extern const struct panel_id sharp_ls043k3sx01_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01 */
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04
extern const struct panel_id sharp_ls043k3sx04_panel_default;
extern const struct panel_id sharp_ls043k3sx04_panel_id_1a;
extern const struct panel_id sharp_ls043k3sx04_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04 */
#endif /* CONFIG_FB_MSM_MIPI_DSI_RENESAS_R63306 */

#ifdef CONFIG_FB_MSM_MIPI_DSI_SAMSUNG_S6D6AA0
#ifdef CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_SONY_ACX439AKM
extern const struct panel_id sony_acx439akm_panel_default_id_old;
extern const struct panel_id sony_acx439akm_panel_id_old;
extern const struct panel_id sony_acx439akm_panel_id_1a;
extern const struct panel_id sony_acx439akm_panel_id_1e;
#endif /* CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_SONY_ACX439AKM */
#ifdef CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_SONY_ACX444AKM
extern const struct panel_id sony_acx444akm_panel_default;
extern const struct panel_id sony_acx444akm_panel_id_1b;
extern const struct panel_id sony_acx444akm_panel_id_1c;
extern const struct panel_id sony_acx444akm_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_SONY_ACX444AKM */
#ifdef CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_SONY_ACX451AKM
extern const struct panel_id sony_acx451akm_panel_default;
extern const struct panel_id sony_acx451akm_panel_id_1a;
extern const struct panel_id sony_acx451akm_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_SONY_ACX451AKM */
#endif /* CONFIG_FB_MSM_MIPI_DSI_SAMSUNG_S6D6AA0 */

#ifdef CONFIG_FB_MSM_MIPI_DSI_NOVATEK_NT35565
#ifdef CONFIG_FB_MSM_MIPI_NT35565_PANEL_SHARP_LS038K3SX01
extern const struct panel_id sharp_ls038k3sx01_panel_default;
extern const struct panel_id sharp_ls038k3sx01_panel_id_1a;
extern const struct panel_id sharp_ls038k3sx01_panel_id_1b;
extern const struct panel_id sharp_ls038k3sx01_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_NT35565_PANEL_SHARP_LS038K3SX01 */
#endif /* CONFIG_FB_MSM_MIPI_DSI_NOVATEK_NT35565 */

#endif
