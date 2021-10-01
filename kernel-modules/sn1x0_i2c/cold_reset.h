/******************************************************************************
 *  Copyright (C) 2020 NXP
 *   *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/
#ifndef _NFC_COMMON_H_
#define _NFC_COMMON_H_

#define MSG_NFCC_RSP              0x40
#define MSG_PROP_GID              0x0F
#define ESE_CLD_RST_OID           0x1E
#define RST_PROTECTION_CMD_INDEX  0x03

#define RST_PROTECTION_OID        0x1F
#define RST_PROTECTION_ENABLED    0x08

typedef enum ese_cold_reset_origin {
    ESE_COLD_RESET_NOT_REQUESTED = 0x00,
    ESE_COLD_RESET_SOURCE_NFC = 0x01,
    ESE_COLD_RESET_SOURCE_SPI = 0x02,
    ESE_COLD_RESET_SOURCE_UWB = 0x04,
}ese_cold_reset_origin_t;

void ese_reset_resource_init(void);
void ese_reset_resource_destroy(void);
#endif /* _NFC_COMMON_H_ */
