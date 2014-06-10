/* linux/drivers/misc/isdb-t/tuner_register.h
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Tatsuya Ooka <Tatsuya.Ooka@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#ifndef _TUNER_REG_H
#define _TUNER_REG_H

/* Register address */
/* Main#1 */
#define TUNER_ADR_DTVSET_F      0x00
#define TUNER_ADR_SYSSET_F      0x02
#define TUNER_ADR_NCOFREQU_F    0x15
#define TUNER_ADR_NCOFREQM_F    0x16
#define TUNER_ADR_NCOFREQL_F    0x17
#define TUNER_ADR_FADU_F        0x18
#define TUNER_ADR_FADM_F        0x19
#define TUNER_ADR_FADL_F        0x1A
#define TUNER_ADR_SPSYNCSET_F   0x26
#define TUNER_ADR_FECTSSET_F    0x30
#define TUNER_ADR_MFECSET_F     0x31
#define TUNER_ADR_SYN2RDSET_F   0x88
#define TUNER_ADR_ACRDSET_F     0x89
#define TUNER_ADR_CNSET_F       0x8D
#define TUNER_ADR_BERSET1_F     0x90
#define TUNER_ADR_BERSET2_F     0x91
#define TUNER_ADR_BERSET3_F     0x92
#define TUNER_ADR_BERSTSET_F    0x93
#define TUNER_ADR_BERRDSET_F    0x95
#define TUNER_ADR_MDRD_F        0x96
#define TUNER_ADR_SSEQRD_F      0x97
#define TUNER_ADR_SYN2FLG_F     0x99
#define TUNER_ADR_ACEMGFLG_F    0x9A
#define TUNER_ADR_TMCCD1_F      0xA4
#define TUNER_ADR_TMCCD8_F      0xAB
#define TUNER_ADR_ACRD1_F       0xAC
#define TUNER_ADR_BERFLG_F      0xBB
#define TUNER_ADR_BERRDU_F      0xBC
#define TUNER_ADR_BERLENRDL_F   0xC0
#define TUNER_ADR_DOSET1_F      0xCE
#define TUNER_ADR_DOSET2_F      0xCF
#define TUNER_ADR_DOSET3_F      0xD0
#define TUNER_ADR_DOSET4_F      0xD1
#define TUNER_ADR_DOSET5_F      0xD2
#define TUNER_ADR_INTDEF1_F     0xD3
#define TUNER_ADR_INTDEF2_F     0xD4
#define TUNER_ADR_INTSET1_F     0xD5
#define TUNER_ADR_INTSET2_F     0xD6
#define TUNER_ADR_INTSET3_F     0xD7
#define TUNER_ADR_INTSET4_F     0xD8
#define TUNER_ADR_INTSET5_F     0xD9
#define TUNER_ADR_INTCND_F      0xDA
#define TUNER_ADR_INTST1_F      0xDB
#define TUNER_ADR_STMSET1_F     0xDC
#define TUNER_ADR_STMSET2_F     0xDD
#define TUNER_ADR_INVSET_F      0xDE
#define TUNER_ADR_TSTPOSET_F    0xDF
#define TUNER_ADR_PSCOP1        0xE8
#define TUNER_ADR_PSCOP2        0xE9
#define TUNER_ADR_PSCOP3        0xEA
#define TUNER_ADR_PSCCTRL       0xF3
#define TUNER_ADR_PSCSET        0xF4
#define TUNER_ADR_PSCPRG        0xF5
#define TUNER_ADR_PSCFLG        0xF7
#define TUNER_ADR_CHIPRD_F      0xFF

/* Main#2 */
#define TUNER_ADR_SSEQRD_S      0x81
#define TUNER_ADR_LCKMON2_S     0x84
#define TUNER_ADR_BERSET_S      0xA3
#define TUNER_ADR_BERFLG_S      0xA5
#define TUNER_ADR_BERRDU_S      0xAC
#define TUNER_ADR_BERRDL_S      0xAD
#define TUNER_ADR_INTDEF_S      0xC0
#define TUNER_ADR_INTSET1_S     0xC1
#define TUNER_ADR_INTST2_S      0xC5

/* Sub */
#define TUNER_ADR_REG0          0xA0
#define TUNER_ADR_REG1          0xA1
#define TUNER_ADR_TNCOP8        0xD4

/* General purpose */
#define SIG_ENAS_ALL              (0)
#define SIG_ENAE_ALL              (7)
#define SIG_ENA_ALL               (0xFF)

/* INTDEF1_F */
#define SIG_ENAS_INTDEF1_F        (0)
#define SIG_ENAE_INTDEF1_F        (7)
#define SIG_ENA_INTDEF1_F         (0xFF)
#define SIG_ENAIRQ_INTDEF1_NONE   (0x00)

/* INTDEF2_F */
#define SIG_ENAIRQ_INTDEF2_NONE   (0x00)

/* BERFLG_S */
#define SIG_ENAS_BERCHK_S     (1)
#define SIG_ENA_BERCHK_S      (0x02)
#define SIG_ENAS_BERRDY_S     (0)
#define SIG_ENA_BERRDY_S      (0x01)

/* BERSET1_F */
#define SIG_ENAS_BERDSEL_F    (6)
#define SIG_ENAE_BERDSEL_F    (7)
#define SIG_ENA_BERDSEL_F     (0xC0)
#define SIG_ENAS_BERVSEL_F    (5)
#define SIG_ENA_BERVSEL_F     (0x20)
#define SIG_ENAS_BERTIMMD_F   (4)
#define SIG_ENA_BERTIMMD_F    (0x10)
#define SIG_ENAS_BERTIM_F     (0)
#define SIG_ENA_BERTIM_F      (0x0F)

/* BERSET2_F */
#define SIG_ENAS_BERVLENA_F   (0)
#define SIG_ENAE_BERVLENA_F   (3)
#define SIG_ENA_BERVLENA_F    (0x0F)

/* BERSET3_F */
#define SIG_ENAS_BERVLENB_F   (4)
#define SIG_ENAE_BERVLENB_F   (7)
#define SIG_ENA_BERVLENB_F    (0xF0)
#define SIG_ENAS_BERVLENC_F   (0)
#define SIG_ENA_BERVLENC_F    (0x0F)

/* BERRDSET_F */
#define SIG_ENAS_BERRDSEL_F   (2)
#define SIG_ENAE_BERRDSEL_F   (3)
#define SIG_ENA_BERRDSEL_F    (0x0C)
#define SIG_ENAS_BERLAY_F     (0)
#define SIG_ENA_BERLAY_F      (0x03)

/* BERFLG_F */
#define SIG_ENAS_BERDRDY_F    (7)
#define SIG_ENA_BERDRDY_F     (0x80)
#define SIG_ENAS_BERDCHK_F    (6)
#define SIG_ENA_BERDCHK_F     (0x40)
#define SIG_ENAS_BERVRDYA_F   (5)
#define SIG_ENA_BERVRDYA_F    (0x20)
#define SIG_ENAS_BERVCHKA_F   (4)
#define SIG_ENA_BERVCHKA_F    (0x10)
#define SIG_ENAS_BERVRDYB_F   (3)
#define SIG_ENA_BERVRDYB_F    (0x08)
#define SIG_ENAS_BERVCHKB_F   (2)
#define SIG_ENA_BERVCHKB_F    (0x04)
#define SIG_ENAS_BERVRDYC_F   (1)
#define SIG_ENA_BERVRDYC_F    (0x02)
#define SIG_ENAS_BERVCHKC_F   (0)
#define SIG_ENA_BERVCHKC_F    (0x01)

/* BERSET_S */
#define SIG_ENAS_BERST_S     (1)
#define SIG_ENA_BERST_S      (0x02)
#define SIG_PARAM_BERST_S    ((0)<<SIG_ENAS_BERST_S)
#define SIG_ENAS_BERLAY_S     (2)
#define SIG_ENA_BERLAY_S      (0x04)
#define SIG_PARAM_BERLAY_S    (0<<SIG_ENAS_BERLAY_S)
#define SIG_ENAS_BERSEL_S     (3)
#define SIG_ENA_BERSEL_S      (0x08)
#define SIG_ENAS_BERLEN_S     (4)
#define SIG_ENAE_BERLEN_S     (7)
#define SIG_ENA_BERLEN_S      (0xF0)

/* TNCOP8 */
#define SIG_ENAS_RSSI         (0)
#define SIG_ENAE_RSSI         (7)
#define SIG_ENA_RSSI          (0xFF)

/* REG0 */
#define SIG_ENAS_CLPON        (1)
#define SIG_ENAE_CLPON        (1)
#define SIG_ENA_CLPON         (0x02)

/* REG1 */
#define SIG_ENAS_RLPON        (1)
#define SIG_ENAE_RLPON        (1)
#define SIG_ENA_RLPON         (0x02)

#endif /* _TUNER_REG_H */
