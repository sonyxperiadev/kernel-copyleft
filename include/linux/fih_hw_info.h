/*
*Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved
*/
#ifndef _FIH_HW_INFO_H
#define _FIH_HW_INFO_H

typedef struct
{
  unsigned int ID;
  char* STR;
  int LEN; //string lengh
}ID2STR_MAP;

typedef enum
{
  PROJECT_SEAGULL =0x01,
  PROJECT_END = 0xFE,
  PROJECT_MAX = 0xFF,
} fih_product_id_type;

//notice that the phase must be ordered
typedef enum
{
  PHASE_EVM  = 0x00,
  PHASE_EVM2 = 0x01,
  PHASE_PD   = 0x10,
  PHASE_DP   = 0x20,
  PHASE_SP   = 0x30,
  PHASE_SP2  = 0x31,
  PHASE_AP   = 0x40,
  PHASE_PQ   = 0x50,
  PHASE_TP   = 0x60,
  PHASE_MP   = 0x70,
  PHASE_MP_PCN3000 = 0x80,
  PHASE_END  = 0xFE,
  PHASE_MAX  = 0XFF,
}fih_product_phase_type;

typedef enum
{
  BAND_125     = 0x01,
  BAND_1258    = 0x02,
  BAND_1245    = 0x03,
  BAND_1357820 = 0x04,
  BAND_END     = 0xFE,
  BAND_MAX     = 0xFF,
}fih_band_id_type;

typedef enum
{
  SINGLE_SIM =0x01,
  DUAL_SIM   =0x02,
  SIM_END    =0xFE,
  SIM_MAX    =0xFF,
}fih_sim_type;

//============================

static const ID2STR_MAP project_id_map[] =
{
  {PROJECT_SEAGULL, "Seagull", 7},
  {PROJECT_MAX,     "unknown", 7}
};

static const ID2STR_MAP phase_id_map[] =
{
  {PHASE_EVM,  "EVM",  3},
  {PHASE_EVM2, "EVM2", 4},
  {PHASE_PD,   "PD",   2},
  {PHASE_DP,   "DP",   2},
  {PHASE_SP,   "SP",   2},
  {PHASE_SP2,  "SP2",  3},
  {PHASE_AP,   "AP",   2},
  {PHASE_PQ,   "PQ",   2},
  {PHASE_TP,   "TP",   2},
  {PHASE_MP,   "MP",   2},
  {PHASE_MP_PCN3000, "MP_PCN3000", 10},
  {PHASE_END,  "END",  3},
  {PHASE_MAX, "unknown", 7}
};

static const ID2STR_MAP band_id_map[] =
{
  {BAND_125,     "125",     3},
  {BAND_1258,    "1258",    4},
  {BAND_1245,    "1245",    4},
  {BAND_1357820, "1357820", 7},
  {BAND_MAX,     "unknown", 7}
};

static const ID2STR_MAP sim_id_map[] =
{
  {SINGLE_SIM, "SINGLE",  6},
  {DUAL_SIM,   "DUAL",    4},
  {SIM_MAX,    "unknown", 7}
};

//============================
void fih_set_oem_info(void);
void fih_info_init(void);

unsigned int fih_get_product_id(void);
unsigned int fih_get_product_phase(void);
unsigned int fih_get_band_id(void);
unsigned int fih_get_sim_id(void);
int fih_get_hwid_adc(int index);

int fih_phase_match(fih_product_phase_type start,fih_product_phase_type end);
int fih_dts_phase_match(const char* start, const char* end);
void fih_dts_phase_cfg(fih_product_phase_type start,fih_product_phase_type end, char* str);
void fih_hwid_get(void);
char *fih_get_nonHLOS_version(void);
char *fih_get_nonHLOS_git_head(void);
void fih_get_nonHLOS_info(void);

#endif
