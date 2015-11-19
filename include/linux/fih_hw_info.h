/*
*Copyright(C) 2014-2015 Foxconn International Holdings, Ltd. All rights reserved
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
  PROJECT_TULIP =0x01,
  PROJECT_END = 0xFE,
  PROJECT_MAX = 0xFF,
} fih_product_id_type;

//notice that the phase must be ordered
typedef enum
{
  PHASE_EVM     = 0x00,
  PHASE_EVM2    = 0x01,
  PHASE_EVM3    = 0x02,
  PHASE_PD1     = 0x10,
  PHASE_PD2     = 0x11,
  PHASE_PD3     = 0x12,
  PHASE_PD4     = 0x13,
  PHASE_DP      = 0x20,
  PHASE_Pre_SP  = 0x29,
  PHASE_SP1_0   = 0x30,
  PHASE_SP1_2   = 0x31,
  PHASE_SP1_0_v2= 0x32,
  PHASE_SP1_2_v2= 0x33,
  PHASE_SP39_1_2= 0x34,
  PHASE_Pre_AP  = 0x39,
  PHASE_Pre_AP39= 0x3a,
  PHASE_AP      = 0x40,
  PHASE_TP      = 0x50,
  PHASE_TP_PI   = 0x51,
  PHASE_TP_ZDT  = 0x52,
  PHASE_PQ      = 0x60,
  PHASE_PQ_16G  = 0x68,
  PHASE_PQ2     = 0x69,
  PHASE_MP      = 0x70,
  PHASE_MP_16G  = 0x78,
  PHASE_END     = 0xFE,
  PHASE_MAX     = 0XFF,
}fih_product_phase_type;

typedef enum
{
  BAND_RITA = 0x01,
  BAND_GINA = 0x02,
  BAND_REX  = 0x03,
  BAND_APAC = 0x04,
  BAND_VIV  = 0x05,
  BAND_END  = 0xFE,
  BAND_MAX  = 0xFF,
}fih_band_id_type;

typedef enum
{
  SIM_SINGLE =0x01,
  SIM_DUAL   =0x02,
  SIM_END    =0xFE,
  SIM_MAX    =0xFF,
}fih_sim_type;

//============================

static const ID2STR_MAP project_id_map[] =
{
  {PROJECT_TULIP, "Tulip", 5},
  {PROJECT_MAX,   "unknown", 7}
};

static const ID2STR_MAP phase_id_map[] =
{
  {PHASE_EVM,     "EVM",     3},
  {PHASE_EVM2,    "EVM2",    4},
  {PHASE_EVM3,    "EVM3",    4},
  {PHASE_PD1,     "PD1",     3},
  {PHASE_PD2,     "PD2",     3},
  {PHASE_PD3,     "PD3",     3},
  {PHASE_PD4,     "PD4",     3},
  {PHASE_DP,      "DP",      2},
  {PHASE_Pre_SP,  "Pre-SP",  6},
  {PHASE_SP1_0,   "SP1.0",   5},
  {PHASE_SP1_2,   "SP1.2",   5},
  {PHASE_SP1_0_v2,"SP1.0v2" ,7},
  {PHASE_SP1_2_v2,"SP1.2v2" ,7},
  {PHASE_SP39_1_2,"SP39_1.2",8},
  {PHASE_Pre_AP,  "Pre-AP",  6},
  {PHASE_Pre_AP39,"Pre-AP39",8},
  {PHASE_AP,      "AP",      2},
  {PHASE_TP,      "TP",      2},
  {PHASE_TP_PI,   "TP-PI",   5},
  {PHASE_TP_ZDT,  "TP-ZDT",  6},
  {PHASE_PQ,      "PQ",      2},
  {PHASE_PQ_16G,  "PQ-16G",  6},
  {PHASE_PQ2,     "PQ2",     3},
  {PHASE_MP,      "MP",      2},
  {PHASE_MP_16G,  "MP-16G",  6},
  {PHASE_END,     "END",     3},
  {PHASE_MAX,     "unknown", 7}
};

static const ID2STR_MAP band_id_map[] =
{
  {BAND_RITA, "RITA", 4},
  {BAND_GINA, "GINA", 4},
  {BAND_REX,  "REX",  3},
  {BAND_APAC, "APAC", 4},
  {BAND_VIV,  "VIV",  3},
  {BAND_MAX,  "unknown", 7}
};

static const ID2STR_MAP sim_id_map[] =
{
  {SIM_SINGLE, "SINGLE",  6},
  {SIM_DUAL,   "DUAL",    4},
  {SIM_MAX,    "unknown", 7}
};

//============================
void fih_set_hwid2smem(void);
void fih_get_hwid_From_smem(void);/*BSP-ELuo-HWID_VERIFY-00*/
void fih_proc_init(void);

unsigned int fih_get_s1_boot(void);
unsigned int fih_get_product_id(void);
unsigned int fih_get_product_phase(void);
unsigned int fih_get_band_id(void);
unsigned int fih_get_sim_id(void);

int fih_phase_match(fih_product_phase_type start,fih_product_phase_type end);
int fih_dts_phase_match(const char* start, const char* end);
void fih_dts_phase_cfg(fih_product_phase_type start,fih_product_phase_type end, char* str);
void fih_hwid_get(void);
char *fih_get_nonHLOS_version(void);
char *fih_get_nonHLOS_git_head(void);
void fih_get_nonHLOS_info(void);

#endif
