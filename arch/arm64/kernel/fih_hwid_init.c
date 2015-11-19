/*
* Copyright(C) 2012-2013 Foxconn International Holdings, Ltd. All rights reserved
*/
#include <linux/err.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/export.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/fih_hw_info.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

//need to init the table to get correct gpio number
struct gpio HWID_GPIO[]=
{
  {  -1, GPIOF_DIR_IN, "fih_hwid_pin"}, //HW_ID_0
  {  -1, GPIOF_DIR_IN, "fih_hwid_pin"}, //HW_ID_1
  {  -1, GPIOF_DIR_IN, "fih_hwid_pin"}, //HW_ID_2
  {  -1, GPIOF_DIR_IN, "fih_hwid_pin"}, //HW_ID_3
  {  -1, GPIOF_DIR_IN, "fih_hwid_pin"}, //HW_ID_4
  {  -1, GPIOF_DIR_IN, "fih_hwid_pin"}, //HW_ID_5
  {  -1, GPIOF_DIR_IN, "fih_hwid_pin"}, //HW_ID_6  EVM is open pin
  {  -1, GPIOF_DIR_IN, "fih_hwid_pin"}  //HW_ID_7  EVM is open pin
};


typedef struct
{
  int HWID_LOGIC;
  fih_product_id_type PROJECT;
  fih_band_id_type BAND;
  fih_product_phase_type PHASE;
  fih_sim_type SIM;
}fih_hwid_map;


//============================
fih_hwid_map *HWID_MAP;

static fih_hwid_map HWID_MAP_8916[]=
{
  /* Tulip */
  //======Rita======
  {0x00, PROJECT_TULIP, BAND_RITA, PHASE_EVM,    SIM_SINGLE},
  {0x01, PROJECT_TULIP, BAND_RITA, PHASE_PD1,    SIM_SINGLE},
  {0x02, PROJECT_TULIP, BAND_RITA, PHASE_PD2,    SIM_SINGLE},
  {0x03, PROJECT_TULIP, BAND_RITA, PHASE_PD3,    SIM_SINGLE},
  {0x04, PROJECT_TULIP, BAND_RITA, PHASE_DP,     SIM_SINGLE},
  {0x05, PROJECT_TULIP, BAND_RITA, PHASE_Pre_SP, SIM_SINGLE},
  {0x06, PROJECT_TULIP, BAND_RITA, PHASE_SP1_0,  SIM_SINGLE},
  {0x07, PROJECT_TULIP, BAND_RITA, PHASE_AP,     SIM_SINGLE},
  {0x08, PROJECT_TULIP, BAND_RITA, PHASE_TP,     SIM_SINGLE},
  {0x09, PROJECT_TULIP, BAND_RITA, PHASE_PQ,     SIM_SINGLE},
  {0x0a, PROJECT_TULIP, BAND_RITA, PHASE_MP,     SIM_SINGLE},
  //===============
  {0x10, PROJECT_TULIP, BAND_RITA, PHASE_PD1,    SIM_DUAL  },
  {0x11, PROJECT_TULIP, BAND_RITA, PHASE_PD2,    SIM_DUAL  },
  {0x12, PROJECT_TULIP, BAND_RITA, PHASE_PD3,    SIM_DUAL  },
  {0x13, PROJECT_TULIP, BAND_RITA, PHASE_DP,     SIM_DUAL  },
  {0x14, PROJECT_TULIP, BAND_RITA, PHASE_Pre_SP, SIM_DUAL  },
  {0x15, PROJECT_TULIP, BAND_RITA, PHASE_SP1_0,  SIM_DUAL  },
  {0x16, PROJECT_TULIP, BAND_RITA, PHASE_AP,     SIM_DUAL  },
  {0x18, PROJECT_TULIP, BAND_RITA, PHASE_TP,     SIM_DUAL  },
  {0x19, PROJECT_TULIP, BAND_RITA, PHASE_PQ,     SIM_DUAL  },
  {0x1a, PROJECT_TULIP, BAND_RITA, PHASE_MP,     SIM_DUAL  },
  //======Gina======
  {0x20, PROJECT_TULIP, BAND_GINA, PHASE_PD1,    SIM_SINGLE},
  {0x21, PROJECT_TULIP, BAND_GINA, PHASE_PD2,    SIM_SINGLE},
  {0x22, PROJECT_TULIP, BAND_GINA, PHASE_PD3,    SIM_SINGLE},
  {0x23, PROJECT_TULIP, BAND_GINA, PHASE_DP,     SIM_SINGLE},
  {0x24, PROJECT_TULIP, BAND_GINA, PHASE_Pre_SP, SIM_SINGLE},
  {0x25, PROJECT_TULIP, BAND_GINA, PHASE_SP1_0,  SIM_SINGLE},
  {0x26, PROJECT_TULIP, BAND_GINA, PHASE_AP,     SIM_SINGLE},
  {0x28, PROJECT_TULIP, BAND_GINA, PHASE_TP,     SIM_SINGLE},
  {0x29, PROJECT_TULIP, BAND_GINA, PHASE_PQ,     SIM_SINGLE},
  {0x2a, PROJECT_TULIP, BAND_GINA, PHASE_MP,     SIM_SINGLE},
  //===============
  {0x30, PROJECT_TULIP, BAND_GINA, PHASE_PD1,    SIM_DUAL  },
  {0x31, PROJECT_TULIP, BAND_GINA, PHASE_PD2,    SIM_DUAL  },
  {0x32, PROJECT_TULIP, BAND_GINA, PHASE_PD3,    SIM_DUAL  },
  {0x33, PROJECT_TULIP, BAND_GINA, PHASE_DP,     SIM_DUAL  },
  {0x34, PROJECT_TULIP, BAND_GINA, PHASE_Pre_SP, SIM_DUAL  },
  {0x35, PROJECT_TULIP, BAND_GINA, PHASE_SP1_0,  SIM_DUAL  },
  {0x36, PROJECT_TULIP, BAND_GINA, PHASE_AP,     SIM_DUAL  },
  {0x38, PROJECT_TULIP, BAND_GINA, PHASE_TP,     SIM_DUAL  },
  {0x39, PROJECT_TULIP, BAND_GINA, PHASE_PQ,     SIM_DUAL  },
  {0x3a, PROJECT_TULIP, BAND_GINA, PHASE_MP,     SIM_DUAL  },
  //======Rex=======
  {0x40, PROJECT_TULIP, BAND_REX,  PHASE_PD1,    SIM_SINGLE},
  {0x41, PROJECT_TULIP, BAND_REX,  PHASE_PD2,    SIM_SINGLE},
  {0x42, PROJECT_TULIP, BAND_REX,  PHASE_PD3,    SIM_SINGLE},
  {0x43, PROJECT_TULIP, BAND_REX,  PHASE_DP,     SIM_SINGLE},
  {0x44, PROJECT_TULIP, BAND_REX,  PHASE_Pre_SP, SIM_SINGLE},
  {0x45, PROJECT_TULIP, BAND_REX,  PHASE_SP1_0,  SIM_SINGLE},
  {0x46, PROJECT_TULIP, BAND_REX,  PHASE_AP,     SIM_SINGLE},
  {0x48, PROJECT_TULIP, BAND_REX,  PHASE_TP,     SIM_SINGLE},
  {0x49, PROJECT_TULIP, BAND_REX,  PHASE_PQ,     SIM_SINGLE},
  {0x4a, PROJECT_TULIP, BAND_REX,  PHASE_MP,     SIM_SINGLE},
  //================
  {0x50, PROJECT_TULIP, BAND_REX,  PHASE_EVM2,   SIM_DUAL  },
  //======APAC======
  {0x60, PROJECT_TULIP, BAND_APAC, PHASE_PD1,    SIM_SINGLE},
  {0x61, PROJECT_TULIP, BAND_APAC, PHASE_PD2,    SIM_SINGLE},
  {0x62, PROJECT_TULIP, BAND_APAC, PHASE_PD3,    SIM_SINGLE},
  {0x63, PROJECT_TULIP, BAND_APAC, PHASE_DP,     SIM_SINGLE},
  {0x64, PROJECT_TULIP, BAND_APAC, PHASE_Pre_SP, SIM_SINGLE},
  {0x65, PROJECT_TULIP, BAND_APAC, PHASE_SP1_0,  SIM_SINGLE},
  {0x66, PROJECT_TULIP, BAND_APAC, PHASE_AP,     SIM_SINGLE},
  {0x68, PROJECT_TULIP, BAND_APAC, PHASE_TP,     SIM_SINGLE},
  {0x69, PROJECT_TULIP, BAND_APAC, PHASE_PQ,     SIM_SINGLE},
  {0x6a, PROJECT_TULIP, BAND_APAC, PHASE_MP,     SIM_SINGLE},
  //======VIV=======
  {0x70, PROJECT_TULIP, BAND_VIV,  PHASE_PD1,    SIM_SINGLE},
  {0x71, PROJECT_TULIP, BAND_VIV,  PHASE_PD2,    SIM_SINGLE},
  {0x72, PROJECT_TULIP, BAND_VIV,  PHASE_PD3,    SIM_SINGLE},
  {0x73, PROJECT_TULIP, BAND_VIV,  PHASE_DP,     SIM_SINGLE},
  {0x74, PROJECT_TULIP, BAND_VIV,  PHASE_Pre_SP, SIM_SINGLE},
  {0x75, PROJECT_TULIP, BAND_VIV,  PHASE_SP1_0,  SIM_SINGLE},
  {0x76, PROJECT_TULIP, BAND_VIV,  PHASE_AP,     SIM_SINGLE},
  {0x78, PROJECT_TULIP, BAND_VIV,  PHASE_TP,     SIM_SINGLE},
  {0x79, PROJECT_TULIP, BAND_VIV,  PHASE_PQ,     SIM_SINGLE},
  {0x7a, PROJECT_TULIP, BAND_VIV,  PHASE_MP,     SIM_SINGLE},
  //================
  {0xff, PROJECT_MAX,   BAND_MAX,  PHASE_MAX,    SIM_MAX}
};

static fih_hwid_map HWID_MAP_8936[]=
{
  /* Tulip */
  //======Rita======
  {0x00, PROJECT_TULIP, BAND_RITA, PHASE_PD4,      SIM_SINGLE},
  {0x01, PROJECT_TULIP, BAND_RITA, PHASE_DP,       SIM_SINGLE},
  {0x02, PROJECT_TULIP, BAND_RITA, PHASE_Pre_SP,   SIM_SINGLE},
  {0x03, PROJECT_TULIP, BAND_RITA, PHASE_SP1_0,    SIM_SINGLE},
  {0x04, PROJECT_TULIP, BAND_RITA, PHASE_SP1_2,    SIM_SINGLE},
  {0x05, PROJECT_TULIP, BAND_RITA, PHASE_Pre_AP,   SIM_SINGLE},
  {0x06, PROJECT_TULIP, BAND_RITA, PHASE_AP,       SIM_SINGLE},
  {0x07, PROJECT_TULIP, BAND_RITA, PHASE_TP,       SIM_SINGLE},
  {0x08, PROJECT_TULIP, BAND_RITA, PHASE_PQ,       SIM_SINGLE},
  {0x09, PROJECT_TULIP, BAND_RITA, PHASE_MP,       SIM_SINGLE},
  {0x0a, PROJECT_TULIP, BAND_RITA, PHASE_SP1_0_v2, SIM_SINGLE}, //WTR4905 V2
  {0x0b, PROJECT_TULIP, BAND_RITA, PHASE_SP1_2_v2, SIM_SINGLE}, //WTR4905 V2
  //===============
  {0x10, PROJECT_TULIP, BAND_RITA, PHASE_PD4,      SIM_DUAL  },
  {0x11, PROJECT_TULIP, BAND_RITA, PHASE_DP,       SIM_DUAL  },
  {0x12, PROJECT_TULIP, BAND_RITA, PHASE_Pre_SP,   SIM_DUAL  },
  {0x13, PROJECT_TULIP, BAND_RITA, PHASE_SP1_0,    SIM_DUAL  },
  {0x14, PROJECT_TULIP, BAND_RITA, PHASE_SP1_2,    SIM_DUAL  },
  {0x15, PROJECT_TULIP, BAND_RITA, PHASE_Pre_AP,   SIM_DUAL  },
  {0x16, PROJECT_TULIP, BAND_RITA, PHASE_AP,       SIM_DUAL  },
  {0x17, PROJECT_TULIP, BAND_RITA, PHASE_TP_PI,    SIM_DUAL  },
  {0x18, PROJECT_TULIP, BAND_RITA, PHASE_TP_ZDT,   SIM_DUAL  },
  {0x19, PROJECT_TULIP, BAND_RITA, PHASE_PQ,       SIM_DUAL  },
  {0x1a, PROJECT_TULIP, BAND_RITA, PHASE_MP,       SIM_DUAL  },
  {0x1b, PROJECT_TULIP, BAND_RITA, PHASE_SP1_0_v2, SIM_DUAL  }, //WTR4905 V2
  {0x1c, PROJECT_TULIP, BAND_RITA, PHASE_SP1_2_v2, SIM_DUAL  }, //WTR4905 V2
  {0x1d, PROJECT_TULIP, BAND_RITA, PHASE_PQ2,      SIM_DUAL  },
  //======Gina======
  {0x20, PROJECT_TULIP, BAND_GINA, PHASE_PD4,      SIM_SINGLE},
  {0x21, PROJECT_TULIP, BAND_GINA, PHASE_DP,       SIM_SINGLE},
  {0x22, PROJECT_TULIP, BAND_GINA, PHASE_Pre_SP,   SIM_SINGLE},
  {0x23, PROJECT_TULIP, BAND_GINA, PHASE_SP1_0,    SIM_SINGLE},
  {0x24, PROJECT_TULIP, BAND_GINA, PHASE_SP1_2,    SIM_SINGLE},
  {0x25, PROJECT_TULIP, BAND_GINA, PHASE_Pre_AP,   SIM_SINGLE},
  {0x26, PROJECT_TULIP, BAND_GINA, PHASE_AP,       SIM_SINGLE},
  {0x27, PROJECT_TULIP, BAND_GINA, PHASE_TP_PI,    SIM_SINGLE},
  {0x28, PROJECT_TULIP, BAND_GINA, PHASE_TP_ZDT,   SIM_SINGLE},
  {0x29, PROJECT_TULIP, BAND_GINA, PHASE_PQ,       SIM_SINGLE},
  {0xa9, PROJECT_TULIP, BAND_GINA, PHASE_PQ_16G,   SIM_SINGLE},
  {0x2a, PROJECT_TULIP, BAND_GINA, PHASE_MP,       SIM_SINGLE},
  {0xaa, PROJECT_TULIP, BAND_GINA, PHASE_MP_16G,   SIM_SINGLE},
  {0x2b, PROJECT_TULIP, BAND_GINA, PHASE_SP1_0_v2, SIM_SINGLE}, //WTR4905 V2
  {0x2c, PROJECT_TULIP, BAND_GINA, PHASE_SP1_2_v2, SIM_SINGLE}, //WTR4905 V2
  {0x2d, PROJECT_TULIP, BAND_GINA, PHASE_Pre_AP39, SIM_SINGLE}, //CPU8939
  {0x2e, PROJECT_TULIP, BAND_GINA, PHASE_PQ2,      SIM_SINGLE},
  //===============
  {0x30, PROJECT_TULIP, BAND_GINA, PHASE_PD4,      SIM_DUAL  },
  {0x31, PROJECT_TULIP, BAND_GINA, PHASE_DP,       SIM_DUAL  },
  {0x32, PROJECT_TULIP, BAND_GINA, PHASE_Pre_SP,   SIM_DUAL  },
  {0x33, PROJECT_TULIP, BAND_GINA, PHASE_SP1_0,    SIM_DUAL  },
  {0x34, PROJECT_TULIP, BAND_GINA, PHASE_SP1_2,    SIM_DUAL  },
  {0x35, PROJECT_TULIP, BAND_GINA, PHASE_Pre_AP,   SIM_DUAL  },
  {0x36, PROJECT_TULIP, BAND_GINA, PHASE_AP,       SIM_DUAL  },
  {0x37, PROJECT_TULIP, BAND_GINA, PHASE_TP_PI,    SIM_DUAL  },
  {0x38, PROJECT_TULIP, BAND_GINA, PHASE_TP_ZDT,   SIM_DUAL  },
  {0x39, PROJECT_TULIP, BAND_GINA, PHASE_PQ,       SIM_DUAL  },
  {0x3a, PROJECT_TULIP, BAND_GINA, PHASE_MP,       SIM_DUAL  },
  {0x3b, PROJECT_TULIP, BAND_GINA, PHASE_SP1_0_v2, SIM_DUAL  }, //WTR4905 V2
  {0x3c, PROJECT_TULIP, BAND_GINA, PHASE_SP1_2_v2, SIM_DUAL  }, //WTR4905 V2
  {0x3d, PROJECT_TULIP, BAND_GINA, PHASE_PQ2,      SIM_DUAL  },
  //======Rex=======
  {0x40, PROJECT_TULIP, BAND_REX,  PHASE_PD4,      SIM_SINGLE},
  {0x41, PROJECT_TULIP, BAND_REX,  PHASE_DP,       SIM_SINGLE},
  {0x42, PROJECT_TULIP, BAND_REX,  PHASE_Pre_SP,   SIM_SINGLE},
  {0x43, PROJECT_TULIP, BAND_REX,  PHASE_SP1_0,    SIM_SINGLE},
  {0x44, PROJECT_TULIP, BAND_REX,  PHASE_SP1_2,    SIM_SINGLE},
  {0x45, PROJECT_TULIP, BAND_REX,  PHASE_Pre_AP,   SIM_SINGLE},
  {0x46, PROJECT_TULIP, BAND_REX,  PHASE_AP,       SIM_SINGLE},
  {0x47, PROJECT_TULIP, BAND_REX,  PHASE_TP_PI,    SIM_SINGLE},
  {0x48, PROJECT_TULIP, BAND_REX,  PHASE_TP_ZDT,   SIM_SINGLE},
  {0x49, PROJECT_TULIP, BAND_REX,  PHASE_PQ,       SIM_SINGLE},
  {0x4a, PROJECT_TULIP, BAND_REX,  PHASE_MP,       SIM_SINGLE},
  {0x4b, PROJECT_TULIP, BAND_REX,  PHASE_SP1_0_v2, SIM_SINGLE}, //WTR4905 V2
  {0x4c, PROJECT_TULIP, BAND_REX,  PHASE_SP1_2_v2, SIM_SINGLE}, //WTR4905 V2
  {0x4d, PROJECT_TULIP, BAND_REX,  PHASE_PQ2,      SIM_SINGLE},
  //======APAC======
  {0x51, PROJECT_TULIP, BAND_REX,  PHASE_EVM3,     SIM_DUAL  },
  {0x52, PROJECT_TULIP, BAND_APAC, PHASE_PD4,      SIM_SINGLE},
  {0x53, PROJECT_TULIP, BAND_APAC, PHASE_DP,       SIM_SINGLE},
  {0x54, PROJECT_TULIP, BAND_APAC, PHASE_Pre_SP,   SIM_SINGLE},
  {0x55, PROJECT_TULIP, BAND_APAC, PHASE_SP1_0,    SIM_SINGLE},
  {0x56, PROJECT_TULIP, BAND_APAC, PHASE_SP1_2,    SIM_SINGLE},
  {0x57, PROJECT_TULIP, BAND_APAC, PHASE_TP_PI,    SIM_SINGLE},
  {0x58, PROJECT_TULIP, BAND_APAC, PHASE_Pre_AP,   SIM_SINGLE},
  {0x59, PROJECT_TULIP, BAND_APAC, PHASE_AP,       SIM_SINGLE},
  {0x5a, PROJECT_TULIP, BAND_APAC, PHASE_TP_ZDT,   SIM_SINGLE},
  {0x5b, PROJECT_TULIP, BAND_APAC, PHASE_PQ,       SIM_SINGLE},
  {0x5c, PROJECT_TULIP, BAND_APAC, PHASE_MP,       SIM_SINGLE},
  {0x5d, PROJECT_TULIP, BAND_APAC, PHASE_SP1_0_v2, SIM_SINGLE}, //WTR4905 V2
  {0x5e, PROJECT_TULIP, BAND_APAC, PHASE_SP1_2_v2, SIM_SINGLE}, //WTR4905 V2
  {0x5f, PROJECT_TULIP, BAND_APAC, PHASE_PQ2,      SIM_SINGLE},
  //================
  {0x60, PROJECT_TULIP, BAND_APAC, PHASE_PD4,      SIM_DUAL  },
  {0x61, PROJECT_TULIP, BAND_APAC, PHASE_DP,       SIM_DUAL  },
  {0x62, PROJECT_TULIP, BAND_APAC, PHASE_Pre_SP,   SIM_DUAL  },
  {0x63, PROJECT_TULIP, BAND_APAC, PHASE_SP1_0,    SIM_DUAL  },
  {0x64, PROJECT_TULIP, BAND_APAC, PHASE_SP1_2,    SIM_DUAL  },
  {0x65, PROJECT_TULIP, BAND_APAC, PHASE_Pre_AP,   SIM_DUAL  },
  {0x66, PROJECT_TULIP, BAND_APAC, PHASE_AP,       SIM_DUAL  },
  {0x67, PROJECT_TULIP, BAND_APAC, PHASE_TP_PI,    SIM_DUAL  },
  {0x68, PROJECT_TULIP, BAND_APAC, PHASE_TP_ZDT,   SIM_DUAL  },
  {0x69, PROJECT_TULIP, BAND_APAC, PHASE_PQ,       SIM_DUAL  },
  {0x6a, PROJECT_TULIP, BAND_APAC, PHASE_MP,       SIM_DUAL  },
  {0x6b, PROJECT_TULIP, BAND_APAC, PHASE_SP1_0_v2, SIM_DUAL  }, //WTR4905 V2
  {0x6c, PROJECT_TULIP, BAND_APAC, PHASE_SP1_2_v2, SIM_DUAL  }, //WTR4905 V2
  {0x6d, PROJECT_TULIP, BAND_APAC, PHASE_PQ2,      SIM_DUAL  },
  //======VIV=======
  {0x72, PROJECT_TULIP, BAND_VIV,  PHASE_PD3,      SIM_SINGLE},
  {0x73, PROJECT_TULIP, BAND_VIV,  PHASE_DP,       SIM_SINGLE},
  {0x74, PROJECT_TULIP, BAND_VIV,  PHASE_Pre_SP,   SIM_SINGLE},
  {0x75, PROJECT_TULIP, BAND_VIV,  PHASE_SP1_0,    SIM_SINGLE},
  {0x76, PROJECT_TULIP, BAND_VIV,  PHASE_SP1_2,    SIM_SINGLE},
  {0x78, PROJECT_TULIP, BAND_VIV,  PHASE_SP39_1_2, SIM_SINGLE},  //CPU8939
  {0x79, PROJECT_TULIP, BAND_VIV,  PHASE_AP,       SIM_SINGLE},
  {0x7a, PROJECT_TULIP, BAND_VIV,  PHASE_TP,       SIM_SINGLE},
  {0x7b, PROJECT_TULIP, BAND_VIV,  PHASE_PQ,       SIM_SINGLE},
  {0x7c, PROJECT_TULIP, BAND_VIV,  PHASE_MP,       SIM_SINGLE},
  //================
  {0xff, PROJECT_MAX,   BAND_MAX,  PHASE_MAX,    SIM_MAX}
};

static unsigned int str_idx[4]={0};

unsigned int FIH_S1_BOOT = 0;
fih_product_id_type FIH_PROJECT_ID = PROJECT_TULIP;
fih_product_phase_type FIH_PHASE_ID = PHASE_EVM;
fih_band_id_type FIH_BAND_ID = BAND_RITA;
fih_sim_type FIH_SIM_ID = SIM_SINGLE;

unsigned int fih_get_s1_boot(void)
{
  return FIH_S1_BOOT;
}
EXPORT_SYMBOL(fih_get_s1_boot);

unsigned int fih_get_product_id(void)
{
  return FIH_PROJECT_ID;
}
EXPORT_SYMBOL(fih_get_product_id);

unsigned int fih_get_product_phase(void)
{
  return FIH_PHASE_ID;
}
EXPORT_SYMBOL(fih_get_product_phase);

unsigned int fih_get_band_id(void)
{
  return FIH_BAND_ID;
}
EXPORT_SYMBOL(fih_get_band_id);

unsigned int fih_get_sim_id(void)
{
  return FIH_SIM_ID;
}
EXPORT_SYMBOL(fih_get_sim_id);

int fih_phase_match(fih_product_phase_type start,fih_product_phase_type end)
{
  if( (FIH_PHASE_ID >= start)&&(FIH_PHASE_ID < end) )
  {
    return 1;
  }
  else if( (start == end)&&(FIH_PHASE_ID == start) )
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
EXPORT_SYMBOL(fih_phase_match);

//BSP-REXER-DTS-00+[
int str2id_phase(const char* phase)
{
  int i=0;

  for(i=0; phase_id_map[i].ID != PHASE_MAX; i++)
  {
    if( !strcmp(phase_id_map[i].STR,phase) )
    {
      return phase_id_map[i].ID;
    }
  }
  return -1;
}

int fih_dts_phase_match(const char* start, const char* end)
{
  int start_id,end_id;

  start_id = str2id_phase(start);
  end_id = str2id_phase(end);

  if( (start_id==-1)||(end_id==-1) )
  {
    //dts no phase setting, assume match
    return 1;
  }
  else if( (FIH_PHASE_ID >= start_id)&&(FIH_PHASE_ID < end_id) )
  {
    return 1;
  }
  else if( (start_id == end_id)&&(FIH_PHASE_ID == start_id) )
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
EXPORT_SYMBOL(fih_dts_phase_match);


void fih_dts_phase_cfg(fih_product_phase_type start,fih_product_phase_type end,char* str)
{
  int i;

  if( (FIH_PHASE_ID >= start)&&(FIH_PHASE_ID < end) )
  {
    for(i=0; phase_id_map[i].ID != PHASE_MAX; i++)
    {
      if(phase_id_map[i].ID == start)
      {
        strcat(str,"-");
        strcat(str,phase_id_map[i].STR);
        strcat(str,"\0");
        break;
      }
    }
  }
}
EXPORT_SYMBOL(fih_dts_phase_cfg);
//BSP-REXER-DTS-00+]

void id2str_map(void)
{
  int i;

  for(i=0; project_id_map[i].ID != PROJECT_MAX; i++)
  {
    if(project_id_map[i].ID == FIH_PROJECT_ID)
    {
      str_idx[0]=i;
      break;
    }
  }

  for(i=0; band_id_map[i].ID != BAND_MAX; i++)
  {
    if(band_id_map[i].ID == FIH_BAND_ID)
    {
      str_idx[1]=i;
      break;
    }
  }

  for(i=0; phase_id_map[i].ID != PHASE_MAX; i++)
  {
    if(phase_id_map[i].ID == FIH_PHASE_ID)
    {
      str_idx[2]=i;
      break;
    }
  }

  for(i=0; sim_id_map[i].ID != SIM_MAX; i++)
  {
    if(sim_id_map[i].ID == FIH_SIM_ID)
    {
      str_idx[3]=i;
      break;
    }
  }
}

void hwid_info(void)
{
  id2str_map();

  pr_info("FIH_PROJECT: %s\r\n",project_id_map[str_idx[0]].STR);
  pr_info("FIH_BAND:    %s\r\n",band_id_map[str_idx[1]].STR);
  pr_info("FIH_PHASE:   %s\r\n",phase_id_map[str_idx[2]].STR);
  pr_info("FIH_SIM:     %s\r\n",sim_id_map[str_idx[3]].STR);
}

unsigned int hwid_map(void)
{
  int i, hwid_logic=0;
  int rc=-1;

  for(i=0; i<ARRAY_SIZE(HWID_GPIO); i++)
  {
    hwid_logic +=( ( !!__gpio_get_value(HWID_GPIO[i].gpio) ) << i );
  }

  pr_info("HWID_LOGIC 0x%x\r\n",hwid_logic);

  if(of_machine_is_compatible("qcom,msm8916"))
  {
    HWID_MAP=HWID_MAP_8916;
    pr_info("CPU 8916 HWID MAP\r\n");
  }
  else if (of_machine_is_compatible("qcom,msm8936"))
  {
    HWID_MAP=HWID_MAP_8936;
    pr_info("CPU 8936 HWID MAP\r\n");
  }
  else if (of_machine_is_compatible("qcom,msm8939"))
  {
    HWID_MAP=HWID_MAP_8936;  //use 8936 HWID MAP replace
    pr_info("CPU 8939 HWID MAP\r\n");
  }
  else
  {
    HWID_MAP=HWID_MAP_8936;
    pr_info("use default CPU 8936 HWID MAP\r\n");
  }

  for(i=0; HWID_MAP[i].HWID_LOGIC!=0xff; i++)
  {
    if(hwid_logic==HWID_MAP[i].HWID_LOGIC)
    {
      FIH_PROJECT_ID = HWID_MAP[i].PROJECT;
      FIH_BAND_ID    = HWID_MAP[i].BAND;
      FIH_PHASE_ID   = HWID_MAP[i].PHASE;
      FIH_SIM_ID     = HWID_MAP[i].SIM;
      rc=0;
      break;
    }
  }

  return rc;
}

unsigned int env_init(void)
{
  int rc=0;

  rc=gpio_request_array(HWID_GPIO, ARRAY_SIZE(HWID_GPIO));

  return rc;
}

void fih_hwid_get(void)
{
  pr_info("======FIH HWID init start======\r\n");

  if (fih_get_s1_boot())
  {
    pr_info("This is s1 boot\r\n");
  }
  else
  {
    pr_info("This is qcom boot\r\n");
  }

  if(env_init())
  {
    pr_info("FIH HWID init fail\r\n");
    pr_info("================================\r\n");
  }

  if(hwid_map())
  {
    pr_info("******map fail use default HWID******\r\n");
  }

  hwid_info();

  pr_info("================================\r\n");

}


static int fih_hwid_probe(struct platform_device *pdev)
{

  struct device *dev = &pdev->dev;
  struct pinctrl *hwid_pinctrl;
  struct pinctrl_state *init_state;
  int i,rc=0;

  hwid_pinctrl=devm_pinctrl_get(dev);
  rc = IS_ERR(hwid_pinctrl);
  if(rc)
  {
    if (PTR_ERR(hwid_pinctrl) == -EPROBE_DEFER)
    {
      rc = -EPROBE_DEFER;
    }
    pr_debug("FIH HWID does not use pinctrl\n");
    hwid_pinctrl = NULL;
    goto fail;
  }

  if (hwid_pinctrl)
  {
    init_state=pinctrl_lookup_state(hwid_pinctrl,"init");
    if(IS_ERR(init_state))
    {
      pr_debug("FIH HWID pinctrl can't get init state\n");
      rc = PTR_ERR(init_state);
      goto fail;
    }
  }

  rc = pinctrl_select_state(hwid_pinctrl, init_state);

  if(rc)
  {
    pr_debug("FIH HWID pinctrl can't set init state\n");
    goto fail;
  }

  for (i=0; i<ARRAY_SIZE(HWID_GPIO); i++)
  {
    HWID_GPIO[i].gpio=of_get_named_gpio(dev->of_node, "gpios", i);
  }

  fih_hwid_get();

fail:
  return rc;
};

static struct of_device_id fih_hwid_match_table[]=
{
  {.compatible = "fih,hwid"},
  {}
};

static struct platform_driver fih_hwid_driver=
{
  .probe = fih_hwid_probe,
  .driver =
  {
    .name = "fih_hwid",
    .of_match_table = fih_hwid_match_table,
    .owner = THIS_MODULE,
  },
};

static int __init fih_hwid_init(void)
{
  return platform_driver_register(&fih_hwid_driver);
}
postcore_initcall_sync(fih_hwid_init);

static int get_s1_boot(char *str)
{
  if (!strcmp(str,"s1"))
  {
    FIH_S1_BOOT=1;
  }
  return 0;
}
__setup("androidboot.bootloader=", get_s1_boot);


