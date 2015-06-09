//Copyright(C) 2014-2015 Foxconn International Holdings, Ltd. All rights reserved
/*
  The purpose is that handle the no used GPIO pin,
  to avoid extra power consumption.
  Notice that the method here is that write the register directly.
  It is not correspond to Qcom's method.
*/

#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/fih_gpio_init.h>

static void __iomem *addr;
static int fih_gpio_env = FIH_DISABLE;
FIH_GPIO_TABLE main_table[FIH_GPIO_NUM];

static FIH_GPIO_TABLE FAKE_TEMP[]=
{
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

//=======================================================
static FIH_GPIO_TABLE TUL_ALL_PD1[]= //Not include RITA
{
  { 74, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 75, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 76, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 77, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 78, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 79, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 80, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 81, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 83, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 86, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 88, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 89, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 92, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 94, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {100, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {101, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {102, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {105, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {106, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {112, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {115, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {117, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {118, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {119, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

static FIH_GPIO_TABLE TUL_ALL_PD3[]= //Not include VIV
{
  { 90, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

static FIH_GPIO_TABLE TUL_ALL_DP[]= //Not include VIV
{
  { 16, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

//=======================================================
static FIH_GPIO_TABLE TUL_RITA_EVM[]=
{
  { 32, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 35, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 72, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 73, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 89, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 91, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {101, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {102, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {105, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {106, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {107, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

static FIH_GPIO_TABLE TUL_RITA_PD1[]=
{
  { 32, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  { 35, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  { 72, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  { 73, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  { 74, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 75, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 76, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 77, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 78, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 79, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 80, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 81, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 83, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 86, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 88, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 89, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 91, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  { 92, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 94, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {100, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {107, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  {112, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {115, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {117, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {118, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {119, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

/*
  the numbers stand for "NA" GPIO settings after patching
  example //32,35,72,73,89,91,101,102,105,106,107
*/

static FIH_GPIO_MAP TUL_RITA_MAP[]=
{
  {PHASE_EVM, SIM_SINGLE, TUL_RITA_EVM}, //32,35,72,73,89,91,101,102,105,106,107
  {PHASE_PD1, SIM_SINGLE, TUL_RITA_PD1}, //74,75,76,77,78,79,80,81,83,86,88,89,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_PD2, SIM_SINGLE, FAKE_TEMP},    //same as TUL_RITA_PD1
  {PHASE_PD3, SIM_SINGLE, TUL_ALL_PD3},  //74,75,76,77,78,79,80,81,83,86,88,89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_DP,  SIM_SINGLE, TUL_ALL_DP},   //16,74,75,76,77,78,79,80,81,83,86,88 89,90,92,94,100,101,102,105,106,112,115,117,118,119
  //------------------------------------------------------------
  {PHASE_PD1, SIM_DUAL,   TUL_ALL_PD1},  //74,75,76,77,78,79,80,81,83,86,88,89,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_PD2, SIM_DUAL,   FAKE_TEMP},    //same as TUL_ALL_PD1
  {PHASE_PD3, SIM_DUAL,   TUL_ALL_PD3},  //74,75,76,77,78,79,80,81,83,86,88,89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_DP,  SIM_DUAL,   TUL_ALL_DP},   //16,74,75,76,77,78,79,80,81,83,86,88 89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_MAX, SIM_MAX,    FAKE_TEMP}
};
//=======================================================
static FIH_GPIO_MAP TUL_GINA_MAP[]=
{
  {PHASE_PD1, SIM_SINGLE, TUL_ALL_PD1},  //74,75,76,77,78,79,80,81,83,86,88,89,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_PD2, SIM_SINGLE, FAKE_TEMP},    //same as TUL_ALL_PD1
  {PHASE_PD3, SIM_SINGLE, TUL_ALL_PD3},  //74,75,76,77,78,79,80,81,83,86,88,89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_DP,  SIM_SINGLE, TUL_ALL_DP},   //16,74,75,76,77,78,79,80,81,83,86,88 89,90,92,94,100,101,102,105,106,112,115,117,118,119
  //------------------------------------------------------------
  {PHASE_PD1, SIM_DUAL,   TUL_ALL_PD1},  //74,75,76,77,78,79,80,81,83,86,88,89,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_PD2, SIM_DUAL,   FAKE_TEMP},    //same as TUL_ALL_PD1
  {PHASE_PD3, SIM_DUAL,   TUL_ALL_PD3},  //74,75,76,77,78,79,80,81,83,86,88,89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_DP,  SIM_DUAL,   TUL_ALL_DP},   //16,74,75,76,77,78,79,80,81,83,86,88 89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_MAX, SIM_MAX,    FAKE_TEMP}
};
//=======================================================
static FIH_GPIO_TABLE TUL_REX_EVM2[]=
{
  { 74, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 75, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 76, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 77, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 78, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 79, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 80, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 81, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 83, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 86, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 88, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 89, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 90, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 92, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { 94, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {100, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {101, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {102, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {105, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {106, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

static FIH_GPIO_TABLE TUL_REX_PD3[]=
{
  {112, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {115, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {117, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {118, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {119, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

static FIH_GPIO_MAP TUL_REX_MAP[]=
{
  {PHASE_PD1,  SIM_SINGLE, TUL_ALL_PD1},  //74,75,76,77,78,79,80,81,83,86,88,89,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_PD2,  SIM_SINGLE, FAKE_TEMP},    //same as TUL_ALL_PD1
  {PHASE_PD3,  SIM_SINGLE, TUL_ALL_PD3},  //74,75,76,77,78,79,80,81,83,86,88,89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_DP,   SIM_SINGLE, TUL_ALL_DP},   //16,74,75,76,77,78,79,80,81,83,86,88 89,90,92,94,100,101,102,105,106,112,115,117,118,119
  //------------------------------------------------------------
  {PHASE_EVM2, SIM_DUAL,   TUL_REX_EVM2}, //74,75,76,77,78,79,80,81,83,86,88,89,90,92,94,100,101,102,105,106
  {PHASE_PD3,  SIM_DUAL,   TUL_REX_PD3},  //74,75,76,77,78,79,80,81,83,86,88,89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_DP,   SIM_DUAL,   TUL_ALL_DP},   //16,74,75,76,77,78,79,80,81,83,86,88 89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_MAX,  SIM_MAX,    FAKE_TEMP}
};
//=======================================================
static FIH_GPIO_MAP TUL_APAC_MAP[]=
{
  {PHASE_PD1, SIM_SINGLE, TUL_ALL_PD1},  //74,75,76,77,78,79,80,81,83,86,88,89,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_PD2, SIM_SINGLE, FAKE_TEMP},    //same as TUL_ALL_PD1
  {PHASE_PD3, SIM_SINGLE, TUL_ALL_PD3},  //74,75,76,77,78,79,80,81,83,86,88,89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_DP,  SIM_SINGLE, TUL_ALL_DP},   //16,74,75,76,77,78,79,80,81,83,86,88 89,90,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_MAX, SIM_MAX, FAKE_TEMP}
};
//=======================================================
static FIH_GPIO_TABLE TUL_VIV_PD3[]=
{
  { 49, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {112, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  {115, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  {117, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  {118, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  {119, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_DISABLE},
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

static FIH_GPIO_TABLE TUL_VIV_DP[]=
{
  {116, FIH_CFG(0, FIH_INPUT, FIH_PULL_DOWN, FIH_2MA), FIH_ENABLE},
  { -1, FIH_CFG(0, 0, 0, 0), -1}
};

static FIH_GPIO_MAP TUL_VIV_MAP[]=
{
  {PHASE_PD1, SIM_SINGLE, TUL_ALL_PD1},  //74,75,76,77,78,79,80,81,83,86,88,89,92,94,100,101,102,105,106,112,115,117,118,119
  {PHASE_PD2, SIM_SINGLE, FAKE_TEMP},    //same as TUL_ALL_PD1
  {PHASE_PD3, SIM_SINGLE, TUL_VIV_PD3},  //49,74,75,76,77,78,79,80,81,83,86,88,89,92,94,100,101,102,105,106
  {PHASE_DP,  SIM_SINGLE, TUL_VIV_DP},   //49,74,75,76,77,78,79,80,81,83,86,88,89,92,94,100,101,102,105,106,116
  {PHASE_MAX, SIM_MAX, FAKE_TEMP}
};
//=======================================================

void fih_gp_out_set(unsigned int pin, int level)
{
  void __iomem *inout_reg = FIH_GP_INOUT(addr, pin);

  if(fih_gpio_env==FIH_DISABLE)
  {
    pr_info("fih_gpio_env_init is not done\r\n");
  }

  if( (pin>=0)&&(pin<FIH_GPIO_NUM) )
  {
    writel_relaxed(level ? BIT(FIH_GP_OUT_BIT) : 0, inout_reg);
  }
  else
  {
    pr_info("FIH gpio pin %d is invalid\r\n", pin);
  }
}

int fih_gp_in_get(unsigned int pin)
{
  int rc=-1;
  void __iomem *inout_reg = FIH_GP_INOUT(addr, pin);

  if(fih_gpio_env==FIH_DISABLE)
  {
    pr_info("fih_gpio_env_init is not done\r\n");
    return rc;
  }

  if( (pin>=0)&&(pin<FIH_GPIO_NUM) )
  {
    rc=readl_relaxed(inout_reg)& BIT(FIH_GP_IN_BIT);
  }
  else
  {
    pr_info("FIH gpio pin %d is invalid\r\n", pin);
    return rc;
  }

  return rc;

}

void fih_gp_cfg(unsigned int pin, unsigned int config)
{
  void __iomem *cfg_reg = FIH_GP_CFG(addr, pin);

  if(fih_gpio_env==FIH_DISABLE)
  {
    pr_info("fih_gpio_env_init is not done\r\n");
  }

  if( (pin>=0)&&(pin<FIH_GPIO_NUM) )
  {
    writel_relaxed(config, cfg_reg);
  }
  else
  {
    pr_info("FIH gpio pin %d is invalid\r\n", pin);
  }
}

static FIH_GPIO_MAP* fih_select_map(void)
{
  switch( fih_get_band_id() )
  {
    case BAND_RITA:
      return TUL_RITA_MAP;

    case BAND_GINA:
      return TUL_GINA_MAP;

    case BAND_REX:
      return TUL_REX_MAP;

    case BAND_APAC:
      return TUL_APAC_MAP;

    case BAND_VIV:
      return TUL_VIV_MAP;

    default:
      pr_info("FIH use default gpio map\r\n");
      return TUL_RITA_MAP;
  }
}

static void fih_init_gpio_table(FIH_GPIO_MAP* map)
{
  int i, j, pin;
  fih_product_phase_type phase;
  fih_sim_type sim;

  phase=fih_get_product_phase();
  sim=fih_get_sim_id();

  for(i=0; map[i].phase!=PHASE_MAX; i++ )
  {
    if( (sim==map[i].sim)&&(phase>=map[i].phase) )
    {
      for(j=0; map[i].table[j].pin!=-1; j++)
      {
        pin=map[i].table[j].pin;
        if( (pin>=0)&&(pin<FIH_GPIO_NUM) )
        {
          main_table[pin].pin   =pin;
          main_table[pin].config=map[i].table[j].config;
          main_table[pin].enable=map[i].table[j].enable;
        }
      }
    }
  }
}

static void fih_config_gpio_table(void)
{
  int i;

  for(i=0; i<122; i++)
  {
    if(main_table[i].enable==FIH_ENABLE)
    {
      fih_gp_cfg(main_table[i].pin, main_table[i].config);
    }
  }
};

static void fih_gpio_env_init(void)
{
  addr = ioremap((__force phys_addr_t)0x1000000, 0x300000);
  memset(main_table, 0, 122*sizeof(FIH_GPIO_TABLE));
  fih_gpio_env=FIH_ENABLE;
}

static int fih_gpio_probe(struct platform_device *pdev)
{
  int rc=0;
  FIH_GPIO_MAP* temp_map;

  pr_info("======FIH GPIO init start======\r\n");
  fih_gpio_env_init();
  temp_map=fih_select_map();
  fih_init_gpio_table(temp_map);
  fih_config_gpio_table();
  pr_info("=======FIH GPIO init end=======\r\n");

  return rc;
};

static struct of_device_id fih_gpio_match_table[]=
{
  {.compatible = "fih,gpio"},
  {}
};

static struct platform_driver fih_gpio_driver=
{
  .probe = fih_gpio_probe,
  .driver =
  {
    .name = "fih_gpio",
    .of_match_table = fih_gpio_match_table,
    .owner = THIS_MODULE,
  },
};

static int __init fih_gpio_init(void)
{
  return platform_driver_register(&fih_gpio_driver);
}
arch_initcall(fih_gpio_init);

