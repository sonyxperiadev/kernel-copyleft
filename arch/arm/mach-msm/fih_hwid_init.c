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

#include <linux/qpnp/pin.h>
#include <linux/qpnp/qpnp-adc.h>

typedef struct
{
  int PROJECT_LV; //GPIO_3 for PMIC
  int BAND_LV;    //GPIO_6 for PMIC
  int PHASE_LV;   //GPIO_5 for PMIC
  fih_product_id_type PROJECT;
  fih_band_id_type BAND;
  fih_product_phase_type PHASE;
}fih_hwid_map;

typedef struct
{
  int LEVEL;
  int MIN; //uV
  int MAX; //uV
}fih_adc_map;

static struct qpnp_pin_cfg high =
{
  .mode = QPNP_PIN_MODE_DIG_IN,
  .pull = QPNP_PIN_GPIO_PULL_NO,
  .out_strength = QPNP_PIN_OUT_STRENGTH_HIGH,
  .src_sel = QPNP_PIN_SEL_FUNC_CONSTANT,
  .master_en = QPNP_PIN_MASTER_DISABLE,
};

static struct qpnp_pin_cfg low =
{
  .mode = QPNP_PIN_MODE_DIG_OUT,
  .output_type = QPNP_PIN_OUT_BUF_OPEN_DRAIN_NMOS,
  .invert = QPNP_PIN_INVERT_DISABLE,
  .pull = QPNP_PIN_GPIO_PULL_NO,
  .out_strength = QPNP_PIN_OUT_STRENGTH_HIGH,
  .src_sel = QPNP_PIN_SEL_FUNC_CONSTANT,
  .master_en = QPNP_PIN_MASTER_ENABLE,
};

//============================

static const fih_hwid_map HWID_MAP[]=
{
  /* Seagull */
  //======Rita======
  {0, 1, 0, PROJECT_SEAGULL, BAND_125, PHASE_EVM},
  {0, 1, 1, PROJECT_SEAGULL, BAND_1258, PHASE_PD},
  {0, 1, 2, PROJECT_SEAGULL, BAND_1258, PHASE_DP},
  {0, 1, 3, PROJECT_SEAGULL, BAND_1258, PHASE_SP},
  {0, 1, 4, PROJECT_SEAGULL, BAND_1258, PHASE_AP},
  {0, 1, 5, PROJECT_SEAGULL, BAND_1258, PHASE_PQ},
  {0, 1, 6, PROJECT_SEAGULL, BAND_1258, PHASE_TP},
  {0, 1, 7, PROJECT_SEAGULL, BAND_1258, PHASE_MP},
  {0, 1, 11, PROJECT_SEAGULL, BAND_125,PHASE_EVM2}, //New Rita EVM
  //====LTE(Rex)====
  {0, 2, 1, PROJECT_SEAGULL, BAND_1245, PHASE_PD},
  {0, 2, 2, PROJECT_SEAGULL, BAND_1245, PHASE_DP},
  {0, 2, 3, PROJECT_SEAGULL, BAND_1245, PHASE_SP}, //use ALPS connector , W/O card "Open"
  {0, 2, 4, PROJECT_SEAGULL, BAND_1245, PHASE_SP2}, //use JAE connector , W/O card "Close"
  {0, 2, 5, PROJECT_SEAGULL, BAND_1245, PHASE_AP},
  {0, 2, 6, PROJECT_SEAGULL, BAND_1245, PHASE_PQ},
  {0, 2, 7, PROJECT_SEAGULL, BAND_1245, PHASE_TP},
  {0, 2, 8, PROJECT_SEAGULL, BAND_1245, PHASE_MP},
  {0, 2, 9, PROJECT_SEAGULL, BAND_1245, PHASE_MP_PCN3000},
  //====LTE(Gina)===
  {0, 3, 0, PROJECT_SEAGULL, BAND_1357820, PHASE_EVM},
  {0, 3, 1, PROJECT_SEAGULL, BAND_1357820, PHASE_PD},
  {0, 3, 2, PROJECT_SEAGULL, BAND_1357820, PHASE_DP},
  {0, 3, 3, PROJECT_SEAGULL, BAND_1357820, PHASE_SP}, //use ALPS connector , W/O card "Open"
  {0, 3, 4, PROJECT_SEAGULL, BAND_1357820, PHASE_SP2}, //use JAE connector , W/O card "Close"
  {0, 3, 5, PROJECT_SEAGULL, BAND_1357820, PHASE_AP},
  {0, 3, 6, PROJECT_SEAGULL, BAND_1357820, PHASE_PQ},
  {0, 3, 7, PROJECT_SEAGULL, BAND_1357820, PHASE_TP},
  {0, 3, 8, PROJECT_SEAGULL, BAND_1357820, PHASE_MP},
  {0, 3, 9, PROJECT_SEAGULL, BAND_1357820, PHASE_MP_PCN3000},
  //================
  {-1, -1, -1, PROJECT_MAX, BAND_MAX, PHASE_MAX}
};

/*
  EE define range for 1% deviation
  { 0,  540000,  569000},
  { 1,  720000,  757000},
  { 2,  906000,  950000},
  { 3, 1099000, 1150000},
  { 4, 1290000, 1346000},
  { 5, 1507000, 1568000},
  { 6, 1706000, 1771000},
  { 7, 1943000, 2011000},
  { 8, 2159000, 2228000},
  { 9, 2416000, 2484000},
  {10, 2647000, 2714000},
  {11, 2896000, 2961000},
*/

//SW define range
static const fih_adc_map ADC_MAP[]=
{ //uV
  { 0,  490000,  644000},
  { 1,  645000,  830000},
  { 2,  831000, 1023000},
  { 3, 1024000, 1219000},
  { 4, 1220000, 1426000},
  { 5, 1427000, 1636000},
  { 6, 1637000, 1856000},
  { 7, 1857000, 2084000},
  { 8, 2085000, 2321000},
  { 9, 2322000, 2565000},
  {10, 2566000, 2804000},
  {11, 2805000, 3017000},
  {-1,   -1,   -1}
};

static unsigned int pm_gpio[3]={3, 6, 5};
static unsigned int str_idx[3]={0};
static int adc_volt[3]={0};
static int adc_lv[3]={0};
static struct qpnp_vadc_chip *hwid_vadc;

fih_product_id_type FIH_PROJECT_ID = PROJECT_SEAGULL;
fih_product_phase_type FIH_PHASE_ID = PHASE_EVM;
fih_band_id_type FIH_BAND_ID = BAND_1357820;
fih_sim_type FIH_SIM_ID = SINGLE_SIM;

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

int fih_get_hwid_adc(int index)
{
  if( (index>=1)&&(index<=3) )
  {
    return adc_volt[index-1];
  }
  else
  {
    pr_info("FIH invalid HWID adc index %d\r\n",index);
    return 0;
  }
}
EXPORT_SYMBOL(fih_get_hwid_adc);

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
}

void hwid_info(void)
{
  id2str_map();

  pr_info("adc level %d %d %d\r\n",adc_lv[0],adc_lv[1],adc_lv[2]);
  pr_info("FIH_PROJECT: %s\r\n",project_id_map[str_idx[0]].STR);
  pr_info("FIH_BAND:    %s\r\n",band_id_map[str_idx[1]].STR);
  pr_info("FIH_PHASE:   %s\r\n",phase_id_map[str_idx[2]].STR);
}

int adc_map(int64_t uV)
{
  int i;

  for (i=0; ADC_MAP[i].LEVEL!=-1; i++)
  {
    if( (uV < ADC_MAP[i].MAX)&&(uV > ADC_MAP[i].MIN) )
    {
      return ADC_MAP[i].LEVEL;
    }
  }
  return -1;
}

unsigned int hwid_map(void)
{
  int i, pmic_idx;
  int rc=-1;
  struct qpnp_vadc_result *result;

  result = kzalloc(sizeof(*result), GFP_KERNEL);

  for (i=0;i<3;i++)
  {
    pmic_idx = qpnp_pin_map("pm8226-gpio",pm_gpio[i]);
    rc = qpnp_pin_config(pmic_idx,&low);
    if(rc)
    {
      pr_err("qpnp_pin config low failed, rc=%d\n", rc);
      goto fail;
    }

    rc = qpnp_vadc_read(hwid_vadc,P_MUX7_1_3,result);
    if(rc)
    {
      pr_err("qpnp_vadc read failed, rc=%d\n", rc);
      goto fail;
    }

    pr_info("pm_gpio %d adc %lld uV\r\n",pm_gpio[i],result->physical);
    adc_volt[i]=(int)result->physical;
    adc_lv[i]=adc_map(result->physical);

    rc = qpnp_pin_config(pmic_idx,&high);
    if(rc)
    {
      pr_err("qpnp_pin config high failed, rc=%d\n", rc);
      goto fail;
    }
  }

  for (i=0;HWID_MAP[i].PROJECT_LV!=-1;i++)
  {
    if( (adc_lv[0]==HWID_MAP[i].PROJECT_LV) \
      &&(adc_lv[1]==HWID_MAP[i].BAND_LV) \
      &&(adc_lv[2]==HWID_MAP[i].PHASE_LV)
      )
    {
      FIH_PROJECT_ID = HWID_MAP[i].PROJECT;
      FIH_BAND_ID    = HWID_MAP[i].BAND;
      FIH_PHASE_ID   = HWID_MAP[i].PHASE;
      rc=0;
      break;
    }
  }

fail:
  kfree(result);
  return rc;
}

unsigned int env_init(void)
{
  int i,pmic_idx;
  int rc=0;

  for (i=0;i<3;i++)
  {
    pmic_idx = qpnp_pin_map("pm8226-gpio",pm_gpio[i]);
    rc = qpnp_pin_config(pmic_idx,&high);
    if(rc)
    {
      pr_err("qpnp_pin config high failed, rc=%d\n", rc);
      goto fail;
    }
  }

fail:
  return rc;
}

void fih_hwid_get(void)
{
  int rc=0;
  struct qpnp_vadc_result *result;

  result = kzalloc(sizeof(*result), GFP_KERNEL);

  pr_info("======FIH HWID init start======\r\n");
  if( env_init() )
  {
    pr_err("FIH HWID init fail\r\n");
    pr_info("================================\r\n");
    goto fail;
  }

  if(hwid_map())
  {
    pr_info("******map fail use default HWID******\r\n");
  }

  hwid_info();

  //init again to save power
  env_init();
  rc = qpnp_vadc_read(hwid_vadc,P_MUX7_1_3,result);
  if(rc)
  {
    pr_err("re-init adc read failed, rc=%d\n", rc);
  }
  else
  {
    pr_info("re-init adc %lld uV \r\n",result->physical);
  }
  pr_info("================================\r\n");

fail:
  kfree(result);
}
EXPORT_SYMBOL(fih_hwid_get);

static int fih_hwid_probe(struct platform_device *pdev)
{
  int rc=0;

  hwid_vadc=qpnp_get_vadc(&pdev->dev, "hwid");
  if (IS_ERR(hwid_vadc))
  {
    rc = PTR_ERR(hwid_vadc);
    if (rc != -EPROBE_DEFER)
    {
      pr_err("vadc property missing, rc=%d\n", rc);
    }
    else
    {
      pr_err("vadc failed, rc=%d\n", rc);
    }
  }
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

