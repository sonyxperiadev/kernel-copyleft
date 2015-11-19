#ifndef _FIH_GPIO_INIT_H
#define _FIH_GPIO_INIT_H

#include <linux/fih_hw_info.h>

#define FIH_GPIO_NUM 122
#define FIH_VALID_PIN(pin) (pin>=0)&&(pin<FIH_GPIO_NUM)

#define FIH_GP_IN_BIT   0
#define FIH_GP_OUT_BIT  1

#define FIH_GP_CFG(base, pin)         (base + 0x0 + 0x1000 * (pin))
#define FIH_GP_INOUT(base, pin)       (base + 0x4 + 0x1000 * (pin))
#define FIH_GP_INTR_CFG(base, pin)    (base + 0x8 + 0x1000 * (pin))
#define FIH_GP_INTR_STATUS(base, pin) (base + 0xc + 0x1000 * (pin))

enum
{
  FIH_INPUT,
  FIH_OUTPUT,
};

enum
{
  FIH_NO_PULL,
  FIH_PULL_DOWN,
  FIH_KEEPER,
  FIH_PULL_UP,
};

enum
{
  FIH_2MA,
  FIH_4MA,
  FIH_6MA,
  FIH_8MA,
  FIH_10MA,
  FIH_12MA,
  FIH_14MA,
  FIH_16MA,
};

enum
{
  FIH_DISABLE,
  FIH_ENABLE,
  FIH_ENABLE_HIGH, //for output pin
  FIH_ENABLE_LOW,  //for output pin
};

#define FIH_CFG(func, dir, pull, drvstr) \
    (((func)   & 0xf) << 2)    |   \
    (((dir)    & 0x1) << 9)    |   \
    (((pull)   & 0x3) << 0)    |   \
    (((drvstr) & 0xf) << 6)

#define FIH_FUNC(fih_cfg)   (((fih_cfg) >> 2) & 0xf)
#define FIH_DIR(fih_cfg)    (((fih_cfg) >> 9) & 0x1)
#define FIH_PULL(fih_cfg)   (((fih_cfg) >> 0) & 0x3)
#define FIH_DRVSTR(fih_cfg) (((fih_cfg) >> 6) & 0xf)

typedef struct
{
  int pin;
  unsigned int config;
  int enable;
}FIH_GPIO_TABLE;

typedef struct
{
  fih_product_phase_type phase;
  fih_sim_type sim;
  FIH_GPIO_TABLE* table;
}FIH_GPIO_MAP;

void fih_gp_out_set(unsigned int pin, int level);
int fih_gp_in_get(unsigned int pin);
void fih_gp_cfg(unsigned int pin, unsigned int config);
#endif



