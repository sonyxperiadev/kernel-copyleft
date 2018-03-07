#ifndef __CEI_HW_ID_H
#define __CEI_HW_ID_H

#include <linux/fs.h>
#define HARDWARE_INFO_COMMANDLINE_LENGTH 8
/* Implement CCI HW ID/PROJECT ID */

enum CEI_PHASE_ID_TYPE {
	PDP = 0,
	DP,
	SP,
	AP,
	TP,
	PQ,
	MP,
	HW_ID_SIZE,
};

enum CEI_PROJECT_ID_TYPE {
	BY12 = 0,
	BY13,
	BY14,
	BY15,
	BY16,
	BY22,
	BY23,
	BY24,
	BY25,
};

enum CEI_SKU_ID_TYPE {
	GINA = 0,
	APAC,
	REX,
};

enum CEI_SIMSLOT_ID_TYPE {
	SS = 0,
	DS,
};

typedef struct
{
	int cei_phase_id;
	int cei_project_id;
	int cei_sku_id;
} cei_hardware_info;

extern char *get_cei_phase_id(void);
extern char *get_cei_project_id(void);
extern char *get_cei_sku_id(void);
extern char *get_cei_simslot_id(void);
extern char *get_cei_mb_id(void);
extern char *get_cei_fp_id(void);
extern int get_phase_name_index(char *name);
extern int get_simslot_name_index(char *name);

#endif /* __CCI_HW_ID_H */
