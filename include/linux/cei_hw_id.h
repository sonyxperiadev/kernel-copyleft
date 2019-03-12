#ifndef __CEI_HW_ID_H
#define __CEI_HW_ID_H

#include <linux/fs.h>
#define HARDWARE_INFO_COMMANDLINE_LENGTH 8
/* Implement CCI HW ID/PROJECT ID */

enum CEI_PHASE_ID_TYPE {
	PDP = 0,
	DP,
	SP,
	AP1,
	AP2,
	TP,
	PQ,
	MP,
	HW_ID_SIZE,
};

enum CEI_PROJECT_ID_TYPE {
	BY35 = 0,
	BY36,
	BY34,
	BY37,
	BY38,
	BY39,
	AY35,
	AY36,
	AY37,
	AY38,
	AY39,
};

enum CEI_SKU_ID_TYPE {
	GINA = 0,
	APAC,
	REX,
	APAC_2
};

enum CEI_SIMSLOT_ID_TYPE {
	SS = 0,
	DS,
};

typedef struct {
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
extern char *get_cei_android_boot_mode(void);
extern int get_simslot_name_index(char *name);
extern char *get_cei_android_boot_mode(void);

#endif /* __CCI_HW_ID_H */
