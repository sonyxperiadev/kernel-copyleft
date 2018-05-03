/*
* Copyright(C) 2011-2014 Foxconn International Holdings, Ltd. All rights reserved
*/
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/fih_hw_info.h>
#include <linux/version_host.h>

/* CORE-HC-productinfo-00+[ */
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/fih_sw_info.h>
/* CORE-HC-productinfo-00+] */

static int proc_calc_metrics(char *page, char **start, off_t off,
                             int count, int *eof, int len)
{
  if (len <= off+count) *eof = 1;
  *start = page + off;
  len -= off;
  if (len > count) len = count;
  if (len < 0 ) len = 0;
  return len;
}

static int device_model_read_proc(char *page, char **start, off_t off,
                                  int count, int *eof, void *data)
{
  int len, i;
  unsigned int project = fih_get_product_id();
  char ver[16]= {0} ;

  for(i=0; project_id_map[i].ID != PROJECT_MAX; i++)
  {
    if(project_id_map[i].ID == project)
    {
      strncpy(ver, project_id_map[i].STR ,project_id_map[i].LEN);
      ver[project_id_map[i].LEN]='\0';
      break;
    }
  }

  len = snprintf(page, count, "%s\n", ver);

  return proc_calc_metrics(page, start, off, count, eof, len);
}

static int phase_id_read_proc(char *page, char **start, off_t off,
                              int count, int *eof, void *data)
{
  int len, i;
  int phase = fih_get_product_phase();
  char ver[16]= {0};

  for(i=0; phase_id_map[i].ID != PHASE_MAX; i++)
  {
    if(phase_id_map[i].ID == phase)
    {
      strncpy(ver, phase_id_map[i].STR ,phase_id_map[i].LEN);
      ver[phase_id_map[i].LEN]='\0';
      break;
    }
  }

  len = snprintf(page, count, "%s\n", ver);

  return proc_calc_metrics(page, start, off, count, eof, len);
}

static int band_read_proc(char *page, char **start, off_t off,
                          int count, int *eof, void *data)
{
  int len, i;
  int band = fih_get_band_id();
  char ver[16]= {0};

  for(i=0; band_id_map[i].ID != BAND_MAX; i++)
  {
    if(band_id_map[i].ID == band)
    {
      strncpy(ver, band_id_map[i].STR ,band_id_map[i].LEN);
      ver[band_id_map[i].LEN]='\0';
      break;
    }
  }

  len = snprintf(page, count, "%s\n", ver);

  return proc_calc_metrics(page, start, off, count, eof, len);
}

static int siminfo_read_proc(char *page, char **start, off_t off,
                             int count, int *eof, void *data)
{
  int len, i;
  int sim = fih_get_sim_id();
  char ver[16]= {0} ;

  for(i=0; sim_id_map[i].ID != SIM_MAX; i++)
  {
    if(sim_id_map[i].ID == sim)
    {
      strncpy(ver, sim_id_map[i].STR ,sim_id_map[i].LEN);
      ver[sim_id_map[i].LEN]='\0';
      break;
    }
  }

  len = snprintf(page, count, "%s\n", ver);

  return proc_calc_metrics(page, start, off, count, eof, len);
}

static int nonHLOS_ver_read_proc(char *page, char **start, off_t off,
                                  int count, int *eof, void *data)
{
    int len;
    char ver[30];
    snprintf(ver, sizeof(ver), fih_get_nonHLOS_version());

    len = snprintf(page, count, "%s\n",ver);

    return proc_calc_metrics(page, start, off, count, eof, len);
}

static int HLOS_ver_read_proc(char *page, char **start, off_t off,
                                   int count, int *eof, void *data)
{
  int len;
  len = snprintf(page, count, "%s.%s.%s.%s\n",
                 VER_HOST_BSP_VERSION,
                 VER_HOST_PLATFORM_NUMBER,
                 VER_HOST_BRANCH_NUMBER,
                 VER_HOST_BUILD_NUMBER);

  return proc_calc_metrics(page, start, off, count, eof, len);
}

static int HLOS_git_head_read_proc(char *page, char **start, off_t off,
                                   int count, int *eof, void *data)
{
  int len;

  len = snprintf(page, count, "%s\n", VER_HOST_GIT_COMMIT);

  return proc_calc_metrics(page, start, off, count, eof, len);
}
static int nonHLOS_git_head_read_proc(char *page, char **start, off_t off,
                                      int count, int *eof, void *data)
{
  int len;
  char head[64];
  snprintf(head, sizeof(head), fih_get_nonHLOS_git_head());

  len = snprintf(page, count, "%s\n",head);

  return proc_calc_metrics(page, start, off, count, eof, len);
}

/* CORE-HC-productinfo-00+[ */
void *get_productinfo_virt_addr(void) {
  static void *virt_addr = NULL;

  if (unlikely(virt_addr == NULL)){
    virt_addr = ioremap(FIH_PRODUCTINFO_ADDR, FIH_PRODUCTINFO_LEN);
  }

  return virt_addr;
}
EXPORT_SYMBOL(get_productinfo_virt_addr);

static int official_ver_read_proc(char *page, char **start, off_t off,
                       int count, int *eof, void *data)
{
    int len;
    char *productinfo_virt_addr;

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();

    if (productinfo_virt_addr != NULL)
    {
        len = snprintf(page, count, "%s", productinfo_virt_addr+OFFICIAL_VER_OFFSET);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return proc_calc_metrics(page, start, off, count, eof, len);
}

static int official_ver_write_proc(struct file *file, const char __user *buf,
                                   unsigned long count, void *data)
{
    char item[PRODUCTINFO_ITEM_LEN];
    char *productinfo_virt_addr;

    if (!count)
    {
        pr_err("count = 0\n");
        return 0;
    }
    if (count > PRODUCTINFO_ITEM_LEN - 1) {
        pr_err("buffer overflow, count = %d\n", (int)count);
        return -EINVAL;
    }
    if (copy_from_user(item, buf, count)) {
        pr_err("copy from user failed\n");
        return -EFAULT;
    }

    item[count] = '\0';

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();
    if (productinfo_virt_addr != NULL)
    {
        snprintf(productinfo_virt_addr+OFFICIAL_VER_OFFSET, PRODUCTINFO_ITEM_LEN, "%s", item);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return count;
}

static int vender_ver_read_proc(char *page, char **start, off_t off,
                       int count, int *eof, void *data)
{
    int len;
    char *productinfo_virt_addr;

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();

    if (productinfo_virt_addr != NULL)
    {
        len = snprintf(page, count, "%s", productinfo_virt_addr+VENDER_VER_OFFSET);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return proc_calc_metrics(page, start, off, count, eof, len);
}

static int vender_ver_write_proc(struct file *file, const char __user *buf,
                                   unsigned long count, void *data)
{
    char item[PRODUCTINFO_ITEM_LEN];
    char *productinfo_virt_addr;

    if (!count) {
        pr_err("count = 0\n");
        return 0;
    }
    if (count > PRODUCTINFO_ITEM_LEN - 1) {
        pr_err("buffer overflow, count = %d\n", (int)count);
        return -EINVAL;
    }
    if (copy_from_user(item, buf, count)) {
        pr_err("copy from user failed\n");
        return -EFAULT;
    }

    item[count] = '\0';

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();
    if (productinfo_virt_addr != NULL)
    {
        snprintf(productinfo_virt_addr+VENDER_VER_OFFSET, PRODUCTINFO_ITEM_LEN, "%s", item);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return count;
}

static int product_model_read_proc(char *page, char **start, off_t off,
                       int count, int *eof, void *data)
{
    int len;
    char *productinfo_virt_addr;

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();

    if (productinfo_virt_addr != NULL)
    {
        len = snprintf(page, count, "%s", productinfo_virt_addr+PRODUCT_MODEL_OFFSET);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return proc_calc_metrics(page, start, off, count, eof, len);
}

static int product_model_write_proc(struct file *file, const char __user *buf,
                                   unsigned long count, void *data)
{
    char item[PRODUCTINFO_ITEM_LEN];
    char *productinfo_virt_addr;

    if (!count) {
        pr_err("count = 0\n");
        return 0;
    }
    if (count > PRODUCTINFO_ITEM_LEN - 1) {
        pr_err("buffer overflow, count = %d\n", (int)count);
        return -EINVAL;
    }
    if (copy_from_user(item, buf, count)) {
        pr_err("copy from user failed\n");
        return -EFAULT;
    }

    item[count] = '\0';

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();
    if (productinfo_virt_addr != NULL)
    {
        snprintf(productinfo_virt_addr+PRODUCT_MODEL_OFFSET, PRODUCTINFO_ITEM_LEN, "%s", item);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return count;
}

static int build_type_read_proc(char *page, char **start, off_t off,
                       int count, int *eof, void *data)
{
    int len;
    char *productinfo_virt_addr;

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();

    if (productinfo_virt_addr != NULL)
    {
        len = snprintf(page, count, "%s", productinfo_virt_addr+BUILD_TYPE_OFFSET);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return proc_calc_metrics(page, start, off, count, eof, len);
}

static int build_type_write_proc(struct file *file, const char __user *buf,
                                   unsigned long count, void *data)
{
    char item[PRODUCTINFO_ITEM_LEN];
    char *productinfo_virt_addr;

    if (!count) {
        pr_err("count = 0\n");
        return 0;
    }
    if (count > PRODUCTINFO_ITEM_LEN - 1) {
        pr_err("buffer overflow, count = %d\n", (int)count);
        return -EINVAL;
    }
    if (copy_from_user(item, buf, count)) {
        pr_err("copy from user failed\n");
        return -EFAULT;
    }

    item[count] = '\0';

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();
    if (productinfo_virt_addr != NULL)
    {
        snprintf(productinfo_virt_addr+BUILD_TYPE_OFFSET, PRODUCTINFO_ITEM_LEN, "%s", item);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return count;
}

static struct
{
  char *name;
  int (*read_proc)(char*,char**,off_t,int,int*,void*);
  int (*write_proc)(struct file*, const char*,unsigned long, void*);
}*p2, productinfo[] =
     {
       {"official_ver",     official_ver_read_proc,     official_ver_write_proc},
       {"vender_ver",       vender_ver_read_proc,       vender_ver_write_proc},
       {"product_model",    product_model_read_proc,    product_model_write_proc},
       {"build_type",       build_type_read_proc,       build_type_write_proc},
       {NULL,},
     };
/* CORE-HC-productinfo-00+] */

static struct
{
  char *name;
  int (*read_proc)(char*,char**,off_t,int,int*,void*);
}*p, fih_info[] =
     {
       {"devmodel",         device_model_read_proc},
       {"phaseid",          phase_id_read_proc},
       {"bandinfo",         band_read_proc},
       {"siminfo",          siminfo_read_proc},
       {"nonHLOS_ver",      nonHLOS_ver_read_proc},
       {"HLOS_ver",         HLOS_ver_read_proc},
       {"nonHLOS_git_head", nonHLOS_git_head_read_proc},
       {"HLOS_git_head",    HLOS_git_head_read_proc},
       {NULL,},
     };

void fih_info_init(void)
{
    struct proc_dir_entry *entry;    /* CORE-HC-productinfo-00+ */

    for (p = fih_info; p->name; p++)
        create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);

    /* CORE-HC-productinfo-00+[ */
    for (p2 = productinfo; p2->name; p2++)
    {
        entry = create_proc_entry(p2->name, 0777, NULL);
        entry->read_proc = p2->read_proc;
        entry->write_proc = p2->write_proc;
    }
    /* CORE-HC-productinfo-00+] */
}
EXPORT_SYMBOL(fih_info_init);

void fih_info_remove(void)
{
    for (p = fih_info; p->name; p++)
        remove_proc_entry(p->name, NULL);

    /* CORE-HC-productinfo-00+[ */
    for (p2 = productinfo; p2->name; p2++)
        remove_proc_entry(p2->name, NULL);
    /* CORE-HC-productinfo-00+] */

}
EXPORT_SYMBOL(fih_info_remove);
