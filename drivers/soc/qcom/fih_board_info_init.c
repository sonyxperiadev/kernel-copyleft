/*
* Copyright(C) 2011-2014 Foxconn International Holdings, Ltd. All rights reserved
*/

#include <linux/export.h>
#include <linux/module.h>
#include <linux/err.h>

#include <linux/version_host.h>
#include <linux/fih_hw_info.h>
#include <linux/fih_sw_info.h>

#include <soc/qcom/smem.h>

char fih_nonHLOS_version[64];
char fih_nonHLOS_git_head[128];
char *fih_imei_from_cmdline = NULL;   /*BSP-LC-Write_IMEI_MEID_to_NV-00 +*/


/*
   Get and print the HLOS image version information
*/
void fih_get_HLOS_info(void)
{
    char   fih_host_version[32];
    snprintf(fih_host_version, sizeof(fih_host_version), "%s.%s.%s.%s",
               VER_HOST_BSP_VERSION,
               VER_HOST_PLATFORM_NUMBER,
               VER_HOST_BRANCH_NUMBER,
               VER_HOST_BUILD_NUMBER);

    pr_info("=======================================================\r\n");
    pr_info("FIH HLOS version = %s \r\n",fih_host_version);
    pr_info("FIH HLOS git head = %s \r\n",VER_HOST_GIT_COMMIT);
	pr_info("=======================================================\r\n");
}


/*
   Get the print NON-HLOS image version information
   This function will call when modem subsystem is UP!!
*/
void fih_get_nonHLOS_info(void)
{


  struct smem_oem_info *fih_smem_info = smem_alloc(SMEM_ID_VENDOR0, sizeof(*fih_smem_info), 0, SMEM_ANY_HOST_FLAG);

  if (fih_smem_info==NULL)
  {
    pr_err("FIH get nonHLOS info : fih_smem_info is NULL\r\n");
    return;
  }

  snprintf(fih_nonHLOS_git_head,sizeof(fih_nonHLOS_git_head),fih_smem_info->nonHLOS_git_head);
  snprintf(fih_nonHLOS_version, sizeof(fih_nonHLOS_version),fih_smem_info->nonHLOS_version);

  pr_info("=======================================================\r\n");
  pr_info("FIH nonHLOS git head = %s \r\n",fih_nonHLOS_git_head);
  pr_info("FIH non-HLOS version = %s \r\n",fih_nonHLOS_version);
  pr_info("=======================================================\r\n");

}
EXPORT_SYMBOL(fih_get_nonHLOS_info);


/*Get NON-HLOS version*/
char *fih_get_nonHLOS_version(void)
{
  return fih_nonHLOS_version;
}
EXPORT_SYMBOL(fih_get_nonHLOS_version);


/*Get NON-HLOS git head versioni*/
char *fih_get_nonHLOS_git_head(void)
{
  return fih_nonHLOS_git_head;
}
EXPORT_SYMBOL(fih_get_nonHLOS_git_head);


/*
   Get HWID information and save to smem
*/
void fih_set_hwid2smem(void)
{
  unsigned int hwid = 0;
  struct smem_oem_info *fih_smem_info = smem_alloc(SMEM_ID_VENDOR0, sizeof(*fih_smem_info), 0, SMEM_ANY_HOST_FLAG);

  if (fih_smem_info==NULL)
  {
    pr_err("FIH set oem info : fih_smem_info is NULL\r\n");
    return;
  }

  hwid |= fih_get_product_id()&0xff;
  hwid |= (fih_get_product_phase()&0xff) << PHASE_ID_SHIFT_MASK;
  hwid |= (fih_get_band_id()&0xff) << BAND_ID_SHIFT_MASK;
  hwid |= (fih_get_sim_id()&0x03) << SIM_ID_SHIFT_MASK;
  fih_smem_info->hwid = hwid;

  pr_info("=======================================================\r\n");
  pr_info("FIH set hwid 0x%08X to smem\r\n",fih_smem_info->hwid);
  pr_info("=======================================================\r\n");
}

/*BSP-LC-Write_IMEI_MEID_to_NV-00 +[*/
/*
 * 1.This function will get IMEI string form cmdline and store it 
 *   in fih_imei_from_cmdline
 * 2.For SoMC new S1 boot, the prefix string of IMEI is 
 *   "oemandroidboot.phoneid"
 */
static int get_imei_from_cmdline(char *str)
{
  fih_imei_from_cmdline = str;
  pr_info("Get IMEI from cmdline = %s", fih_imei_from_cmdline);
  return 0;
}
early_param("oemandroidboot.phoneid", get_imei_from_cmdline);

/*
 * Calculate the check bit value
 */
static void imei_check_digit(char *check)
{	
    int digit = 0, digitDouble = 0, index=0;
    char data[9]={0};
    
    data[1] = ((check[0] << 4) & 0xf0) | (check[1] & 0xf);
    data[2] = ((check[2] << 4) & 0xf0) | (check[3] & 0xf);
    data[3] = ((check[4] << 4) & 0xf0) | (check[5] & 0xf);
    data[4] = ((check[6] << 4) & 0xf0) | (check[7] & 0xf);
    data[5] = ((check[8] << 4) & 0xf0) | (check[9] & 0xf);
    data[6] = ((check[10] << 4) & 0xf0) | (check[11] & 0xf);
    data[7] = ((check[12] << 4) & 0xf0) | (check[13] & 0xf);
    
    for( index = 1; index < 8; index ++ )
    {
      digit       += (data[index] >> 4) & 0x0F;
      digitDouble =  (data[index] & 0x0F) * 2;
      digit       += digitDouble / 10;
      digit       += digitDouble % 10;
    } 
    
    if( 0 == ( digit % 10 ) )
        data[8] = 0x00;  
    else
        data[8] =  ( 10 - ( digit % 10 ) )  & 0x0F;

    {
        if(data[8] != (check[14] & 0x0f)){
            //ALOGE("IMEI value is not correct. Modify the last item from %x to %x", check[14], (data[8] + 0x30));
            check[14] = data[8] + 0x30;            
        }
    }
}

/*
 * Write IMEI value to share memory
 */
void fih_set_imei2smem(void)
{

  struct smem_oem_info *fih_smem_info = smem_alloc(SMEM_ID_VENDOR0, sizeof(*fih_smem_info), 0, SMEM_ANY_HOST_FLAG);

  if (fih_smem_info==NULL)
  {
    pr_err("FIH set oem info : fih_smem_info is NULL\r\n");
    return;
  }

  fih_smem_info->imei_status = 0;

  if(fih_imei_from_cmdline != NULL)
  {
      fih_imei_from_cmdline += 5;    /*0000:xxxxxxxxxxxxxx00,0000:xxxxxxxxxxxxxx00*/

      imei_check_digit(fih_imei_from_cmdline);
  
      fih_smem_info->imei_1[0] = 0x8;
      fih_smem_info->imei_1[1] = ((fih_imei_from_cmdline[0] << 4) & 0xf0) | 0xa;
      fih_smem_info->imei_1[2] = ((fih_imei_from_cmdline[2] << 4) & 0xf0) | (fih_imei_from_cmdline[1] & 0xf);
      fih_smem_info->imei_1[3] = ((fih_imei_from_cmdline[4] << 4) & 0xf0) | (fih_imei_from_cmdline[3] & 0xf);
      fih_smem_info->imei_1[4] = ((fih_imei_from_cmdline[6] << 4) & 0xf0) | (fih_imei_from_cmdline[5] & 0xf);
      fih_smem_info->imei_1[5] = ((fih_imei_from_cmdline[8] << 4) & 0xf0) | (fih_imei_from_cmdline[7] & 0xf);
      fih_smem_info->imei_1[6] = ((fih_imei_from_cmdline[10] << 4) & 0xf0) | (fih_imei_from_cmdline[9] & 0xf);
      fih_smem_info->imei_1[7] = ((fih_imei_from_cmdline[12] << 4) & 0xf0) | (fih_imei_from_cmdline[11] & 0xf);
      fih_smem_info->imei_1[8] = ((fih_imei_from_cmdline[14] << 4) & 0xf0) | (fih_imei_from_cmdline[13] & 0xf);  

      pr_info("IMEI in share memory = 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X", 
          fih_smem_info->imei_1[0], fih_smem_info->imei_1[1], fih_smem_info->imei_1[2], fih_smem_info->imei_1[3],
          fih_smem_info->imei_1[4], fih_smem_info->imei_1[5], fih_smem_info->imei_1[6], fih_smem_info->imei_1[7],
          fih_smem_info->imei_1[8]);


      fih_smem_info->imei_status = 1;   /*Single SIM only has one IMEI value*/
    
      fih_imei_from_cmdline += 21;
      
      if(fih_imei_from_cmdline[0] == ':')
      {
        fih_imei_from_cmdline += 1;
        imei_check_digit(fih_imei_from_cmdline);
  
        fih_smem_info->imei_2[0] = 0x8;
        fih_smem_info->imei_2[1] = ((fih_imei_from_cmdline[0] << 4) & 0xf0) | 0xa;
        fih_smem_info->imei_2[2] = ((fih_imei_from_cmdline[2] << 4) & 0xf0) | (fih_imei_from_cmdline[1] & 0xf);
        fih_smem_info->imei_2[3] = ((fih_imei_from_cmdline[4] << 4) & 0xf0) | (fih_imei_from_cmdline[3] & 0xf);
        fih_smem_info->imei_2[4] = ((fih_imei_from_cmdline[6] << 4) & 0xf0) | (fih_imei_from_cmdline[5] & 0xf);
        fih_smem_info->imei_2[5] = ((fih_imei_from_cmdline[8] << 4) & 0xf0) | (fih_imei_from_cmdline[7] & 0xf);
        fih_smem_info->imei_2[6] = ((fih_imei_from_cmdline[10] << 4) & 0xf0) | (fih_imei_from_cmdline[9] & 0xf);
        fih_smem_info->imei_2[7] = ((fih_imei_from_cmdline[12] << 4) & 0xf0) | (fih_imei_from_cmdline[11] & 0xf);
        fih_smem_info->imei_2[8] = ((fih_imei_from_cmdline[14] << 4) & 0xf0) | (fih_imei_from_cmdline[13] & 0xf);  
    
        fih_smem_info->imei_status = 2; /*Dual SIM has two IMEI values*/
      }
  } 
}
/*BSP-LC-Write_IMEI_MEID_to_NV-00 +]*/

/*
   Verify if the modem can get correct hwid form smem 
*/
/*BSP-ELuo-HWID_VERIFY-00+[*/
void fih_get_hwid_From_smem(void)
{
  struct smem_oem_info *fih_smem_info = smem_alloc(SMEM_ID_VENDOR0, sizeof(*fih_smem_info), 0, SMEM_ANY_HOST_FLAG);

  if (fih_smem_info==NULL)
  {
    pr_err("FIH set oem info : fih_smem_info is NULL\r\n");
    return;
  }

  pr_info("=======================================================\r\n");
  pr_info("FIH get hwid 0x%08X from smem\r\n",fih_smem_info->hwid_verify);
  pr_info("=======================================================\r\n");
}
EXPORT_SYMBOL(fih_get_hwid_From_smem);
/*BSP-ELuo-HWID_VERIFY-00]+*/

/*
   1. Init FIH's smem region 
   2. Get and print the version of HLOS
   3. Create the FIH's PROC entry
*/

int __init fih_board_info_init(void)
{
	pr_info("fih_board_info_init() start!!!\r\n");
	fih_get_HLOS_info();
	fih_set_hwid2smem();
	fih_proc_init();
	fih_set_imei2smem();    /*BSP-LC-Write_IMEI_MEID_to_NV-00 +*/
	pr_info("fih_board_info_init() finish!!!\r\n");
	return 0;
}
subsys_initcall(fih_board_info_init);
