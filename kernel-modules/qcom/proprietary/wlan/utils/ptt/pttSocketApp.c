/*
 * Copyright (c) 2013-2016,2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * 2013-2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */




/******************************************************************************
 * pttSocketApp.c
 *
 * This file implements the Netlink Proxy Server. It listens for MAC SW
 * messages generated from a test script(Perl/Python) at a remote host,
 * over a TCP/IP connection and translates them into the corresponding
 * Netlink messages to be sent to the HDD/MAC SW.
 *
 * Krishna Reddy, 05/08/2003
 *
 ******************************************************************************/

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

/* WNI Header */
#include <aniNlMsg.h>
#include <aniErrors.h>
#include <aniAsfHdr.h>
#include <aniAsfMem.h>
//#include <aniAsfPortMapD.h>
#include <aniAsfIpc.h>
#include <aniAsfLog.h>
#include <aniAsfProcessUtil.h>
#include <aniNlFuncs.h>

#include "pttSocketApp.h"

#include "msg.h"
#include "diag_lsm.h"
#include "diagpkt.h"
#include "diagi.h"
#include "log.h"
#include "event.h"

#ifdef WLAN_KD_READY_NOTIFIER
#include <android/log.h>
#include <sys/capability.h>
#include "sys/prctl.h"
#include <pwd.h>
#include <sys/types.h>

#define LOGTAG   "WLAN_PSA"
#define LOG_PSA_E(...) __android_log_print(ANDROID_LOG_ERROR, LOGTAG, __VA_ARGS__)
#define LOG_PSA_V(...) __android_log_print(ANDROID_LOG_VERBOSE, LOGTAG, __VA_ARGS__)
#define LOG_PSA_D(...) __android_log_print(ANDROID_LOG_DEBUG, LOGTAG, __VA_ARGS__)
#define LOG_PSA_I(...) __android_log_print(ANDROID_LOG_INFO, LOGTAG, __VA_ARGS__)
#else
#define LOG_PSA_E printf
#define LOG_PSA_V printf
#define LOG_PSA_D printf
#define LOG_PSA_I printf
#endif /* WLAN_KD_READY_NOTIFIER */

/*
 * Globals
 */
tAniRttCtxt serverCtxt;
static int sigInt;
int   pass = 1;
int alreadyRegistered = 0;
int debug = 0;

int endianness=0; //0: little-endian, 1: big-endian

#ifdef WLAN_LOGGING_INFRA_SUPPORT

#define WLAN_LOG_TO_DIAG(xx_ss_id, xx_ss_mask, xx_fmt) \
    do { \
        if (xx_ss_mask & (MSG_BUILD_MASK_ ## xx_ss_id)) { \
            msg_const_type xx_msg = { \
              {__LINE__, (xx_ss_id), (xx_ss_mask)}, (NULL), msg_file}; \
            xx_msg.fmt = xx_fmt; \
            msg_send (&xx_msg); \
        } \
    } while  (0); \

const char wlanLoggingReady[] = "WLAN LOGGING READY";
#endif // WLAN_LOGGING_INFRA_SUPPORT

#ifdef WLAN_KD_READY_NOTIFIER
const char driverLoaded[]   = "KNLREADY";
const char driverUnLoaded[] = "KNLCLOSE";
static int isDriverLoaded   = 0;

#define WLAN_FTM_COMMAND_OFFSET 12
#define WLAN_FTM_OPCODE_OFFSET  14

static uint8 is_ffbm_mode = 0;
#endif /* WLAN_KD_READY_NOTIFIER */

#define WLAN_FTM_TEST_APP_F_75 22
#define WLAN_FTM_SUBSYS_TEST_CLIENT 11

/* PTT Command types */
#define PTT_MSG_READ_REGISTER       0x3040
#define PTT_MSG_WRITE_REGISTER      0x3041
#define PTT_MSG_READ_MEMORY         0x3044
#define PTT_MSG_WRITE_MEMORY        0x3045
#define PTT_MSG_LOG_DUMP_DBG        0x32A1

void *wlan_ftm_func_75(void *req_pkt, uint16 pkt_len);

static const diagpkt_user_table_entry_type wlan_ftm_test_tbl[] =
{ /* susbsys_cmd_code lo = 0 , susbsys_cmd_code hi = 0, call back function */
	{WLAN_FTM_TEST_APP_F_75, WLAN_FTM_TEST_APP_F_75, wlan_ftm_func_75},
};

int pttSocketAppRegister(int, tAniRttCtxt*);
/*
 * Hex dump the specified number of bytes from the passed buffer.
 */
void aniDumpBuf(char *buf, int cnt)
{
   int i;

   if (buf)
   {
      for (i=0; i<cnt; ++i)
      {
         if (i % 16 == 0)
         {
            LOG_PSA_E("\n%p: ",(buf + i));
         }
         LOG_PSA_E("%02X ", (unsigned int)(buf[i]));
      }
      LOG_PSA_E("\n");

   }
}

int logDumpCmd(int cmd, int arg1, int arg2, int arg3, int arg4) {
   //char buf[3000];
   char cmdl[150];
   //FILE *ptr;

   snprintf(cmdl, sizeof(cmdl), "%d %x %x %x %x ", cmd, arg1, arg2, arg3, arg4);
   printf("dump %s",  cmdl);

   //execlp("dump", cmdl);

   return 0;
}

/*
 * Sends data received from a Client (script) to the HDD in the Kernel
 * over a Netlink socket. Prepends a Netlink header to the buffer
 * before sending it to the HDD.
 */
int pttSocketAppSendMsgToKernel(tAniRttCtxt *pserver, int radio,
      tAniHdr *msg, int msgType, int msgLen)
{
    tAniNlHdr *wnl;
    char *localBuf;


    if ((localBuf = malloc(RTT_MAX_MSG_SIZE)) == NULL)
    {
        aniAsfLogMsg(LOG_ERR, ANI_WHERE, "pttSocketApp: %s, malloc failed\n", __FUNCTION__);
        return -1;
    }

    memset(localBuf, 0, RTT_MAX_MSG_SIZE);

    pserver->nl.nlmsg_len = aniNlAlign(sizeof(tAniNlHdr)) + ntohs(msgLen);
    pserver->nl.nlmsg_type = msgType; // send it to the HDD
    pserver->nl.nlmsg_seq++;

    if (pserver->nl.nlmsg_len > RTT_MAX_MSG_SIZE)
    {
        aniAsfLogMsg(LOG_INFO, ANI_WHERE,"Insufficient memory, will not send to aniAsfIpcSend ");
        free(localBuf);
        return -1;
    }

    // copy the netlink msg hdr first (assuming buf is 4 byte aligned)
    wnl = (tAniNlHdr *)localBuf;
    wnl->radio = radio;

    // setup the tAniHdr next
    wnl->wmsg.length = ntohs(msgLen);
    wnl->wmsg.type = ntohs(msg->type);

    aniAsfLogMsg(LOG_INFO, ANI_WHERE,"Sending to aniAsfIpcSend len=%d radio=%d nl size=%d", pserver->nl.nlmsg_len, wnl->radio, sizeof(struct nlmsghdr));
    memcpy(localBuf, &pserver->nl, sizeof(struct nlmsghdr));


    memcpy((char *)(wnl+1), (char *)(msg+1), msgLen - sizeof(tAniHdr));

#ifdef ANI_DEBUG
    printf("pttSocketApp: %s: Sending msg type = 0x%X, len = %d to kernel\n", __FUNCTION__, msgType, msgLen - 4);
    aniAsfLogMsg(LOG_INFO, ANI_WHERE,"Sending to aniAsfIpcSend len=%d radio=%d nl size=%d", pserver->nl.nlmsg_len, wnl->radio, sizeof(struct nlmsghdr));
    aniDumpBuf(localBuf, pserver->nl.nlmsg_len );
#endif

    if (!pass)
    {
        aniAsfLogMsg(LOG_INFO, ANI_WHERE,"-n OPTION! will not send to aniAsfIpcSend ");
        free(localBuf);
        return 0;
    }
    /*
    * push this message down to the HDD via the netlink
    * socket.
    */
    if (aniAsfIpcSend(pserver->ipcnl, localBuf, pserver->nl.nlmsg_len) < 0)
    {
        aniAsfLogMsg(LOG_ERR, ANI_WHERE, "%s: Could not send message to the kernel", __FUNCTION__);
        free(localBuf);
        return -1;
    }

    free(localBuf);
    return 0;
}



/// Callback function Invoked when there is a message from the
/// client(Perl script)
void pttSocketAppProcClientMsg(void *arg)
{
    int lenRcvd = 0;
    int msgLen;
    int nlmsgType = 0;
    tAniRttCtxt *pserver = (tAniRttCtxt *)arg;
    tAniIpc *nIpc = (tAniIpc *)pserver->clIpc;
    char   *data = NULL;
    char   *buf;
    int   *cmds;
    tAniHdr *msg;


    if ((buf = malloc(RTT_MAX_MSG_SIZE)) == NULL)
    {
        aniAsfLogMsg(LOG_ERR, ANI_WHERE, "pttSocketApp: malloc failed\n");
        return;
    }
    memset(buf, 0, RTT_MAX_MSG_SIZE);

    /*
    * read the msg hdr and then determine how many more bytes
    * to read from the socket based on the message type.
    */
    if ((aniAsfIpcRecv(nIpc, buf, 4)) <= 0)
    {
        /*
        * Check if the client connection was closed
        * clean up the connection context for this client.
        */
        aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s:Client Connection died\n", __FUNCTION__);
        free(buf);
        return;
    }
    msgLen = pttSocketRd32((char*) buf);
    if (endianness == 1) //big-endian, swapping is required
        msgLen = pttSocketSwapU32(msgLen);

#ifdef ANI_DEBUG
    printf("pttSocketApp: %s: First 4 bytes of rcvd msgLen=[0x%X]", __FUNCTION__, msgLen);
    aniAsfLogMsg(LOG_INFO, ANI_WHERE,"pttSocketApp: %s: First 4 bytes of rcvd msgLen=[0x%X]", __FUNCTION__, msgLen);
#endif
    /*
    * Now read the remaining bytes of the netsim msg based on
    * length field in the netsim msg hdr.
    */
    if (msgLen  >RTT_MAX_MSG_SIZE)
    {
        /*
        * Corrupted or malformed message, log it and drop the
        * message.
        */
        aniAsfLogMsg(LOG_INFO, ANI_WHERE, "pttSocketApp: bad message from client, len[%x]", msgLen);
        // flush the socket
        while((aniAsfIpcRecv(nIpc, buf, RTT_MAX_MSG_SIZE)) == RTT_MAX_MSG_SIZE);
        free(buf);
        return ;
    }

    data = buf + sizeof(int);

    if (msgLen  > (int)(RTT_MAX_MSG_SIZE - sizeof(int)))
    {
        /*
        * Corrupted or malformed message, log it and drop the
        * message.
        */
        aniAsfLogMsg(LOG_INFO, ANI_WHERE, "pttSocketApp: bad message from client, len[%x]", msgLen);
        // flush the socket
        while((aniAsfIpcRecv(nIpc, buf, RTT_MAX_MSG_SIZE)) == RTT_MAX_MSG_SIZE);
        free(buf);
        return ;
    }


    if ((lenRcvd = (aniAsfIpcRecv(nIpc, data, msgLen))) <= 0)
    {
        aniAsfLogMsg(ANI_IPCRECV_ERR);
        aniAsfIpcClose(nIpc);
        aniAsfLogMsg(LOG_INFO, ANI_WHERE, "pttSocketApp: %s - error while reading ipc\n", __FUNCTION__);
        free(buf);
        return;
    }
    // for dual radio support, next comes radio id
    pserver->radio =  pttSocketRd32((char*) data);
    data += sizeof(int);
    msgLen -=  sizeof(int);

    msg = (tAniHdr *) data;

//   aniAsfLogMsg(LOG_DEBUG, ANI_WHERE,
//      "pttSocketApp: %s: ANImsgType in rcvd msg[0x%X]\n",
//      __FUNCTION__, msg->type);


   // Perform message translation
    if (endianness == 1){ //B.E, so swapping is required
        msg->type = pttSocketSwapU16(msg->type);//ntohs(msg->type);
        msg->length = pttSocketSwapU16(msg->length);//ntohs(msg->length);
    }

   //printf("msgType=%x\n", msg->type);
   //aniDumpBuf((char *) msg, msg->length);
    switch(msg->type)
    {
        default:
            aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Passing rcvd Message with  ani mesgId [%x] radio %d", __FUNCTION__, msg->type, pserver->radio);
#ifdef ANI_DEBUG
            printf("%s: Passing rcvd Message with  ani mesgId [%x] radio %d\n", __FUNCTION__, msg->type, pserver->radio);
#endif
            nlmsgType = ANI_NL_MSG_PTT;
            break;

        case RTT_SME_MSG:
            aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Rcvd eANIAPI_AUTOTEST_ID_SME_MSG, animsgId = [%x]", __FUNCTION__, msg->type);
            break; //Not supported on Gen6
            nlmsgType = ANI_NL_MSG_MACSW;
            break;

        case RTT_REBOOT:
            aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Rcvd RTT_REBOOT Message, rebooting...", __FUNCTION__);
            break;//Not supported on Gen6
            system("reboot");
            break;

        case RTT_DRV_ENABLE:
            aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Rcvd eANIAPI_AUTOTEST_ID_ENABLE_ADAPTER animsgId = [%x]", __FUNCTION__, msg->type);
             break;//Msg not supported on Gen6
            nlmsgType = ANI_NL_MSG_PUMAC;
            break;

        case RTT_DRV_DISABLE:
            aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Rcvd  eANIAPI_AUTOTEST_ID_DISABLE_ADAPTER, animsgId = [%x]\n", __FUNCTION__, msg->type);
            break;//Msg not supported on Gen6
            nlmsgType = ANI_NL_MSG_PUMAC;
            break;

        case RTT_CFG_SET_REQ:
            /* eANIAPI_AUTOTEST_ID_CFG_SET_REQ */
            aniAsfLogMsg(LOG_DEBUG, ANI_WHERE, "%s: Rcvd a eANIAPI_AUTOTEST_ID_CFG_SET_REQ, animsgId = [%x]\n", __FUNCTION__, msg->type);
            break;//Msg not supported on Gen6
            nlmsgType = ANI_NL_MSG_MACSW;
            break;

        case eANI_NIM_CRDT_SYSDBG_LOG_DUMP_RSP:
            break;//Msg not supported on Gen6
            cmds = (int *) msg;
            aniDumpBuf((char *) cmds, 16);
            logDumpCmd(cmds[2], cmds[3], cmds[4], cmds[5], cmds[6]);
            break;
    }

#ifdef ANI_DEBUG
    printf("pttSocketApp: %s: Sending msg type = 0x%X, len = %d to kernel\n", __FUNCTION__, msg->type, msgLen - 4);
    printf("Dumping data rcvd from client socket len=%d\n", msgLen);
    //aniDumpBuf(data, msgLen );
    //aniDumpBuf((char *) msg, msgLen );
#endif
    if (nlmsgType)
    {
        // Send everything to the kernel
        aniAsfLogMsg(LOG_INFO, ANI_WHERE, "pttSocketApp: sending to the driver msgType=%x subtype=%x len=%d",  nlmsgType, msg->type, msgLen);
        if (pttSocketAppSendMsgToKernel(pserver, pserver->radio, msg, nlmsgType, msgLen) < 0)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE, "pttSocketApp: Could not send data to the HDD\n");
        }

    }
   free(buf);
}

/*
 * Send a message to the client that registration is already done
 */
void sendAlreadyRegisteredMessage (void *arg)
{
    tAniRttCtxt *pserver = (tAniRttCtxt *)arg;
    tAniRttCmdRspMsg    msg;

    msg.msgLen = htonl(8);
    msg.radio = htonl(0);
    msg.msgType = htons(RTT_RSP_ALREADY_REGISTERED_MSG);

#ifdef ANI_DEBUG
    printf("sendAlreadyRegisteredMessage: dumping msg  len=%d\n", msg.msgLen);
    aniDumpBuf((char *)&msg, htonl(msg.msgLen) + 4);
#else
    aniAsfLogMsg(LOG_DEBUG, ANI_WHERE, " Sending sendAlreadyRegisteredMessage msg of length %d to client", ntohl(msg.msgLen));
#endif

    if (pserver->clIpc)
    {
        if (aniAsfIpcSend(pserver->clIpc, (char *)&msg, ntohl(msg.msgLen) + 4/* for msgLen */) < 0)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE, "%s: Could not send message to the Client", __FUNCTION__);
        }
    }
    else
    {
        aniAsfLogMsg(LOG_INFO, ANI_WHERE,  "%s: Client connection does not exist dropping the msg\n", __FUNCTION__);
    }

}

/*
 * Send a message of 4 bytes (U32) of value 1 to the client to indicate endianness. The client will read the 4 bytes and figure out the endianess.
 */
void pttSendEndianIndication (void *arg)
{
    //tAniRttCtxt *pserver = (tAniRttCtxt *)arg;
    UNUSED(arg);
    unsigned int endian_check = 1;

    if ( *(unsigned char *)&endian_check == 1)
        endianness = 0; //little-endian
    else
        endianness = 1; //big-endian
#if 0
    aniAsfLogMsg(LOG_DEBUG, ANI_WHERE, " Sending pttSendEndianIndication msg of length %d to client", sizeof(endianness));

    if (pserver->clIpc)
    {
        if (aniAsfIpcSend(pserver->clIpc, (char *)&endianness, 4) < 0)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE, "%s: Could not send message to  the Client", __FUNCTION__);
        }
    }
    else
    {
        aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Client connection does not exist  dropping the msg\n", __FUNCTION__);
    }
#endif
}

char* retrieve_nv_file_name(tANI_U8 *pData)
{
#ifdef ANDROID
    char *filename = "/mnt/vendor/persist/WCN1314_qcom_wlan_nv.bin";
#else
    char *filename = "/persist/WCN1314_qcom_wlan_nv.bin";
#endif

    //typedef struct nvEFSTable_s
    //{
    //   v_U32_t    nvValidityBitmap;
    //   sHalNv     halnv;
    //} nvEFSTable_t;

    //typedef struct
    //{
    //    //always ensure fields are aligned to 32-bit boundaries
    //    tANI_U16  productId;
    //    tANI_U8   productBands;
    //    tANI_U8   wlanNvRevId; //0: WCN1312, 1: WCN1314
    //
    //    tANI_U8   numOfTxChains;
    //    tANI_U8   numOfRxChains;
    //    tANI_U8   macAddr[NV_FIELD_MAC_ADDR_SIZE];
    //
    //    tANI_U8   mfgSN[NV_FIELD_MFG_SN_SIZE];
    //}sNvFields;
    //
    //
    //typedef struct
    //{
    //    sNvFields fields;
    //    sNvTables tables;
    //}sHalNv;

    //0: WCN1312, 1: WCN1314, 2: PRIMA

    if (pData[7] == 0)
    {
        filename = "/etc/firmware/wlan/qcom_wlan_nv.bin";
    }
    else if ((pData[7] == 2) || (pData[7] == 0xCA))
    {
         aniAsfLogMsg(LOG_ERR, ANI_WHERE, "PRIMA NV File\n");
#ifdef ANDROID
         filename = "/mnt/vendor/persist/WCNSS_qcom_wlan_nv.bin";
#else
         filename = "/persist/WCNSS_qcom_wlan_nv.bin";
#endif
    }


    return filename;
}

int write_nv_items_to_efs(tANI_U8 *pData, tANI_U16 data_length)
{

    FILE *fp;
    size_t count;

    fp = fopen(retrieve_nv_file_name(pData), "wb");
    if(fp == NULL) {
        perror("failed to open sample.txt");
        return -1;
    }
    count = fwrite(pData, 1, data_length, fp);
    fclose(fp);

    printf("Wrote %zu bytes.\n", count);
    return 0;
}

/*
 * Register application PID to kernel
 */
int pttSocketAppRegisterKernel(tAniRttCtxt *pserver)
{
    unsigned char    buf[RTT_MAX_MSG_SIZE];
    int              regMsgLen = 0;
    tAniNlHdr       *wnl;
    tAniNlAppRegReq *regReq;

    if (NULL == pserver)
    {
       LOG_PSA_E("%s : Server Context invalid\n", __func__);
       return ANI_E_FAILED;
    }

    // Register with the in kernel Netlink handler
    pserver->nl.nlmsg_type = ANI_NL_MSG_PUMAC;
    pserver->nl.nlmsg_seq++;
    regMsgLen = aniNlLen(sizeof(tAniNlAppRegReq));
    pserver->nl.nlmsg_len = aniNlAlign(sizeof(tAniNlHdr)) + regMsgLen;

    // copy the netlink msg hdr first (assuming buf is 4 byte aligned)
    memcpy(buf, &pserver->nl, sizeof(struct nlmsghdr));
    wnl = (tAniNlHdr *)buf;
    wnl->radio = pserver->radio;
    // setup the tAniHdr next
    wnl->wmsg.type = htons(ANI_MSG_APP_REG_REQ);
    wnl->wmsg.length = regMsgLen;

    // align the buf and setup the tAniAppRegReq next
    regReq = (tAniNlAppRegReq *)(wnl + 1);
    regReq->pid = pserver->snl->nl_pid;

    LOG_PSA_D("Register APP Socket to kernel\n");
    return pttSocketAppRegister(0, pserver);
}

#ifdef WLAN_LOGGING_INFRA_SUPPORT
/*
 * Register application PID to kernel
 */
int pttSocketAppRegisterLoggingKernel(tAniRttCtxt *pserver)
{
    char             buf[RTT_MAX_MSG_SIZE];
    int              regMsgLen = 0;
    tAniNlHdr       *wnl;
    tAniNlAppRegReq *regReq;
    int              ret = 0;

    if (NULL == pserver)
    {
       LOG_PSA_E("%s : Server Context invalid\n", __func__);
       return ANI_E_FAILED;
    }

    // Register with the in kernel Netlink handler
    pserver->nl.nlmsg_type = ANI_NL_MSG_LOG;
    pserver->nl.nlmsg_seq++;
    regMsgLen = aniNlLen(sizeof(tAniNlAppRegReq));
    pserver->nl.nlmsg_len = aniNlAlign(sizeof(tAniNlHdr)) + regMsgLen;

    // copy the netlink msg hdr first (assuming buf is 4 byte aligned)
    memcpy(buf, &pserver->nl, sizeof(struct nlmsghdr));
    wnl = (tAniNlHdr *)buf;
    wnl->radio = pserver->radio;
    /*
     * setup the tAniHdr next
     * In general htons needs to be done here but this requires driver
     * changes as well. For now going ahead without the change
     */
    wnl->wmsg.type = PTT_WLAN_LOG_REGISTER;
    wnl->wmsg.length = regMsgLen;

    // align the buf and setup the tAniAppRegReq next
    regReq = (tAniNlAppRegReq *)(wnl + 1);
    regReq->pid = pserver->snl->nl_pid;

    LOG_PSA_D("Register APP Socket to kernel (LOGGING)\n");
    if (aniAsfIpcSend(pserver->ipcnl, buf, pserver->nl.nlmsg_len) < 0)
    {
        aniAsfLogMsg(LOG_ERR, ANI_WHERE,
                     "Failed to APP Socket to kernel (LOGGING)\n");
        LOG_PSA_D("Failed to APP Socket to kernel (LOGGING)\n");
        return ANI_E_FAILED;
    }

    return ret;
}


/*
 * Process all the WLAN Logging messages coming from the Radio Driver
 */
void pttSocketProcessWlanLoggingMessage(tAniRttCtxt *pserver, tAniNlHdr *pwnl,
                                         int msgLen)
{
    char *wlanLog;
    char *charCache;

    if (!pwnl)
    {
        LOG_PSA_E("pwnl is NULL\n");
        return;
    }

    if (pwnl->wmsg.length > msgLen)
    {
        LOG_PSA_E("Invalid WLAN LOGGING INFRA Len (%x > %x)\n",
                                          pwnl->wmsg.length, msgLen);
        return;
    }

    wlanLog = (char *)&pwnl->wmsg.length + sizeof(pwnl->wmsg.length);

    if (PTT_WLAN_LOG_MSG == pwnl->wmsg.type)
    {
#define DIAG_MSG_MAX_LEN 4096
#define DIAG_MSG_OVERHEAD_LEN 48
        // Split the message to diag acceptable size
        if (pwnl->wmsg.length > (DIAG_MSG_MAX_LEN - DIAG_MSG_OVERHEAD_LEN))
        {
            /*
             * Driver always sends logs with each message appended with '\n'
             * When buffer is about 4KB we are sure that we have multiple
             * messages in it. Try splitting first 2 messages at minimum to
             * be sure that this message is not dropped by diag due to size
             * constraint.
             */
            if (((charCache = strchr(wlanLog, '\n')) != NULL)
                 && ((charCache = strchr((charCache+1), '\n')) != NULL))
            {
                *charCache = '\0';
                WLAN_LOG_TO_DIAG(MSG_SSID_WLAN_RESERVED_10, MSG_LEGACY_MED,
                                  wlanLog);
                wlanLog = charCache++;
            }
        }

        WLAN_LOG_TO_DIAG(MSG_SSID_WLAN_RESERVED_10, MSG_LEGACY_MED, wlanLog);

    }
    else if (PTT_WLAN_LOG_READY_IND_MSG == pwnl->wmsg.type)
    {
        if(!strncmp(wlanLog, wlanLoggingReady,
                    sizeof(wlanLoggingReady)))
        {
            pttSocketAppRegisterLoggingKernel(pserver);
        }
    }
    else if (PTT_WLAN_LOG_REGISTER == pwnl->wmsg.type)
    {
        // Skip this message this will be the response for Reg req.
    }
    else
    {
        LOG_PSA_E("Invalid WLAN LOGGING INFRA Msg Type (%x)\n",
                                                pwnl->wmsg.type);
    }

    return;
}
#endif /* WLAN_LOGGING_INFRA_SUPPORT */

/*
 * Process all the messages from coming from the Radio Driver
 */
void pttSocketAppProcNetlinkMsg (void *arg)
{
    int    msgLen;
    char   *buf;
    int    pttRspLen;
    tAniNlHdr *wnl;
    tAniRttCtxt *pserver = (tAniRttCtxt *)arg;
    tAniIpc      *nIpc = pserver->ipcnl;
    int contentsLength = 0;
    tAniRttCmdRspMsg    *msg;
    unsigned char ftmCommandType;
    unsigned short pttCommandType;
    unsigned int  opCode;

    if ((buf = malloc(RTT_MAX_MSG_SIZE)) == NULL)
    {
        aniAsfLogMsg(LOG_ERR, ANI_WHERE,"pttSocketApp: %s - malloc failed\n",
                                        __FUNCTION__);
        return;
    }
    memset(buf, 0, RTT_MAX_MSG_SIZE);

    wnl = (tAniNlHdr *)buf;
    if (wnl->nlh.nlmsg_pid)
    {
        LOG_PSA_E("NL MSG is not came from kernel, discard it %d",
                  wnl->nlh.nlmsg_pid);
        free(buf);
        return;
    }

    do
    {
        if ((msgLen = aniAsfIpcRecv(nIpc, buf, RTT_MAX_MSG_SIZE)) <= 0)
        {
            aniAsfLogMsg(ANI_IPCRECV_ERR);
            LOG_PSA_E("NL MSG RCV err %d %d", msgLen, errno);
            break;
        }

#ifdef WLAN_KD_READY_NOTIFIER
        if (0 == isDriverLoaded)
        {
           LOG_PSA_D("NL BCAST MSG KDriver Not ready yet : %s\n", (char *)NLMSG_DATA(buf));
           if (!memcmp(driverLoaded, NLMSG_DATA(buf), sizeof(driverLoaded)))
           {
              LOG_PSA_D("NL BCAST MSG Received, Kernel ready, connect LN\n");
              aniAsfIpcConnect(pserver->ipcnl, "localhost", -1, -1);
              if (pttSocketAppRegisterKernel(pserver))
              {
                 LOG_PSA_E("Register into kernel fail\n");
                 break;
              }
              isDriverLoaded = 1;
           }
           break;
        }
        else
        {
           if (!memcmp(driverUnLoaded, NLMSG_DATA(buf), sizeof(driverUnLoaded)))
           {
              LOG_PSA_D("KDriver Unloaded, NL Listen\n");
              isDriverLoaded = 0;
              break;
           }
        }
#endif /* WLAN_KD_READY_NOTIFIER */

#ifdef ANI_DEBUG
        printf("pttSocketAppProcNetlinkMsg: dumping msg rcvd from netlink soc len=%d", msgLen);
        //aniDumpBuf((char *)buf, msgLen);
#endif

        aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: rcvd a NL msg, len[%d], NL type[0x%X] WNI type[0x%hX] len[%d]", __FUNCTION__, wnl->nlh.nlmsg_len, wnl->nlh.nlmsg_type, ntohs(wnl->wmsg.type), ntohs(wnl->wmsg.length));

#ifdef ANI_DEBUG
        LOG_PSA_V("NL MSG, len[%03d], NL type[0x%X] WNI type[0x%4hX] len[%03d]",
                  wnl->nlh.nlmsg_len, wnl->nlh.nlmsg_type,
                  ntohs(wnl->wmsg.type), ntohs(wnl->wmsg.length));
        LOG_PSA_V("NL MSG, PID: %d",wnl->nlh.nlmsg_pid);
#endif

      // Strip the NL hdr, attach a test debug hdr and forward all the
      // messages to the client (Perl script)
        msg = (tAniRttCmdRspMsg *) (buf + (aniNlAlign(sizeof(tAniNlHdr) - sizeof(tAniHdr) - (2*sizeof(int)))));

        if ((tANI_U32) msgLen < wnl->nlh.nlmsg_len)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE, "pttSocketApp: rcvd msg from kernel with invalid len actual %d, from nl hdr %d\n", msgLen, wnl->nlh.nlmsg_len);
            break;
        }
        msg->msgLen = ntohs(wnl->wmsg.length);
        msg->msgLen = htonl(msg->msgLen + sizeof(int));
        pttRspLen = msg->msgLen;

#ifdef WLAN_LOGGING_INFRA_SUPPORT
        if(ANI_NL_MSG_LOG == wnl->nlh.nlmsg_type)
        {
           pttSocketProcessWlanLoggingMessage(pserver, wnl, msgLen);
           break;
        }
#endif

        if((ANI_MSG_APP_REG_RSP == ntohs(wnl->wmsg.type)) || (0x00 == ntohs(wnl->wmsg.type)))
        {
           break;
        }
        aniAsfLogMsg(LOG_DEBUG, ANI_WHERE, "%s: Sending msg of length %d (msgType=0x%x) to client", __FUNCTION__, ntohl(msg->msgLen), wnl->wmsg.type);

#ifdef WLAN_LOGGING_INFRA_SUPPORT
        if(ntohs(wnl->wmsg.type) == PTT_DIAG_CMDS_TYPE)
        {
           /*Skip the netlink header 12 bytes*/
            tANI_U8 *pData = ((tANI_U8 *)msg + 12);
            tANI_U32 diag_type;

            wnl->wmsg.type = ntohs(wnl->wmsg.type);
            wnl->wmsg.length = ntohs(wnl->wmsg.length);

            diag_type = *(tANI_U32*) pData;

            pData += sizeof(tANI_U32);

            if(diag_type == PTT_DIAG_TYPE_LOGS)
            {
                log_hdr_type *pHdr = (log_hdr_type*)pData;
                if (log_status(pHdr->code))
                {
                    log_set_timestamp(pHdr);
                    log_submit(pHdr);
                }
            }
            else if(diag_type == PTT_DIAG_TYPE_EVENTS)
            {
                tANI_U16 event_id;
                tANI_U16 length;

                event_id = *(tANI_U16*)pData;
                pData += sizeof(tANI_U16);

                length = *(tANI_U16*)pData;
                pData += sizeof(tANI_U16);
                event_report_payload(event_id,length,pData);
            }
            else {
                LOG_PSA_E("Error:Invalid Diag Type!!!\n");
            }
            /* Diag command process done
             * should go out from process */
            break;
        }
#endif
        if (pserver->diag_msg.diag_msg_received == TRUE || ntohs(wnl->wmsg.type) == PTT_FTM_CMDS_TYPE)
        {
            /*Skip the netlink header 12 bytes*/
            tANI_U8 *pData = ((tANI_U8 *)msg + 12);
            wnl->wmsg.type = ntohs(wnl->wmsg.type);
            wnl->wmsg.length = ntohs(wnl->wmsg.length);

            msg->msgLen = ntohl(msg->msgLen);
            contentsLength = wnl->wmsg.length - sizeof(tAniHdr);
            if(0 >= contentsLength)
            {
               LOG_PSA_E("Invalid Contents Length %d WNI type[0x%4hX]",
                                       contentsLength, ntohs(wnl->wmsg.type));
               break;
            }
            ftmCommandType = (unsigned char)(*((char *)pData + WLAN_FTM_COMMAND_OFFSET));
            opCode = (unsigned int)(*((char *)pData + WLAN_FTM_OPCODE_OFFSET));

            /*Check whether the command code is commit to EFS*/
            LOG_PSA_V("Command Code=%d, cmd type %d, opcode %d\n",
                      *pData, ftmCommandType, opCode);
            if (*pData == 0xEF)
            {
                pData += sizeof(tANI_U32);
                LOG_PSA_V("********Writing Data to EFS*****\n");
                write_nv_items_to_efs(pData, (contentsLength - sizeof(tANI_U32)));
            }
            else
            {
                /*
                 * Diag application is droping the packet which is having 8KB limit so
                 * Dropping RSP here only instaed of passing RSP to diag application
                 */
                if(contentsLength > USER_SPACE_DATA)
                {
                   LOG_PSA_E("Dropping Response: Contents Length crosses 8KB limit: %d WNI type[0x%4hX]",
                                       contentsLength, ntohs(wnl->wmsg.type));
                   break;
                }
                pserver->diag_msg.pRespData = (char*)malloc(contentsLength);

                if(NULL == pserver->diag_msg.pRespData)
                {
                    LOG_PSA_E("Response Memory alloc fail Break");
                    break;
                }
                memcpy(pserver->diag_msg.pRespData, pData, contentsLength);
                pserver->diag_msg.msg_len = contentsLength;
            }
            break;
        }

        /* Processing only registered PTT commands */
        pttCommandType = ntohs(wnl->wmsg.type);
        if ((pserver->clIpc) &&
            ((pttCommandType == PTT_MSG_READ_REGISTER) ||
             (pttCommandType == PTT_MSG_WRITE_REGISTER) ||
             (pttCommandType == PTT_MSG_READ_MEMORY) ||
             (pttCommandType == PTT_MSG_WRITE_MEMORY) ||
             (pttCommandType == PTT_MSG_LOG_DUMP_DBG)))
        {
            if (endianness == 0) //little-endian
            {
                wnl->wmsg.type = ntohs(wnl->wmsg.type);
                wnl->wmsg.length = ntohs(wnl->wmsg.length);
                msg->msgLen = ntohl(msg->msgLen);
#ifdef ANI_DEBUG
                printf("pttSocketAppProcNetlinkMsg: dumping msg need to be sent to client len=%d", msg->msgLen);
                aniDumpBuf((char *)msg, msg->msgLen+4);
#endif
                if (wnl->wmsg.type < 0x3000)
                    aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Not sending to client msg id < 0x3000 \n", __FUNCTION__);
                else if (aniAsfIpcSend(pserver->clIpc, (char *)msg, msg->msgLen + 4/* for msgLen */) < 0)
                    aniAsfLogMsg(LOG_ERR, ANI_WHERE, "%s: Could not send message to the Client", __FUNCTION__);
            }
            else //big-endian
            {
#if 0
                if (ntohs(wnl->wmsg.type) < 0x3000)
                    aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Not sending to client msg id < 0x3000 \n", __FUNCTION__);
                else if (aniAsfIpcSend(pserver->clIpc, (char *)msg, ntohl(msg->msgLen) + 4/* for msgLen */) < 0)
                    aniAsfLogMsg(LOG_ERR, ANI_WHERE, "%s: Could not send message to the Client", __FUNCTION__);
#endif
                if (wnl->wmsg.type < 0x3000)
                    aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Not sending to client msg id < 0x3000 \n", __FUNCTION__);
                else
                {
                    wnl->wmsg.type = pttSocketSwapU16(wnl->wmsg.type);//ntohs(wnl->wmsg.type);
                    wnl->wmsg.length = pttSocketSwapU16(wnl->wmsg.length);
                    msg->msgLen = pttSocketSwapU32(msg->msgLen);
                    if (aniAsfIpcSend(pserver->clIpc, (char *)msg, pttRspLen + 4/* for msgLen */) < 0)
                        aniAsfLogMsg(LOG_ERR, ANI_WHERE, "%s: Could not send message to the Client", __FUNCTION__);
                }

            }
        }
        else
        {
            aniAsfLogMsg(LOG_INFO, ANI_WHERE, "%s: Client connection does not exist dropping the msg\n", __FUNCTION__);
        }
#ifdef ANI_DEBUG
        printf("pttSocketAppProcNetlinkMsg: dumping stripped msg rcvd from netlink soc len=%d\n", msgLen);
        //aniDumpBuf((char *)msg, pttRspLen + 4);
#endif
    } while(0);
    if(buf)
    {
       free(buf);
    }
}

void pttSocketAppProcConnInd(void *arg)
{
    tAniRttCtxt *pserver = (tAniRttCtxt *)arg;

    // Accept the incoming connection indication
    if ((pserver->clIpc = aniAsfIpcAccept(pserver->ipcs)) == NULL)
    {
        aniAsfLogMsg(ANI_IPCACCEPT_ERR);
        return;
    }

    pttSendEndianIndication(pserver);

    if (alreadyRegistered == 1)
    {
        //sendAlreadyRegisteredMessage(pserver);
        return;
    }
    /*
    * Add the newly created socket to the select list and
    * register a callback.
    */
    aniAsfIpcSetFd(pserver->clIpc, pttSocketAppProcClientMsg, (void *) pserver);


    aniAsfLogMsg(LOG_INFO, ANI_WHERE,"pttSocketApp: Client Connection Accepted\n");

    LOG_PSA_D("Client Connection Accepted\n");
    /* Connected publick socket client should issue connection */
    aniAsfIpcConnect(pserver->ipcnl, "localhost", -1, -1);
    pttSocketAppRegister(0, pserver);
    aniAsfLogMsg(LOG_INFO, ANI_WHERE, "pttSocketApp: pttSocketAppRegister succeeded\n");

    alreadyRegistered = 1;


    return;
}

void pttSocketAppCleanup (tAniIpc *ipcs, tAniIpc *clipc, tAniIpc *ipcnl)
{
    if (clipc)
    {
        aniAsfIpcUnSetFd(clipc);
        aniAsfIpcClose(clipc);
    }

    if (ipcs && (is_ffbm_mode == 1))
    {
        aniAsfIpcUnSetFd(ipcs);
        aniAsfIpcClose(ipcs);
    }

    if (ipcnl)
    {
        aniAsfIpcUnSetFd(ipcnl);
        aniAsfIpcClose(ipcnl);
    }
}

void pttSocketAppSigInt(int signum)
{
    UNUSED(signum);
    sigInt = 1;
    printf("pttSocketApp: Caught a SIGINT, exiting\n");
}

void pttSocketAppSigIntrInit(void)
{
    struct sigaction act;

    // Initialize the sigaction structure
    memset(&act, 0, sizeof(struct sigaction));
    act.sa_handler = &pttSocketAppSigInt;
    sigemptyset(&act.sa_mask);
    act.sa_flags = SA_RESTART;

    // Register the call back function for the SIGALRM signal
    sigaction(SIGINT, &act, NULL);
}

int pttSocketAppInit(int radio, tAniRttCtxt *pserver)
{
    int ret = 0;

    do
    {
        if (!pserver)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Invalid pserver passed in\n");
            ret = ANI_E_FAILED;
            break;
        }

        if (radio && radio >=2)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Invalid radio id [%d] passed in\n", radio);
            ret = ANI_E_FAILED;
            break;
        }
        aniAsfLogMsg(LOG_INFO, ANI_WHERE, "RADIO id [%d] passed in\n", radio);
#ifdef ANI_DEBUG
        printf("RADIO id [%d] passed in\n", radio);
#endif

        pserver->radio = radio;
        if (is_ffbm_mode == 1)
        {
            // Open a socket to listen for client connections

            if ((pserver->ipcs = aniAsfIpcOpen(AF_INET, SOCK_STREAM,
                                               RTT_SERVER_PORT)) == NULL)
            {
                LOG_PSA_E("failed i_net socket open error:%s", strerror(errno));
                aniAsfLogMsg(ANI_IPCOPEN_ERR);
                ret = ANI_E_FAILED;
                break;
            }

#ifdef ASF_PORT_MAP_SUPPORTED
            if (aniAsfIpcListen(pserver->ipcs, -1, -1) < 0)
#else
            if (aniAsfIpcListen(pserver->ipcs, -1) < 0)
#endif
            {
                aniAsfLogMsg(ANI_IPCLISTEN_ERR);
                ret = ANI_E_FAILED;
                break;
            }

            // Register a callback for the Server socket
            aniAsfIpcSetFd(pserver->ipcs, pttSocketAppProcConnInd, pserver);
       }

        // Open Netlink Socket to Pseudo Driver
#ifdef WLAN_KD_READY_NOTIFIER
        if ((pserver->ipcnl = aniAsfIpcOpen(AF_NETLINK, SOCK_DGRAM, 1)) == NULL)
#else
        if ((pserver->ipcnl = aniAsfIpcOpen(AF_NETLINK, SOCK_DGRAM, 0)) == NULL)
#endif /* WLAN_KD_READY_NOTIFIER */
        {
            aniAsfLogMsg(ANI_IPCOPEN_ERR);
            pttSocketAppCleanup(pserver->ipcs, pserver->clIpc, pserver->ipcnl);
            ret = ANI_E_FAILED;
            break;
        }

#ifdef WLAN_KD_READY_NOTIFIER
#ifdef ASF_PORT_MAP_SUPPORTED
        aniAsfIpcListen(pserver->ipcnl, -1, -1);
#else
        aniAsfIpcListen(pserver->ipcnl, -1);
#endif
#else
        aniAsfIpcConnect(pserver->ipcnl, "localhost", -1, -1);
#endif /* WLAN_KD_READY_NOTIFIER */
        /*
        * Init a pre allocated Netlink msg hdr that we will use
        * later on while relaying messages coming in from the client
        * to the Radio Driver.  The nlmsg->type and nlmsg->len
        * parameters are filled in later on for each message as
        * appropriate.
        */
        pserver->nl.nlmsg_seq = 0;

        /*
        * Get the sockaddr_nl structure from ASF for this tAniIpc
        */
        pserver->snl = aniAsfIpcGetSnl(pserver->ipcnl);
        if (!pserver->snl)
        {
            ret = ANI_E_FAILED;
            break;
        }
        pserver->nl.nlmsg_pid = pserver->snl->nl_pid;
        pserver->nl.nlmsg_flags = NLM_F_REQUEST;

        // Register a callback for the Netlink fd
        aniAsfIpcSetFd(pserver->ipcnl, pttSocketAppProcNetlinkMsg, pserver);

#ifndef WLAN_KD_READY_NOTIFIER
        pttSocketAppRegisterKernel(pserver);
#endif /* WLAN_KD_READY_NOTIFIER */
    } while (0);

    return ret;
}

int pttSocketAppRegister(int radio, tAniRttCtxt *pserver)
{
    char   buf[RTT_MAX_MSG_SIZE];
    int ret = 0;
    int regMsgLen = 0;
    tAniNlAppRegReq *regReq;
    tAniNlHdr *wnl;


    do
    {
        if (!pserver)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE,"Invalid pserver passed in\n");
            ret = ANI_E_FAILED;
            break;
        }

        if (radio && radio >=2)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Invalid radio id [%d] passed in\n", radio);
            ret = ANI_E_FAILED;
            break;
        }
        aniAsfLogMsg(LOG_INFO, ANI_WHERE,"RADIO id [%d] passed in\n", radio);
#ifdef ANI_DEBUG
        printf("RADIO id [%d] passed in\n", radio);
#endif

        pserver->radio = radio;


        // Register with the in kernel Netlink handler
        pserver->nl.nlmsg_type = ANI_NL_MSG_PUMAC;
        pserver->nl.nlmsg_seq++;
        regMsgLen = aniNlLen(sizeof(tAniNlAppRegReq));
        pserver->nl.nlmsg_len = aniNlAlign(sizeof(tAniNlHdr)) + regMsgLen;

        // copy the netlink msg hdr first (assuming buf is 4 byte aligned)
        memcpy(buf, &pserver->nl, sizeof(struct nlmsghdr));

        wnl = (tAniNlHdr *)buf;
        wnl->radio = pserver->radio;

        // setup the tAniHdr next
        wnl->wmsg.type = htons(ANI_MSG_APP_REG_REQ);
        wnl->wmsg.length = regMsgLen;

        // align the buf and setup the tAniAppRegReq next
        regReq = (tAniNlAppRegReq *)(wnl + 1);
        regReq->pid = pserver->snl->nl_pid;

        // register for messages of type ANI_NL_MSG_PUMAC
        regReq->type = ANI_NL_MSG_PUMAC;

#ifdef ANI_DEBUG
        printf("pttSocketAppInit: sending register ANI_NL_MSG_PUMAC 0x%x msg to netlink soc len=%d", ANI_NL_MSG_PUMAC, pserver->nl.nlmsg_len);
        aniDumpBuf((char *)buf, pserver->nl.nlmsg_len);
#endif


        if (aniAsfIpcSend(pserver->ipcnl, buf, pserver->nl.nlmsg_len) < 0)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Failed to register for ANI_NL_MSG_PUMAC msgs");
            pttSocketAppCleanup(pserver->ipcs, pserver->clIpc, pserver->ipcnl);
            ret = ANI_E_FAILED;
            break;
        }

        pserver->nl.nlmsg_seq++;
        regReq->type =ANI_NL_MSG_PTT;
#ifdef ANI_DEBUG
        printf("pttSocketAppInit: sending register ANI_NL_MSG_PTT 0x%x msg to netlink soc len=%d", ANI_NL_MSG_PTT, pserver->nl.nlmsg_len);
        aniDumpBuf((char *)buf, pserver->nl.nlmsg_len);
#endif
      // also register for message of type ANI_NL_MSG_MAC_SW
        if (aniAsfIpcSend(pserver->ipcnl, buf, pserver->nl.nlmsg_len) < 0)
        {
            aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Failed to register for ANI_NL_MSG_MAC_SW msgs");
            pttSocketAppCleanup(pserver->ipcs, pserver->clIpc, pserver->ipcnl);
            ret = ANI_E_FAILED;
            break;
        }
    } while (0);

    return ret;
}

void *wlan_ftm_func_75(void *req_pkt, uint16 pkt_len)
{
    void *rsp = NULL;
    tAniRttCtxt *pserver = &serverCtxt;
    char *pBuf = NULL;
    tAniHdr *msg;
    int msgLen;
    unsigned long retry_count = 0xFFFFFFFF;
    int nlmsgType = ANI_NL_MSG_PTT;
#ifdef WLAN_KD_READY_NOTIFIER
    unsigned char ftmCommandType;
    unsigned int  opCode;
#endif /* WLAN_KD_READY_NOTIFIER */
    printf("\n ##### wlan FTM Test App: : Inside wlan_ftm_func_75 #####\n");

    printf("\n ##### Pak_len=%d:: Inside wlan_ftm_func_75 #####\n",pkt_len);

    msgLen = (pkt_len + sizeof(tAniHdr));

    pBuf = (char*)malloc(msgLen);

    if (!pBuf)
    {
        printf("\n ##### malloc failed***");
        return NULL;
    }

    pserver->diag_msg.diag_msg_received = TRUE;

    memset(pBuf, 0, msgLen);

    msg = (tAniHdr*)pBuf;
    msg->length = pkt_len;
    msg->type = PTT_FTM_CMDS_TYPE;

    memcpy((pBuf + sizeof(tAniHdr)), req_pkt, pkt_len);

#ifdef WLAN_KD_READY_NOTIFIER
    ftmCommandType = (unsigned char)(*((char *)req_pkt + WLAN_FTM_COMMAND_OFFSET));
    LOG_PSA_V("ftmCommandType %d\n", ftmCommandType);
    if ((1 == ftmCommandType) && (is_ffbm_mode))
    {
       /* insmod wlan.ko driver */
       LOG_PSA_V("WLAN FTM_Start Command issued\n");
       system("rmmod wlan.ko");
       usleep(100000);
#ifdef VENDOR_MOVE_ENABLED
       system("insmod /vendor/lib/modules/wlan.ko con_mode=5");
#else
       system("insmod /system/lib/modules/wlan.ko con_mode=5");
#endif /* VENDOR_MOVE_ENABLED */
       usleep(100000);
    }
    if ((1 == ftmCommandType) && (!is_ffbm_mode))
    {
        /* insmod wlan.ko driver */
        LOG_PSA_E("NOT FFBM mode, start FTM driver without load KO");
    }
    else if (2 == ftmCommandType)
    {
       LOG_PSA_V("WLAN FTM_Stop Command issued\n");
    }
    else if (3 == ftmCommandType)
    {
       opCode = (unsigned int)(*((char *)req_pkt + WLAN_FTM_OPCODE_OFFSET));
       LOG_PSA_V("WLAN FTM_OP Command issued OPCODE %d\n", opCode);
    }
#endif /* WLAN_KD_READY_NOTIFIER */

    if (pttSocketAppSendMsgToKernel(pserver, pserver->radio, msg, nlmsgType, msgLen) < 0)
    {
        aniAsfLogMsg(LOG_ERR, ANI_WHERE, "pttSocketApp: Could not send data to the HDD\n");
    }
    /*Poll here untill we get the response from the ftm driver*/
    do {
        usleep(10);
        if((pserver->diag_msg.pRespData != NULL) && (pserver->diag_msg.msg_len !=0)) {
            break;
        }
        retry_count--;

    } while(retry_count);

    if(!retry_count) {
        printf("FTM driver response time out\n");
        pserver->diag_msg.diag_msg_received = FALSE;
        free(pBuf);
        return NULL;
    }

    /* Allocate the same length as the request. */
    rsp = diagpkt_alloc(WLAN_FTM_SUBSYS_TEST_CLIENT, pserver->diag_msg.msg_len);

    if (rsp != NULL) {
        memcpy((void *) rsp, (void *) pserver->diag_msg.pRespData, pserver->diag_msg.msg_len);

        free(pserver->diag_msg.pRespData);
        pserver->diag_msg.pRespData = NULL;
        pserver->diag_msg.msg_len=0;
        pserver->diag_msg.diag_msg_received = FALSE;
        printf("Wlan FTM Test APP: diagpkt_alloc succeeded\n");

    } else {
        printf("Wlan FTM Test APP: diagpkt_subsys_alloc failed");
    }

    free(pBuf);

    return rsp;
}


/**
 *    Main function for the NetSim Server daemon
 *
 *    Supports two options
 *       -d - not to daemonize
 *       -v - Set Max log level
 *
 */
int main (int argc, char *argv[])
{
    int   radio = 0;
    int   c;
    int   nodaemon = 0;
    boolean bInit_Success = FALSE;

    // Initialize aniAsfLogInit
    aniAsfLogInit("pttSocketApp", LOG_ERR, ANI_CONS_LOG);

    LOG_PSA_D("PTT_SOCKET_APP Start\n");
    while ((c = getopt(argc, argv, "drvnf")) != EOF)
    {
        switch(c) {
            case 'd':
                nodaemon = 1;
                break;
            case 'v':
                if (optind > 0 && optind < argc)
                {
                    debug = 1;
                    aniAsfLogSetLevel(atoi(argv[optind]));
                }
                break;
            case 'r':
                if (optind > 0 && optind < argc)
                {
                    radio = atoi(argv[optind]);
                }
                break;
            case 'n':
                aniAsfLogMsg(LOG_INFO, ANI_WHERE, "-n option set, will not pass any msg to netlink");
                pass = 0;
                break;
            case 'f':
                LOG_PSA_E("FFBM mode");
                is_ffbm_mode = 1;
                break;
        }
    }

    if (!nodaemon)
    {
        LOG_PSA_E("Daemonize");
        aniAsfDaemonize();
    }
    pttSocketAppSigIntrInit();

    // Init local datastructures
    if ((pttSocketAppInit(radio, &serverCtxt)) < 0)
    {
        aniAsfLogMsg(LOG_ERR, ANI_WHERE, "%s: aniNetsimInit failed\n", argv[0]);
        exit(1);
    }

    /* Blocked Operation */
    LOG_PSA_E("Set as Blocked Operation");
    aniAsfIpcBlockSelect();

    /* Calling LSM init  */
    bInit_Success = Diag_LSM_Init(NULL);

    if (!bInit_Success) {
        printf("FTM Test App: Diag_LSM_Init() failed.");
        exit(1);
    }

    LOG_PSA_D("FTM Test App: Diag_LSM_Init succeeded. \n");
    /* Registering diag packet with no subsystem id. This is so
        * that an empty request to the app. gets a response back
        * and we can ensure that the diag is working as well as the app. is
        * responding subsys id = 11, table = test_tbl_2,
        * To execute on QXDM :: "send_data 75 11 0 0 0 0 0 0"
           OR
        * To execute on QXDM :: "send_data 75 11 22 0 0 0 0 0"
        */
    serverCtxt.diag_msg.diag_msg_received = FALSE;

    //DIAGPKT_DISPATCH_TABLE_REGISTER_V2_DELAY(75, WLAN_FTM_SUBSYS_TEST_CLIENT, wlan_ftm_test_tbl); //send_data 128 69 2 0 0 0 0 0 0
    DIAGPKT_DISPATCH_TABLE_REGISTER(WLAN_FTM_SUBSYS_TEST_CLIENT,wlan_ftm_test_tbl);

    LOG_PSA_D("PTT_SOCKET_APP Init done, wait clients\n");
    // Loop for ever
    while (1)
    {
        aniAsfIpcProcess();
        if (sigInt)
        {
            break;
        }
    }

    pttSocketAppCleanup(serverCtxt.ipcs, serverCtxt.clIpc, serverCtxt.ipcnl);

    Diag_LSM_DeInit();

    return 0;
}
