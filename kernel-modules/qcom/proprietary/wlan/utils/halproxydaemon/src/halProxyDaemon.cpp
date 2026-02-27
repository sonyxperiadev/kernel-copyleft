/*
* Copyright (c) 2014-2019, 2022 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#include "halLog.hpp"
#ifndef ANDROID_WEAR
#include "nan_test.hpp"
#endif
#include "gscan_test.hpp"
#include "llstats_test.hpp"
#include "rtt_test.hpp"
#include "tdls_test.hpp"
#include "wifihal_test.hpp"
#include "wificonfig_test.hpp"
#include "wifi_hal.h"
#include "wifilogger_test.hpp"
#include "wifi_calling_test.hpp"
#include "packet_filtering.hpp"
#include "common.hpp"
#include "nd_offload.hpp"
#include "roam_test.hpp"
#include "wifihal_vendor_test.hpp"

#include <netinet/in.h>
#include <sys/select.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <poll.h>
#include <unistd.h>
#include <vector>

#undef LOG_TAG
#define LOG_TAG "HalProxyDaemon"

#define UDP_PORT 51000
#define GID_WIFI 1010

wifi_handle gbl_handle;

#ifndef HAL_PROXY_DAEMON_VER
#define HAL_PROXY_DAEMON_VER "1.0.0"
#endif /* HAL_PROXY_DAEMON_VER */

wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);
#define SERVER "127.0.0.1"
#define BUFLEN 512  /* Max length of buffer */
#define PORT 8888   /* The port on which to send data */
struct sockaddr_in si;
int app_sock, s2, i, slen=sizeof(si);
char buf[BUFLEN];
char message[BUFLEN];
int glb_total_iter,glb_curr_iter,glb_sleep_time;
std::vector <int> glb_distance_mm;


namespace HAL
{
    class HalProxyDaemon
    {
    public:
        HalProxyDaemon(
            int argc,
            char** argv);

        ~HalProxyDaemon();

        inline bool
        isGood() const
            {
                return isGood_;
            }

        inline wifi_handle
        getWifiHandle()
            {
                return wifiHandle_;
            }
        int
        operator()();

    private:
        enum MsgTypes
        {
            /* Control messages */
            MSG_TYPE_ERROR_RSP         = 0,
            MSG_TYPE_DUMP_STATS_REQ    = 1,
            MSG_TYPE_DUMP_STATS_RSP    = 2,
            MSG_TYPE_CLEAR_STATS_REQ   = 3,
            MSG_TYPE_CLEAR_STATS_RSP   = 4,
            MSG_TYPE_REGISTER_REQ      = 5,
            MSG_TYPE_REGISTER_RSP      = 6,
            MSG_TYPE_DEREGISTER_REQ    = 7,
            MSG_TYPE_DEREGISTER_RSP    = 8,

            /* Data messages */
            MSG_TYPE_RAW_REQ           = 9,
            MSG_TYPE_EVENT             = 10
        };

        void
        hexdump(
            uint8_t* data,
            size_t len);

        void
        clearStats();

        void
        usage() const;

        int
        parseOptions(
            int& argc,
            char** argv);

        const char* program_;
        bool isGood_;

        wifi_handle wifiHandle_;

        struct Stats_t {
            uint32_t numUdpRawReqRcvd_;
            uint32_t numUdpMsgSent_;

            uint32_t numSuppRawReqSent_;
            uint32_t numSuppMsgRcvd_;
        } stats_;

        /* Not thread safe. */
        static const size_t MAX_BUF_SIZE = 2048;
        static char ipcMsgData_[ MAX_BUF_SIZE ];
    };

    /* We do a one-time allocation of this so we don't chew up stack
     * space for this.
     */
    char HalProxyDaemon::ipcMsgData_[
        MAX_BUF_SIZE ];

    HalProxyDaemon::HalProxyDaemon(
        int argc,
        char** argv) :
        program_(argv[0]),
        isGood_(false),
        wifiHandle_(NULL),
        stats_()
    {
        if (parseOptions(argc, argv) < 0)
        {
            HAL_LOG_MSG(
                HAL_LOG_LEVEL_ERROR,
                "parseOptions() failed");
            fprintf(stderr, "parseOptions() failed. \n");
            return;
        }
        clearStats();

        wifi_error err = wifi_initialize(&wifiHandle_);
        if (err)
        {
            HAL_LOG_MSG(
                HAL_LOG_LEVEL_ERROR,
                "wifi hal initialize failed");
            fprintf(stderr, "wifi hal initialize failed. \n");
            return;
        }

        HAL_LOG_MSG(
            HAL_LOG_LEVEL_WARN,
            "halProxyDaemon running");
        fprintf(stderr, "halProxyDaemon running. \n");

        isGood_ = true;
    }

    HalProxyDaemon::~HalProxyDaemon()
    {
    }


    void
    HalProxyDaemon::usage() const
    {
        fprintf(stderr, "Usage: %s [-option]\n", program_);
        fprintf(stderr, " -h                Display help\n");
        fprintf(stderr, "\n");
    }

    int
    HalProxyDaemon::parseOptions(
        int& argc,
        char** argv)
    {
        UNUSED(argc), UNUSED(argv);
        return 0;
    }


    int
    HalProxyDaemon::operator()()
    {
        return 0;
    }

    void
    HalProxyDaemon::hexdump(
        uint8_t* data,
        size_t len)
    {
        char buf[512];
        uint16_t index;
        uint8_t* ptr;
        int pos;

        ptr = data;
        pos = 0;
        for (index=0; index<len; index++)
        {
            pos += snprintf(&(buf[pos]), sizeof(buf) - pos, "%02x ", *ptr++);
            if (pos > 508)
            {
                break;
            }

        }

        HAL_LOG_MSG(
            HAL_LOG_LEVEL_ERROR,
            "HEX DUMP len="
            << len
            << "["
            << buf
            << "]");
    }

    void
    HalProxyDaemon::clearStats()
    {
        memset(&stats_, 0, sizeof(stats_));
    }

}

void app_process_event_handler(void * buf, int len)
{
    char *data = (char *)buf;
    int i=0;

    printf("######################################\n");
    for(i=0; i<len; i++)
    {
        printf("%c", data[i]);
    }
    printf("\n######################################\n");
}

static int app_internal_pollin_handler(wifi_handle handle)
{
    UNUSED(handle);
    memset(buf,'\0', BUFLEN);
    /* Try to receive some data, this is a blocking call */
    /* read datagram from client socket */
    if (recvfrom(app_sock, buf, BUFLEN, 0, (struct sockaddr *) &si,
        (socklen_t*)&slen) == -1)
    {
        ALOGE("Error recvfrom");
    }
    app_process_event_handler(buf, slen);
    return 0;
}

static void app_internal_event_handler(wifi_handle handle, int events)
{
    if (events & POLLERR) {
        ALOGE("Error reading from socket");
    } else if (events & POLLHUP) {
        ALOGE("Remote side hung up");
    } else if (events & POLLIN) {
        ALOGI("Found some events!!!");
        app_internal_pollin_handler(handle);
    } else {
        ALOGE("Unknown event - %0x", events);
    }
}
void app_event_loop(wifi_handle handle)
{
    pollfd pfd;
    memset(&pfd, 0, sizeof(pfd));

    pfd.fd = app_sock;
    pfd.events = POLLIN;

    do {
        pfd.revents = 0;
        ALOGI("Polling socket");
        int result = poll(&pfd, 1, -1);
        ALOGI("Poll result = %0x", result);
        if (result < 0) {
            ALOGE("Error polling socket");
        } else if (pfd.revents & (POLLIN | POLLHUP | POLLERR)) {
            app_internal_event_handler(handle, pfd.revents);
        }
    } while (1);
}

void * my_app_thread_function (void *ptr) {
    UNUSED(ptr);
    app_event_loop(gbl_handle);
    pthread_exit(0);
    return (void *)NULL;
}

void * my_thread_function (void *ptr) {
    UNUSED(ptr);
    wifi_event_loop(gbl_handle);
    pthread_exit(0);
    return (void *)NULL;
}

s8 char_to_hex(char c)
{
    s8 num = -1;

    if (c >= '0' && c <= '9')
       num = c - '0';
    else if (c >= 'a' && c <= 'f')
        num = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F')
        num = c - 'A' + 10;
    else
        fprintf(stderr, "Not a valid hex char\n");
    return num;
}

/*Convert string to MAC address
* @txt: [IN] MAC address as a string
* @mac_addr: [OUT] Buffer for the MAC address
* @lenght: [IN] lenght of mac address
*/
int mac_addr_aton(u8 *mac_addr, const char *txt, size_t length)
{
    size_t str_len;
    size_t i;

    str_len = (length * 2) + length - 1;
    if (strlen(txt) != str_len) {
        fprintf(stderr, "Invalid MAC Address\n");
        return -1;
     }

    for (i = 0; i < length; i++) {
        s8 a, b;
        a = char_to_hex(*txt++);
        if (a < 0) {
            fprintf(stderr, "Invalid Mac Address \n");
            return -1;
        }
        b = char_to_hex(*txt++);
        if (b < 0) {
            fprintf(stderr, "Invalid Mac Address \n");
            return -1;
        }
        mac_addr[i] = (a << 4) | b;
        if (i < (length - 1) && *txt++ != ':') {
            fprintf(stderr, "Invalid Mac Address \n");
            return -1;
        }
    }
    return 0;
}

int mac_addr_aton(u8 *mac_addr, const char *txt)
{
    return mac_addr_aton(mac_addr, txt, MAC_ADDR_LEN);
}

void cleanup_handler(wifi_handle handle)
{
    fprintf(stderr, "Cleanup done for handle : %p\n", handle);
}

/*Fetch the total-number-of-iterations argument from argv.
i is 4
*/
int get_total_iter_index(int argc, char *argv[], int i)
{
    int total_iter_index = 0;
    int cmdId;

    if (i < argc)
        cmdId = atoi(argv[i-1]);
    else
        return 0;

    switch(cmdId) {
        case 10:
            /* Move i by: "<mac_oui>" */
            total_iter_index = i + 1;
            break;
        case 11:
        case 12:
        case 13:
        case 15:
            /* i is alredy pointing to total iterations */
            total_iter_index = i;
            break;
        default:
            fprintf(stderr, "%s: Unknown input.\n", __func__);
    }
    if (total_iter_index && total_iter_index < argc)
        return total_iter_index;

    /* Return 0 means, somthing went wrong in parsing the arguments */
    return 0;
}

/* Validate sleep duration between commands */
int validate_sleep_duration(int sleep_time)
{
    if (sleep_time > MAX_SLEEP_DURATION) {
        fprintf(stderr, "Maximum sleep duration allowed is %d Seconds\n",
                MAX_SLEEP_DURATION);
        return -1;
    }
    return 0;
}

/* Validate sleep duration between commands */
int validate_rtt_sleep_duration(int sleep_time)
{
    if (sleep_time > RTT_MAX_SLEEP_DURATION) {
        fprintf(stderr, "Maximum sleep duration allowed is %d Seconds\n",
                RTT_MAX_SLEEP_DURATION);
        return -1;
    }
    return 0;
}

/* Validate interation count */
int validate_iterations_count(int iter_count)
{
    if (iter_count > MAX_ITERATIONS) {
        fprintf(stderr, "Maximum iterations count is %d\n", MAX_ITERATIONS);
        return -1;
    }
    return 0;
}


/* Validate interation count */
int validate_rtt_iterations_count(int iter_count)
{
    if (iter_count > RTT_MAX_ITERATIONS) {
        fprintf(stderr, "Maximum iterations count is %d\n", RTT_MAX_ITERATIONS);
        return -1;
    }
    return 0;
}

wifi_hal_fn fn;
int
main(
    int argc,
    char* argv[])
{

    int request_id;
    int cmdId = -1;
    int max = 0;
    int flush = 0;
    int band = 0;
    unsigned int sleep_time;
    // to interact with lowi-server over dgram socket,
    // halproxydaemon should be of wifi group
    setgid(GID_WIFI);

    HAL::HalProxyDaemon halProxyDaemon(argc, argv);

    if (!halProxyDaemon.isGood())
    {
        fprintf(stderr, "%s: halProxyDaemon failed at startup.\n", argv[0]);
        ::exit(-1);
    }

    wifi_handle wifiHandle = halProxyDaemon.getWifiHandle();
    if (!wifiHandle)
    {
        fprintf(stderr, "%s: halProxyDaemon invalid wifi handle .\n", argv[0]);
        ::exit(-1);
        return -1;
    }
    gbl_handle = wifiHandle;

    pthread_t thread1;  /* thread variables */
    pthread_t thread2;  /* thread variables */

    /* Create an app socket */
    if ( (app_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        fprintf(stderr, "%s: halProxy failed cause the app socket is not created.\n", __func__);
        ::exit(-1);
    }

    /* create threads 1 */
    pthread_create (&thread1, NULL, &my_thread_function, NULL);
    /* create threads 2 */
    pthread_create (&thread2, NULL, &my_app_thread_function, NULL);

    wifi_interface_handle ifaceHandle = wifi_get_iface_handle(wifiHandle,
                                                             (char*)"wlan0");

    if (!ifaceHandle)
    {
        fprintf(stderr, "%s: halProxy failed cause it couldn't retrieve iface ptrs.\n", __func__);
        ::exit(-1);
        return -1;
    }

    if(init_wifi_vendor_hal_func_table(&fn))
    {
        fprintf(stderr, "Failed to initialize the function table");
        ::exit(-1);
        return -1;
    }
    if(argc >= 2)
    {
        fprintf(stderr, "%s: Version: " HAL_PROXY_DAEMON_VER "\n", argv[0]);
#ifndef ANDROID_WEAR
        if(strcasecmp(argv[1], NAN_TEST::NanTestSuite::NAN_CMD) == 0)
        {
            NAN_TEST::NanTestSuite nan(wifiHandle);
            nan.processCmd(argc, argv);
        }
        else
#endif
        if(strcasecmp(argv[1], LLStats::LLStatsTestSuite::LL_CMD) == 0)
        {
            LLStats::LLStatsTestSuite llstats(wifiHandle);
            return llstats.processCmd(argc, argv);
        }
        else if(strcasecmp(argv[1], NUDStats::NUDStatsTestSuite::NUD_CMD) == 0)
        {
            NUDStats::NUDStatsTestSuite nudstats(wifiHandle);
            return nudstats.processCmd(argc, argv);
        }
        else if(strcasecmp(argv[1], GSCAN_TEST::GScanTestSuite::GSCAN_CMD) == 0)
        {
            srand(time(NULL));
            request_id = rand();
            GSCAN_TEST::GScanTestSuite gscan(
                            wifiHandle,
                            request_id);
            /*command line support*/
            if (argc > 3) {
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Enter ID for GSCAN Cmd, as follows:\n");
                fprintf(stderr, "GSCAN Start                            : 1\n");
                fprintf(stderr, "GSCAN Stop                             : 2\n");
                fprintf(stderr, "GSCAN Get Valid Channels               :"
                                " 3 max_ch band iterations interval\n");
                fprintf(stderr, "GSCAN Get Capabilities                 :"
                                " 4 iterations interval\n");
                fprintf(stderr, "GSCAN Get Cached Results               :"
                                " 5 max_res flush iterations interval\n");
                fprintf(stderr, "GSCAN Set BSSID Hotlist                :"
                                " 6 iterations interval\n");
                fprintf(stderr, "GSCAN Reset BSSID Hotlist              :"
                                " 7 iterations interval\n");
                fprintf(stderr, "GSCAN Set Significant change           :"
                                " 8 iterations interval\n");
                fprintf(stderr, "GSCAN Reset Significant change         :"
                                " 9 iterations interval\n");
                fprintf(stderr, "GSCAN Set MAC OUI                      :"
                                " 10 XX:XX:XX iterations interval\n");
                fprintf(stderr, "GSCAN Set ePNO LIST                    :"
                                " 11 iterations interval\n");
                fprintf(stderr, "GSCAN Set Passpoint List               :"
                                " 12 iterations interval\n");
                fprintf(stderr, "GSCAN Reset Passpoint List             :"
                                " 13 iterations interval\n");
                fprintf(stderr, "CMD 14 is reserved\n");
                fprintf(stderr, "GSCAN ReSet ePNO LIST                  :"
                                " 15 iterations interval\n");
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "######################\n");
                fprintf(stderr, "interval - time delay between current "
                                "command and next commnad(may be same cmd) "
                                "in seconds\n");
                fprintf(stderr, "iterations - No.of times the command to run\n");
                fprintf(stderr, "######################\n");
                int temp = 1;
                /* Set request Id for GSCAN object. */
                gscan.setRequestId(temp);

                int i = ARGV_CMDID_INDEX, iter = 0, total_iter = 0;
                int total_iter_idx = 0;
                while(i<argc){
                    oui scan_oui = {0};
                    cmdId = atoi(argv[i++]);
                    fprintf(stderr, "cmd : %d\n", cmdId);
                    if(cmdId < 0 || cmdId > 15 || cmdId == 14)
                    {
                        fprintf(stderr, "Unknown command\n");
                        break;
                    }
                    switch(cmdId) {
                        case 3:
                            if(argc <= i+3)
                            {
                                fprintf(stderr, "Insufficient Args for cmdId :"
                                        " %d\n", cmdId);
                                return -1;
                            }
                            max = atoi(argv[i++]);
                            fprintf(stderr, "Max number of channels : %d\n", max);
                            band = atoi(argv[i++]);
                            fprintf(stderr, "Channels band : %d\n", band);
                            total_iter = atoi(argv[i]);
                            if (validate_iterations_count(total_iter))
                                return -1;
                            for (iter = 0; iter < total_iter; iter++) {
                                gscan.executeCmd(argc, argv, cmdId, max, flush,
                                                 band, scan_oui);
                                if(total_iter >= 1)
                                {
                                    sleep_time = atoi(argv[i+1]);
                                    if (validate_sleep_duration(sleep_time))
                                        return -1;
                                    sleep(sleep_time);
                                }
                            }
                            i += 2;
                            break;
                        case 5:
                            if(argc <= i+3)
                            {
                                fprintf(stderr, "Insufficient Args for cmdId :"
                                        " %d\n", cmdId);
                                return -1;
                            }
                            max = atoi(argv[i++]);
                            fprintf(stderr, "Max number of cached"
                                    " results: %d\n" , max);
                            flush = atoi(argv[i++]);
                            fprintf(stderr, "flush : %d\n", flush);
                            if(argc <= i+1)
                            {
                                fprintf(stderr, "Insufficient Args for cmdId :"
                                        " %d\n", cmdId);
                                return -1;
                            }
                            fprintf(stderr, "Issuing cmd : %d \n", cmdId);
                            total_iter = atoi(argv[i]);
                            if (validate_iterations_count(total_iter))
                                return -1;
                            for (iter = 0; iter < total_iter; iter++) {
                                gscan.executeCmd(argc, argv, cmdId, max, flush,
                                                 band, scan_oui);
                                if(total_iter >= 1 && atoi(argv[i+1]))
                                {
                                    sleep_time = atoi(argv[i+1]);
                                    if (validate_sleep_duration(sleep_time))
                                        return -1;
                                    sleep(sleep_time);
                                }
                            }
                            i += 2;
                            break;
                        case 4:
                        case 6:
                        case 7:
                        case 8:
                        case 9:
                            if(argc <= i+1)
                            {
                                fprintf(stderr, "Insufficient Args for cmdId :"
                                        " %d\n", cmdId);
                                return -1;
                            }
                            fprintf(stderr, "Issuing cmd : %d \n", cmdId);
                            total_iter = atoi(argv[i]);
                            if (validate_iterations_count(total_iter))
                                return -1;
                            for (iter = 0; iter < total_iter; iter++) {
                                gscan.executeCmd(argc, argv, cmdId, max, flush,
                                                 band, scan_oui);
                                if(total_iter >= 1 && atoi(argv[i+1]))
                                {
                                    sleep_time = atoi(argv[i+1]);
                                    if (validate_sleep_duration(sleep_time))
                                        return -1;
                                    sleep(sleep_time);
                                }
                            }
                            i += 2;
                            break;
                        case 1:
                        case 2:
                            gscan.executeCmd(argc, argv, cmdId, max, flush, band,
                                             scan_oui);
                            break;
                        case 10:
                        case 11:
                        case 12:
                        case 13:
                        case 15:
                            total_iter_idx = get_total_iter_index(argc, argv, i);
                            /*If get_total_iter_index() returns 0 means,
                             *something went wrong in parsing the arguments*/
                            if (!total_iter_idx) {
                                fprintf(stderr, "Insufficient Args for cmdId :"
                                        " %d\n", cmdId);
                                return -1;
                            }
                            total_iter = atoi(argv[total_iter_idx]);
                            if (validate_iterations_count(total_iter))
                                return -1;
                            for (iter = 0; iter < total_iter; iter++) {
                                fprintf(stderr, "Issuing cmd : %d \n", cmdId);
                                gscan.processCmd(argc, argv);
                                if(total_iter >= 1 && (total_iter_idx + 1 < argc) &&
                                   atoi(argv[total_iter_idx + 1]) > 0) {
                                    sleep_time = atoi(argv[total_iter_idx + 1]);
                                    if (validate_sleep_duration(sleep_time))
                                        return -1;
                                    sleep(sleep_time);
                                }
                            }
                            i = total_iter_idx + 2;
                            break;
                        default:
                            fprintf(stderr, "%s: Unknown input.\n", __func__);
                    }
                }
            } else {
                /*interactive mode*/
                //gscan.processCmd(argc, argv);
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Step 1: Enter Request ID for GSCAN Cmd\n");
                fprintf(stderr, "Step 2: Enter ID for GSCAN Cmd, as follows:\n");
                fprintf(stderr, "    Type 1 for GSCAN Start\n");
                fprintf(stderr, "    Type 2 for GSCAN Stop\n");
                fprintf(stderr, "    Type 3 for GSCAN Get Valid Channels\n");
                fprintf(stderr, "    Type 4 for GSCAN Get Capabilities \n");
                fprintf(stderr, "    Type 5 for GSCAN Get Cached Results \n");
                fprintf(stderr, "    Type 6 for GSCAN Set BSSID Hotlist\n");
                fprintf(stderr, "    Type 7 for GSCAN Reset BSSID Hotlist\n");
                fprintf(stderr, "    Type 8 for GSCAN Set Significant change \n");
                fprintf(stderr, "    Type 9 for GSCAN Reset Significant change\n");
                fprintf(stderr, "    Type 10 for GSCAN Set MAC OUI\n");
                fprintf(stderr, "    Type 11 for GSCAN Set ePNO List\n");
                fprintf(stderr, "    Type 12 for GSCAN Set Passpoint List\n");
                fprintf(stderr, "    Type 13 for GSCAN Reset Passpoint List\n");
                fprintf(stderr, "    CMD 14 is reserved\n");
                fprintf(stderr, "    Type 15 for ReSet ePNO\n");
                fprintf(stderr, "    Type 1000 to exit.\n");
                fprintf(stderr, "Note: Use ROAM for roaming APIs as below\n");
                fprintf(stderr, "      hal_proxy_daemon ROAM\n");
                fprintf(stderr, "**********************\n");
                int temp;
                while (1) {
                    fprintf(stderr, "*********************\n");
                    fprintf(stderr, "Step 1: Now Enter Request ID:\n");
                    /* Set request Id for GSCAN object. */
                    read_int(&temp);
                    gscan.setRequestId(temp);
                    fprintf(stderr, "Step 2: Enter GSCAN Cmd ID:\n");
                    read_int(&cmdId);
                    if (cmdId == 1000)
                        break;
                    char addr[10];
                    oui scan_oui = {0};
                    switch(cmdId) {
                        case 3:
                            fprintf(stderr, "Step 3: Enter max number of "
                                    "channels:\n");
                            read_int(&max);
                            fprintf(stderr, "Step 4: Enter channels band:\n");
                            read_int(&band);
                            gscan.executeCmd(argc, argv, cmdId, max, flush, band,
                                    scan_oui);
                            break;
                        case 5:
                            fprintf(stderr, "Step 3: Enter max number of "
                                    "cached results:\n");
                            read_int(&max);
                            fprintf(stderr, "Step 4: Enter flush (0/1):\n");
                            read_int(&flush);
                            gscan.executeCmd(argc, argv, cmdId, max, flush, band,
                                    scan_oui);
                            break;
                        case 10:
                            fprintf(stderr, "Step 3: Enter 3 Bytes of MAC address"
                                    "in XX:XX:XX form:\n");
                            read_string(addr, sizeof(addr));
                            if (mac_addr_aton(scan_oui, addr, MAC_OUI_LEN) != 0)
                                break;
                            gscan.executeCmd(argc, argv, cmdId, max, flush, band,
                                    scan_oui);
                            break;
                        case 1:
                        case 2:
                        case 4:
                        case 6:
                        case 7:
                        case 8:
                        case 9:
                        case 11:
                        case 12:
                        case 13:
                        case 15:
                            gscan.executeCmd(argc, argv, cmdId, max, flush, band,
                                    scan_oui);
                            break;
                        default:
                            fprintf(stderr, "%s: Unknown input.\n", __func__);
                    }
                }
            }
        }
        else if(strcasecmp(argv[1], RTT_TEST::RttTestSuite::RTT_CMD) == 0)
        {
            srand(time(NULL));
            request_id = rand();
            RTT_TEST::RttTestSuite rtt(
                            ifaceHandle,
                            request_id);
            if (argc > 3) {
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Enter ID for RTT Cmd, as follows:\n");
                fprintf(stderr, "RTT Range Request                 :"
                                 " 1 iterations interval\n");
                fprintf(stderr, "RTT Range Cancel                  :"
                                 " 2 iterations interval\n");
                fprintf(stderr, "RTT Get Capability                :"
                                 " 3 iterations interval\n");
                fprintf(stderr, "RTT Set LCI                       :"
                                 " 4 iterations interval\n");
                fprintf(stderr, "RTT Set LCR                       :"
                                 " 5 iterations interval\n");
                fprintf(stderr, "**********************************\n");
                fprintf(stderr, "##################################\n");
                fprintf(stderr, "interval - time delay between current "
                                "command and next commnad(may be same cmd) "
                                "in msec\n");
                fprintf(stderr, "iterations - No.of times the command to run\n");
                fprintf(stderr, "#########################################\n");
                int temp = 2;
                /* Set request Id for RTT object. */
                rtt.setRequestId(temp);
                int i = ARGV_CMDID_INDEX, iter = 0, total_iter = 0;
                while (i < argc) {
                    cmdId = atoi(argv[i++]);
                    if (cmdId < 0 || cmdId > 5) {
                        fprintf(stderr, "Unknown command\n");
                        break;
                    }

                    if (i+1 >= argc) {
                        fprintf(stderr, "Insufficient Args for cmdId: %d",
                                         cmdId);
                        break;
                    }

                    total_iter = atoi(argv[i]);
                    if (validate_rtt_iterations_count(total_iter))
                        return -1;
                    for (iter = 0; iter < total_iter; iter++) {
                        fprintf(stderr, "Issuing cmd : %d\n", cmdId);
                        rtt.executeCmd(argc, argv, cmdId);
                        if (total_iter >= 1 && atoi(argv[i+1])) {
                            sleep_time = atoi(argv[i+1]);
                            if (validate_rtt_sleep_duration(sleep_time))
                                return -1;
                            usleep(sleep_time * 1000);
                        }
                    }
                    i += 2;
                }
            } else {
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Step 1: Enter Request ID for RTT Cmd\n");
                fprintf(stderr, "Step 2: Enter ID for RTT Cmd, as follows:\n");
                fprintf(stderr, "    Type 1 for RTT Range Request\n");
                fprintf(stderr, "    Type 2 for RTT Range Cancel\n");
                fprintf(stderr, "    Type 3 for RTT Get Capabilities \n");
                fprintf(stderr, "    Type 4 for RTT Set LCI \n");
                fprintf(stderr, "    Type 5 for RTT Set LCR \n");
                fprintf(stderr, "    Type 6 for RTT Get Responder Information\n");
                fprintf(stderr, "    Type 7 for RTT Enable responder \n");
                fprintf(stderr, "    Type 8 for RTT Disable responder \n");
                fprintf(stderr, "    Type 1000 to exit.\n");
                fprintf(stderr, "**********************\n");
                int temp;
                while (cmdId != 1000) {
                    fprintf(stderr, "*********************\n");
                    fprintf(stderr, "Now Enter Request ID:\n");
                    /* Set request Id for RTT object. */
                    read_int(&temp);
                    rtt.setRequestId(temp);
                    fprintf(stderr, "Step 2: Enter RTT Cmd ID:\n");
                    read_int(&cmdId);
                    switch(cmdId) {
                        case 1:
                        case 2:
                        case 3:
                        case 4:
                        case 5:
                        case 6:
                        case 7:
                        case 8:
                            rtt.executeCmd(argc, argv, cmdId);
                            break;
                        default:
                            fprintf(stderr, "%s: Unknown input.\n", __func__);
                    }
                }
            }
        }
        else if(strcasecmp(argv[1], RTT_TEST::RttTestSuite::RTTM_CMD) == 0)
        {
            srand(time(NULL));
            request_id = rand();
            RTT_TEST::RttTestSuite rtt(
                            ifaceHandle,
                            request_id);
            if (argc > 3) {
                int temp = 2;
                /* Set request Id for RTT object. */
                rtt.setRequestId(temp);
                int i = ARGV_CMDID_INDEX;
                while (i < argc) {
                    cmdId = atoi(argv[i++]);
                    if (cmdId < 0 || cmdId > 5) {
                        fprintf(stderr, "Unknown command\n");
                        break;
                    }

                    if (i+1 >= argc) {
                        fprintf(stderr, "Insufficient Args for cmdId: %d",
                                         cmdId);
                        break;
                    }

                    glb_total_iter = atoi(argv[i]);
                    if (glb_total_iter > 200)
                    {
                        fprintf(stderr, "Maximum iterations count is 200\n");
                        return -1;
                    }

                    glb_sleep_time = atoi(argv[i+1]);
                    if (glb_sleep_time > 100000)
                    {
                       fprintf(stderr, "Maximum sleep duration allowed is 100 Seconds\n");
                       return -1;
                    }

                    fprintf(stderr, "Issuing RTTM cmd\n");
                    rtt.executeCmd(argc, argv, cmdId);
                    i += 2;
                 }
            }
        }
        else if(strcasecmp(argv[1], TDLS_TEST::TDLSTestSuite::TDLS_CMD) == 0)
        {
            srand(time(NULL));
            request_id = rand();
            TDLS_TEST::TDLSTestSuite TDLS(ifaceHandle);
            int i = 0;
            char addr[20];
            u32 temp_mac[7];
            u8 mac[6] = {0};

            while (1) {
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Step 2: Enter ID for TDLS Cmd, as follows:\n");
                fprintf(stderr, "    Type 1 for TDLS Enable\n");
                fprintf(stderr, "    Type 2 for TDLS Disable\n");
                fprintf(stderr, "    Type 3 for TDLS GetStatus\n");
                fprintf(stderr, "    Type 4 for TDLS Get Capabilities\n");
                fprintf(stderr, "    Type 1000 to exit.\n");
                fprintf(stderr, "*********************\n");
                fprintf(stderr, "TDLS Cmd : ");
                read_int(&cmdId);
                if (cmdId == 1000) {
                    break;
                } else if (cmdId < 1 || cmdId > 4) {
                    fprintf(stderr, "Invalid Cmd : %d \n", cmdId);
                    continue;
                }

                switch(cmdId) {
                    case 1:
                    case 2:
                    case 3:
                        memset(addr, 0, 20 * sizeof(char));
                        memset(&temp_mac[0], 0, 7 * sizeof(u32));
                        memset(&mac[0], 0, 6 * sizeof(u8));
                        fprintf(stderr, "Step 3: MAC ADDR in XX:XX:XX:XX:XX:XX"
                                "format :\n");
                        read_string(addr, sizeof(addr));
                        if (sscanf(addr, "%2x:%2x:%2x:%2x:%2x:%2x%c",
                                    &temp_mac[0], &temp_mac[1],
                                    &temp_mac[2], &temp_mac[3],
                                    &temp_mac[4], &temp_mac[5],
                                    (char *)&temp_mac[6]) != 6) {
                            fprintf(stderr, "Invalid Mac Address \n");
                            continue;
                        }
                        i = 0;
                        while(i<6){
                            mac[i] = (u8)temp_mac[i];
                            i++;
                        }

                        fprintf(stderr, "MAC ADDR in %x:%x:%x:%x:%x:%x"
                         ":\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                        TDLS.executeCmd(argc, argv, cmdId, mac);
                        break;
                    case 4:
                        TDLS.executeCmd(argc, argv, cmdId, mac);
                        break;
                    default:
                        fprintf(stderr, "%s: Unknown input.\n", __func__);
                }
            }
        }
        else if(strcasecmp(argv[1],
            WIFIHAL_TEST::WifiHalTestSuite::WIFIHAL_CMD) == 0)
        {
            WIFIHAL_TEST::WifiHalTestSuite wifiHal(
                            ifaceHandle);
            WIFIHAL_TEST::cmdData data;

            while (cmdId != 1000) {
                cmdId = 0;
                memset(&data, 0, sizeof(WIFIHAL_TEST::cmdData));
                data.no_dfs_flag = 1;

                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Step 1: Enter ID for Wi-Fi HAL Cmd, as follows:\n");
                fprintf(stderr, "    Type 1 for Get Supported Features\n");
                fprintf(stderr, "    Type 2 for Set No DFS Flag\n");
                fprintf(stderr, "    Type 3 for Get Concurrency Matrix\n");
                fprintf(stderr, "    Type 4 for Get the Interfaces available\n");
                fprintf(stderr, "    Type 5 for Set Iface event handler for"
                        " Regulatory domain change monitoring\n");
                fprintf(stderr, "    Type 6 for Reset Iface event handler for"
                        " Regulatory domain change monitoring\n");
                fprintf(stderr, "    Type 7 for Set SAR/tx_power Limits\n");
                fprintf(stderr, "    Type 8 for Reset SAR Limits\n");
                fprintf(stderr, "    Type 9 for Get Usable Channels\n");
                fprintf(stderr, "    Type 10 for Set Coex Unsafe Channels\n");
                fprintf(stderr, "    Type 11 get chip capabilities\n");
                fprintf(stderr, "    Type 1000 to exit.\n");
                fprintf(stderr, "**********************\n");

                fprintf(stderr, "*********************\n");
                fprintf(stderr, "Enter Wi-Fi HAL Cmd ID:\n");
                read_int(&cmdId);
                switch(cmdId) {
                    case 1:
                        break;
                    case 2:
                        fprintf(stderr, "Enter value for No DFS Flag:\n");
                        fprintf(stderr, "0: to ENABLE DFS or 1: to DISABLE DFS\n");
                        read_uint(&data.no_dfs_flag);
                        break;
                    case 3:
                        fprintf(stderr, "Enter value for max concurrency set "
                            "size:\n");
                        read_int(&data.set_size_max);
                        break;
                    case 4:
                        data.wifiHandle = wifiHandle;
                        break;
                    case 5:
                    case 6:
                        fprintf(stderr, "Enter Request ID : \n");
                        read_int(&data.reqId);
                        break;
                    case 7:
                    {
                        int scenario;
                        fprintf(stderr, "Enter scenario ID     :  \n");
                        fprintf(stderr, "\tVOICE_CALL          : 0  \n");
                        fprintf(stderr, "\tHEAD_CELL_OFF       : 1  \n");
                        fprintf(stderr, "\tHEAD_CELL_ON        : 2  \n");
                        fprintf(stderr, "\tBODY_CELL_OFF       : 3  \n");
                        fprintf(stderr, "\tBODY_CELL_ON        : 4  \n:");
                        fprintf(stderr, "\tBODY_BT             : 5  \n");
                        fprintf(stderr, "\tHEAD_HOTSPOT        : 6  \n");
                        fprintf(stderr, "\tHEAD_HOTSPOT_MMW    : 7  \n");
                        fprintf(stderr, "\tBODY_CELL_ON_BT     : 8  \n");
                        fprintf(stderr, "\tBODY_HOTSPOT        : 9  \n");
                        fprintf(stderr, "\tBODY_HOTSPOT_BT     : 10 \n");
                        fprintf(stderr, "\tBODY_HOTSPOT_MMW    : 11 \n");
                        fprintf(stderr, "\tBODY_HOTSPOT_BT_MMW : 12 \n");
                        read_int(&scenario);
                        if (scenario == 0)
                            data.scenario = WIFI_POWER_SCENARIO_VOICE_CALL;
                        else if (scenario == 1)
                          data.scenario = WIFI_POWER_SCENARIO_ON_HEAD_CELL_OFF;
                        else if (scenario == 2)
                          data.scenario = WIFI_POWER_SCENARIO_ON_HEAD_CELL_ON;
                        else if (scenario == 3)
                          data.scenario = WIFI_POWER_SCENARIO_ON_BODY_CELL_OFF;
                        else if (scenario == 4)
                          data.scenario = WIFI_POWER_SCENARIO_ON_BODY_CELL_ON;
                        else if (scenario == 5)
                          data.scenario = WIFI_POWER_SCENARIO_ON_BODY_BT;
                        else if (scenario == 6)
                          data.scenario = WIFI_POWER_SCENARIO_ON_HEAD_HOTSPOT;
                        else if (scenario == 7)
                          data.scenario = WIFI_POWER_SCENARIO_ON_HEAD_HOTSPOT_MMW;
                        else if (scenario == 8)
                          data.scenario = WIFI_POWER_SCENARIO_ON_BODY_CELL_ON_BT;
                        else if (scenario == 9)
                          data.scenario = WIFI_POWER_SCENARIO_ON_BODY_HOTSPOT;
                        else if (scenario == 10)
                          data.scenario = WIFI_POWER_SCENARIO_ON_BODY_HOTSPOT_BT;
                        else if (scenario == 11)
                          data.scenario = WIFI_POWER_SCENARIO_ON_BODY_HOTSPOT_MMW;
                        else if (scenario == 12)
                          data.scenario = WIFI_POWER_SCENARIO_ON_BODY_HOTSPOT_BT_MMW;
                        else {
                          fprintf(stderr, "Invalid scenario: %d\n", scenario);
                          continue;
                        }
                    }
                    break;
                    case 8:
                    break;

                    case 9:
                        data.wifiHandle = wifiHandle;

                        fprintf(stderr, "Enter value of band mask:\n");
                                read_uint(&data.band_mask);
                        fprintf(stderr, "Enter value of iface mask:\n");
                                read_uint(&data.iface_mask);
                        fprintf(stderr, "Enter value of filter mask:\n");
                                read_uint(&data.filter_mask);
                        fprintf(stderr, "Enter max size:\n");
                                read_uint(&data.max_size);
                        fprintf(stderr, "Band mask %u, iface mask %u, filter mask %u, max size %u\n",
                                data.band_mask, data.iface_mask,
                                data.filter_mask, data.max_size);
                        break;
                    case 10:
                         data.wifiHandle = wifiHandle;
                         u32 band;
                         FILE *f_read;
                         char comment[100];
                         f_read = fopen("/data/vendor/wifi/wifi_hal_coex_unsafe_channels_data.txt", "r");
                         if (f_read) {
                             //Detect a comment and move to next line
                             while (!feof(f_read)) {
                                 memset(&comment, 0, sizeof(comment));
                                 if (fscanf(f_read, "%s", comment) == 0) {
                                     fprintf(stderr, "%s: Failed to read data"
                                     " from file. Exit\n", __func__);
                                     goto failure;
                                 }
                                 if (comment[0] != '#')
                                    break;
                                 fscanf(f_read, "%*[^\n]");     // Skip to the End of the Line
                                 fscanf(f_read, "%*1[\n]");     // Skip One Newline
                             }
                             data.no_of_channels = atoi(comment);
                             if (data.no_of_channels > 0) {
                                 data.unsafe_channs = (wifi_coex_unsafe_channel *)malloc(sizeof(wifi_coex_unsafe_channel)*data.no_of_channels);
                                 if (!data.unsafe_channs)
                                     goto failure;
                                 memset(data.unsafe_channs, 0, sizeof(wifi_coex_unsafe_channel)*data.no_of_channels);
                                 if ((fscanf(f_read, "%d", &data.iface_mask)) == 0) {
                                      fprintf(stderr, "%s: Failed to read iface mask"
                                              " from file. Exit\n", __func__);
                                      goto failure;
                                 }
                             }
                             for (int i = 0; i < data.no_of_channels; i++) {
                                 if ((fscanf(f_read, "%d", &data.unsafe_channs[i].band) &&
                                      fscanf(f_read, "%d", &data.unsafe_channs[i].channel) &&
                                      fscanf(f_read, "%d", &data.unsafe_channs[i].power_cap_dbm)) == 0) {
                                      fprintf(stderr, "%s: Failed parsing Unsafe channel data "
                                              " file. Exit\n", __func__);
                                      free(data.unsafe_channs);
                                      goto failure;
                                 }
                             }
                             fclose(f_read);
                             break;
                             failure:
                                 fclose(f_read);
                                 return -1;
                         }
                         else {
                             fprintf(stderr, "Enter num_channels:\n");
                                     read_uint(&data.no_of_channels);
                             fprintf(stderr, "Number of channels Entered:%d\n", data.no_of_channels);
                             if (data.no_of_channels > 0) {
                                 data.unsafe_channs = (wifi_coex_unsafe_channel *)malloc(sizeof(wifi_coex_unsafe_channel)*data.no_of_channels);
                                 if (!data.unsafe_channs)
                                     return -1;
                                 memset(data.unsafe_channs, 0, sizeof(wifi_coex_unsafe_channel)*data.no_of_channels);
                                 fprintf(stderr, "Enter iface bit mask_to be restricted:\n");
                                         read_uint(&data.iface_mask);
                             }
                             for (int i = 0; i < data.no_of_channels; i++) {
                                 fprintf(stderr, "Enter the following data for channel:%d\n", i);
                                 fprintf(stderr, "Enter band:\n---Input 1 for 2G\n---Input 2 for 5G\n---Input 4 for 6G\n");
                                         read_uint(&band);
                                 data.unsafe_channs[i].band = (wlan_mac_band) band;
                                 fprintf(stderr, "Enter channel:\n");
                                         read_uint(&data.unsafe_channs[i].channel);
                                 fprintf(stderr, "Enter power_cap_dbm:\n");
                                         read_int(&data.unsafe_channs[i].power_cap_dbm);
                             }
                         }
                         break;
                    case 11:
                        data.wifiHandle = wifiHandle;
                        break;
                    default:
                        fprintf(stderr, "%s: Unknown input.\n", __func__);
                        continue;
                }
                wifiHal.executeCmd(cmdId, data);
            }
        }
        else if(strcasecmp(argv[1],
            WIFI_CONFIG_TEST::WiFiConfigTestSuite::WIFI_CONFIG_CMD) == 0)
        {
            WIFI_CONFIG_TEST::WiFiConfigTestSuite wifiConfig(ifaceHandle);
            if (argc > 3) {
                /*Support for configuration of DTIM multiplier*/
                if(argc == 5)
                    return wifiConfig.processCmd(argc, argv);

                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Enter ID for Config Cmd, as follows:\n");
                fprintf(stderr, " Set Extended DTIM                       :"
                                " 1 <Extended_dtim> <iteration> <interval>\n");
                fprintf(stderr, " Set Avg. Factor                         :"
                                "2 <Avg_factor> <iteration> <interval>\n");
                fprintf(stderr, "Set Guard Time                           :"
                                "3 <Guard_time> <iteration> <interval>\n");
                fprintf(stderr, "Set Country Code                         :"
                                "4 <Country_code> <iteration> <interval>\n");
                fprintf(stderr, "Set Latency Mode                         :"
                                "5 <Latency_mode> <iteration> <interval>\n");
                fprintf(stderr, "Set Thermal Mode                         :"
                                "6 <thermal_mode> <completion window> <iteration> <interval>\n");
                fprintf(stderr, "Set Primary Sta in Multi Sta Connection  :"
                                "7 <sta_iface> <iteration> <interval>\n");
                fprintf(stderr, "Set Use Case in Multi Sta Connection     :"
                                "8 <use_case> <iteration> <interval>\n");
                fprintf(stderr, "Set Modulated DTIM                       :"
                                "9 <Modulated_dtim> <iteration> <interval>\n");
                fprintf(stderr, "Set Voip mode                       :"
                                "10 <voip_mode> <iteration> <interval>\n");
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "######################\n");
                fprintf(stderr, "interval - time delay between current "
                                "command and next commnad(may be same cmd) "
                                "in seconds\n");
                fprintf(stderr, "iterations -No.of times the command to run\n");
                fprintf(stderr, "######################\n");
                int temp = 4;
                /* Set request Id for Wi-Fi Config object. */
                wifiConfig.setRequestId(temp);
                WIFI_CONFIG_TEST::cmdData data;
                int i = ARGV_CMDID_INDEX, iter = 0, total_iter = 0;
                while (i < argc) {
                    cmdId = atoi(argv[i++]);
                    fprintf(stderr, "cmd : %d\n", cmdId);
                    if (cmdId < 0 || cmdId > 9) {
                        fprintf(stderr, "Unknown command\n");
                        break;
                    }
                    memset(&data, 0, sizeof(WIFI_CONFIG_TEST::cmdData));
                    switch(cmdId) {
                        case 1:
                            if (i < argc)
                                data.dtim = atoi(argv[i++]);
                            break;
                        case 2:
                            if (i < argc)
                                data.avgFactor = atoi(argv[i++]);
                            break;
                        case 3:
                            if (i < argc)
                                data.guardTime = atoi(argv[i++]);
                            break;
                        case 4:
                            if (i < argc)
                                strlcpy(data.countryCode, argv[i++],
                                        sizeof(data.countryCode));
                            break;
                        case 5:
                            int latency_mode;

                            if (i >= argc)
                                break;

                            latency_mode = atoi(argv[i++]);
                            switch(latency_mode) {
                                case 0:
                                    data.latencyMode = WIFI_LATENCY_MODE_NORMAL;
                                    break;
                                case 1:
                                    data.latencyMode = WIFI_LATENCY_MODE_LOW;
                                    break;
                                default:
                                    fprintf(stderr, "%s: Unknown latency mode.\n", __func__);
                                    break;
                            }
                            break;
                        case 6:
                            data.wifiHandle = wifiHandle;
                            if (i < argc)
                                data.thermalMode = (wifi_thermal_mode)atoi(argv[i++]);
                            if (i < argc)
                                data.completionWindow = (u32)atoi(argv[i++]);
                            break;
                        case 7:
                            data.wifiHandle = wifiHandle;
                            if (i >= argc)
                                break;
                            data.ifaceHandle = wifi_get_iface_handle(wifiHandle, argv[i++]);
                            if(!data.ifaceHandle)
                            {
                                 fprintf(stderr, "Interface %s is not up, exiting.\n", argv[i-1]);
                                 break;
                            }
                            break;
                        case 8:
                            data.wifiHandle = wifiHandle;
                            int case_info;

                            if (i >= argc)
                                break;

                            case_info = atoi(argv[i++]);
                            switch(case_info) {
                                case 0:
                                    data.caseInfo = WIFI_DUAL_STA_TRANSIENT_PREFER_PRIMARY;
                                    break;
                                case 1:
                                    data.caseInfo = WIFI_DUAL_STA_NON_TRANSIENT_UNBIASED;
                                    break;
                                default:
                                    fprintf(stderr, "%s: Unknown Use Case.\n", __func__);
                                    break;
                            }
                            break;
                        case 9:
                            if (i < argc)
                                data.dtim = atoi(argv[i++]);
                            break;
                        case 10:
                            data.wifiHandle = wifiHandle;
                            if (i >= argc)
                                break;
                            data.ifaceHandle = wifi_get_iface_handle(wifiHandle, argv[i++]);
                            if(!data.ifaceHandle)
                            {
                                 fprintf(stderr, "Interface %s is not up, exiting.\n", argv[i-1]);
                                 break;
                            }
                            data.voip_mode =(wifi_voip_mode)atoi(argv[i++]);
                            break;
                        default:
                            fprintf(stderr, "%s: Unknown input.\n", __func__);
                    }
                    if (i < argc) {
                        total_iter = atoi(argv[i]);
                        if (validate_iterations_count(total_iter))
                            return -1;
                    } else {
                        fprintf(stderr, "No iterations found: consider as"
                                " a single instance \n");
                        total_iter = 1;
                    }
                    for (iter = 0; iter < total_iter; iter++) {
                        wifiConfig.executeCmd(cmdId, data);
                        if (total_iter >= 1 && ((i+1 < argc) && atoi(argv[i+1]))) {
                            sleep_time = atoi(argv[i+1]);
                            if (validate_sleep_duration(sleep_time))
                                return -1;
                            sleep(sleep_time);
                        }
                    }
                    i += 2;
                }
            } else {
                /* If number of args = 3, use default menu driven interface. */
                WIFI_CONFIG_TEST::cmdData data;
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Step 1: Enter Request ID for Wi-Fi Config Cmd\n");
                fprintf(stderr, "Step 2: Enter ID for Config Cmd, as follows:\n");
                fprintf(stderr, "    Type 1 for Set Extended DTIM\n");
                fprintf(stderr, "    Type 2 for Set Avg. Factor\n");
                fprintf(stderr, "    Type 3 for Set Guard Time\n");
                fprintf(stderr, "    Type 4 for Set Country Code\n");
                fprintf(stderr, "    Type 5 for Set Latency Mode\n");
                fprintf(stderr, "    Type 6 for Set Thermal Mode\n");
                fprintf(stderr, "    Type 7 for Set Primary Sta in Multi-Sta\n");
                fprintf(stderr, "    Type 8 for Set Multi Sta Use Case\n");
                fprintf(stderr, "    Type 9 for Set Modulated DTIM\n");
                fprintf(stderr, "    Type 10 for Set Voip Mode\n");
                fprintf(stderr, "    Type 11 for get chip capabilities\n");
                fprintf(stderr, "    Type 1000 to exit.\n");
                fprintf(stderr, "**********************\n");
                int temp, iface_i, iface_num;
                while (cmdId != 1000) {
                    iface_i = 0;
                    iface_num = 0;
                    cmdId = 0;
                    memset(&data, 0, sizeof(WIFI_CONFIG_TEST::cmdData));
                    fprintf(stderr, "*********************\n");
                    fprintf(stderr, "Step 1: Now Enter Request ID:\n");
                    /* Set request Id for Wi-Fi Config object. */
                    read_int(&temp);
                    wifiConfig.setRequestId(temp);
                    fprintf(stderr, "Step 2: Enter Wi-Fi Config Cmd ID:\n");
                    read_int(&cmdId);
                    switch(cmdId) {
                        case 1:
                            fprintf(stderr, "Enter value for Extended DTIM:\n");
                            read_int(&data.dtim);
                            break;
                        case 2:
                            fprintf(stderr, "Enter value for Avg. Factor:\n");
                            read_uint(&data.avgFactor);
                            break;
                        case 3:
                            fprintf(stderr, "Enter value for Guard Time:\n");
                            read_uint(&data.guardTime);
                            break;
                        case 4:
                            fprintf(stderr, "Enter value for Country Code:\n");
                            read_string(data.countryCode, sizeof(data.countryCode));
                            break;
                        case 5:
                            int latency_mode;

                            fprintf(stderr, "Enter value for Latency Mode:\n");
                            fprintf(stderr, "Latency Modes - 0: NORMAL, 1: ULTRALOW, Default: NORMAL\n");
                            read_int(&latency_mode);
                            switch(latency_mode) {
                                case 0:
                                    data.latencyMode = WIFI_LATENCY_MODE_NORMAL;
                                    break;
                                case 1:
                                    data.latencyMode = WIFI_LATENCY_MODE_LOW;
                                    break;
                                default:
                                    fprintf(stderr, "%s: Unknown latency mode.\n", __func__);
                                    break;
                            }
                            break;
                        case 6:
                            data.wifiHandle = wifiHandle;

                            fprintf(stderr, "Enter value for thermal level:\n");
                            fprintf(stderr, "Thermal modes - 0: NONE, 1: LIGHT, 2: MODERATE, 3: SEVERE, 4: CRITICAL, 5: EMERGENCY, Default: NONE\n");
                            read_uint((unsigned int *)&data.thermalMode);

                            fprintf(stderr, "Enter value for completion window of thermal mode configuration:\n");
                            fprintf(stderr, "completion window (in milliseconds) \n");
                            read_uint(&data.completionWindow);
                            break;
                        case 7:
                            data.wifiHandle = wifiHandle;
                            wifi_interface_handle *interfaces;
                            WIFIHAL_TEST::interface_info **iface;
                            char name[IFNAMSIZ+1];
                            size_t len;
                            fprintf(stderr, "Enter primary sta iface in Multi Sta:\n");

                            wifi_get_ifaces(wifiHandle, &iface_num, &interfaces);
                            iface = (WIFIHAL_TEST::interface_info **)interfaces;

                            fprintf(stderr, "Number of interfaces available: %d\n", iface_num);

                            for(iface_i = 0; iface_i < iface_num; iface_i++) {
                                if(wifi_get_iface_name((wifi_interface_handle)iface[iface_i],
                                                        &name[0], IFNAMSIZ) == WIFI_SUCCESS) {
                                               fprintf(stderr, "Interface %d : %s\n", iface_i, name);
                                } else {
                                        fprintf(stderr, "Failed to get the iface name");
                                }
                            }
                            fprintf(stderr, "Enter primary sta iface name in Multi Sta:\n");
                            fgets(name, IFNAMSIZ+1, stdin);
                            len = strlen(name);
                            if (len > 0 && name[len - 1] == '\n')
                                  name[len - 1] = '\0';
                            fprintf(stderr, "Interface name entered for primary sta: %s\n", name);
                            data.ifaceHandle = wifi_get_iface_handle(wifiHandle, name);
                            if(!data.ifaceHandle)
                            {
                                 fprintf(stderr, "Interface %s is not up, exiting.\n", name);
                                 break;
                            }
                            break;
                        case 8:
                            data.wifiHandle = wifiHandle;
                            int case_info;

                            fprintf(stderr, "Enter value for Multi Sta Use Case:\n");
                            fprintf(stderr, "Use Case - 0: TRANSIENT_PREFER_PRIMARY, 1: NON_TRANSIENT_UNBIASED, Default: NONE\n");
                            read_int(&case_info);
                            switch(case_info) {
                                case 0:
                                    data.caseInfo = WIFI_DUAL_STA_TRANSIENT_PREFER_PRIMARY;
                                    break;
                                case 1:
                                    data.caseInfo = WIFI_DUAL_STA_NON_TRANSIENT_UNBIASED;
                                    break;
                                default:
                                    fprintf(stderr, "%s: Unknown Use Case.\n", __func__);
                                    break;
                            }
                            break;
                        case 9:
                            fprintf(stderr, "Enter value for Modulated DTIM:\n");
                            read_int(&data.dtim);
                            break;
                        case 10:
                            fprintf(stderr, "Enter (0 for Off, 1 for On) voip mode:\n");
                            read_int((int *)&data.voip_mode);
                            break;
                        case 11:
                            data.wifiHandle = wifiHandle;
                            break;
                        default:
                            fprintf(stderr, "%s: Unknown input.\n", __func__);
                    }
                    wifiConfig.executeCmd(cmdId, data);
                }
            }
        }
        else if(strcasecmp(argv[1],
                WIFI_LOGGER_TEST::WifiLoggerTestSuite::WIFI_LOGGER_CMD) == 0)
        {
            srand(time(NULL));
            request_id = rand();
            // Open and close files to flush the
            FILE *pkt_fate_dump_data =
                         fopen("/data/vendor/wifi/pkt_fate_stats_data.txt", "w");
            FILE *pkt_fate_dump_mgmt =
                         fopen("/data/vendor/wifi/pkt_fate_stats_mgmt.txt", "w");
            if (pkt_fate_dump_mgmt)
                fclose(pkt_fate_dump_mgmt);
            if (pkt_fate_dump_data)
                fclose(pkt_fate_dump_data);

            WIFI_LOGGER_TEST::WifiLoggerTestSuite wifi_logger(wifiHandle, request_id);
//            wifi_logger.processCmd(argc, argv, version_type);
            WIFI_LOGGER_TEST::cmdData data;
            fprintf(stderr, "**********************\n");
            fprintf(stderr, "Step 1: Enter Request ID for WiFiLogger Cmd\n");
            fprintf(stderr, "Step 2: Enter ID for WifiLogger Cmd, as follows:\n");
            fprintf(stderr, "    Type 1 for WifiLogger Get Driver Version \n");
            fprintf(stderr, "    Type 2 for WifiLogger Get Firmware Version \n");
            fprintf(stderr, "    Type 3 for WifiLogger Start Logging  \n");
            fprintf(stderr, "    Type 4 for WifiLogger Stop Logging  \n");
            fprintf(stderr, "    Type 5 for WifiLogger Get Ring Buffer Status \n");
            fprintf(stderr, "    Type 6 for WifiLogger Get Supported Feature Set \n");
            fprintf(stderr, "    Type 7 for WifiLogger Get Ring Data \n");
            fprintf(stderr, "    Type 8 for WifiLogger trigger Memory Dump \n");
            fprintf(stderr, "    Type 9 for WifiLogger Get Driver Memory Dump \n");
            fprintf(stderr, "    Type 10 for WifiLogger Get wake reason stats \n");
            fprintf(stderr, "    Type 11 for WifiLogger Start packet fate stats\n");
            fprintf(stderr, "    Type 12 for WifiLogger Get Tx packet fate stats \n");
            fprintf(stderr, "    Type 13 for WifiLogger Get Rx packet fate stats \n");
            fprintf(stderr, "    Type 1000 to exit.\n");
            fprintf(stderr, "**********************\n");
            int temp;
            while (cmdId != 1000) {
                cmdId = 0;
                memset(&data, 0, sizeof(WIFI_LOGGER_TEST::cmdData));
                fprintf(stderr, "*********************\n");
                fprintf(stderr, "Now Enter Request ID:\n");
                /* Set request Id for WiFiLogger object. */
                read_int(&temp);
                wifi_logger.setRequestId(temp);
                fprintf(stderr, "Step 2: Enter WiFiLogger Cmd ID:\n");
                read_int(&cmdId);
                switch(cmdId) {
                    case 1:
                        data.versionType = 1;
                        break;
                    case 2:
                        data.versionType = 2;
                        break;
                    case 3:
                        fprintf(stderr, "Enter Ring Id:\n");
                        fprintf(stderr, "1) Wake Lock 2) Connectivity "
                                        "3) Per Pkt Stats 4) driver prints "
                                        "5) firmware prints:\n");
                        read_uint(&data.ringId);
                        if (data.ringId >= 6) {
                            fprintf(stderr, "Invalid Ring Id <1-5> \n");
                            continue;
                        }
                        fprintf(stderr, "Enter Verbose level<1-3>:\n");
                        fprintf(stderr, "        1) Normal Log Level:\n");
                        fprintf(stderr, "        2) Log Level to reproduce a"
                                                    "problem:\n");
                        fprintf(stderr, "        3) Log Level to Debug"
                                                    " a problem:\n");
                        fprintf(stderr, "Only verbose Level 3 "
                                        "for per pkt stats: \n");
                        read_uint(&data.verboseLevel);
                        if (!data.verboseLevel || data.verboseLevel > 3) {
                            fprintf(stderr, "Invalid Verbose level \n");
                            continue;
                        }
                        fprintf(stderr, "Enter Max Interval in Sec:\n");
                        read_uint(&data.maxInterval);
                        fprintf(stderr, "Enter Min Data size in bytes:\n");
                        read_uint(&data.minDataSize);
                        fprintf(stderr, "Logs are collected at"
                                        "/data/vendor/wifi/:\n");
                        break;
                    case 4:
                        fprintf(stderr, "Enter Ring Id:\n");
                        fprintf(stderr, "1) Wake Lock 2) Connectivity"
                                        "3) Per Pkt Stats:\n");
                        read_uint(&data.ringId);
                        if (data.ringId >= 4) {
                            fprintf(stderr, "Invalid Ring Id <1-3> \n");
                            continue;
                        }
                        data.verboseLevel = 0;
                        break;
                    case 5:
                    case 6:
                        break;
                    case 7:
                        fprintf(stderr, "Enter Ring Id:\n");
                        fprintf(stderr, "1) Wake Lock 2) Connectivity"
                                        "3) Per Pkt Stats:\n");
                        read_uint(&data.ringId);
                        if (data.ringId < 1 || data.ringId > 3) {
                            fprintf(stderr, "Not Supported Yet \n");
                            continue;
                        }
                        break;
                    case 8:
                    case 9:
                    case 10:
                    case 11:
                    case 12:
                    case 13:
                        break;
                    case 1000:
                        exit(0);
                    default:
                        fprintf(stderr, "%s: Unknown input.\n", __FUNCTION__);
                }
                wifi_logger.executeCmd(argc, argv, cmdId, data);
            }
        }
        else if(strcasecmp(argv[1], ROAM_TEST::RoamTestSuite::ROAM_CMD) == 0)
        {
            ROAM_TEST::RoamTestSuite roam(wifiHandle);
            return roam.processRoamCmd(argc, argv);

        }
        else if(strcasecmp(argv[1], WFC_TEST::WFCTestSuite::WFC_CMD) == 0)
        {
            srand(time(NULL));
            request_id = rand();
            WFC_TEST::WFCTestSuite WFC(
                            ifaceHandle,
                            request_id);
            if (argc > 3) {
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Enter ID for WFC Cmd, as follows:\n");
                fprintf(stderr, "Start Sending Offloadedpackets            :"
                                "1 <IP_packet_length> <IP_packet[in hex format in "
                                "space seperated format]> <source_mac_addr> "
                                "<Destination_mac_addr> <period_in_msec> "
                                "<ethr type in hexadecimal format> "
                                "iterations interval\n");
                fprintf(stderr, "Stop sending offloaded packets            :"
                                "2 iterations interval\n");
                fprintf(stderr, "Start monitoring for RSSI                 :"
                                 " 3 min_rssi max_rssi iterations interval\n");
                fprintf(stderr, "Stop monitoring for RSSI                 :"
                                 " 4 iterations interval\n");
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "######################\n");
                fprintf(stderr, "interval - time delay between current "
                                "command and next commnad(may be same cmd) "
                                "in seconds\n");
                fprintf(stderr, "iterations - No.of times the command to run\n");
                fprintf(stderr, "######################\n");
                int temp = 3;
                /* Set request Id for WFC object. */
                WFC.setRequestId(temp);
                void *input_buf = NULL;
                int i = ARGV_CMDID_INDEX, iter = 0, total_iter = 0;
                while (i < argc) {
                    cmdId = atoi(argv[i++]);
                    fprintf(stderr, "cmd : %d\n", cmdId);
                    if (cmdId < 0 || cmdId > 4) {
                        fprintf(stderr, "Unknown command\n");
                        break;
                    }
                    switch(cmdId) {
                        case 1:
                        {
                            if(argc < i || argc <= (i + atoi(argv[i]) +3))
                            {
                                fprintf(stderr, "Insufficient Args for cmdId :"
                                        " %d\n", cmdId);
                                return -1;
                            }
                            startSendOffloadPkts *data;
                            u8 buf[MAX_WLAN_MAC_PKT_LENGTH +
                                   sizeof(startSendOffloadPkts)];
                            int j;
                            u16 length;
                            fprintf(stderr, "Issuing cmd : %d\n", cmdId);
                            length = (u16) atoi(argv[i++]);
                            if (length > MAX_WLAN_MAC_PKT_LENGTH) {
                                fprintf(stderr, "Too big lenght provided\n");
                                break;
                            }

                            data = (startSendOffloadPkts *)buf;
                            data->ip_packet_len = length;

                            for (j = 0; j < data->ip_packet_len; j++)
                                sscanf(argv[i++], "%hhx", &data->ip_packet[j]);

                            fprintf(stderr, "\nData length available: %d", j);
                            fprintf(stderr, "\nData :");
                            for (j = 0; j < data->ip_packet_len; j++)
                                fprintf(stderr, "%hhx ", data->ip_packet[j]);

                            if (mac_addr_aton(data->src_mac_addr, argv[i++]))
                                break;
                            if (mac_addr_aton(data->dst_mac_addr, argv[i++]))
                                break;

                            sscanf(argv[i++], "%u", &data->period_msec);
                            fprintf(stderr, "period in msec: %u\n",
                                    data->period_msec);

                            sscanf(argv[i++], "%hx", &data->ether_type);
                            fprintf(stderr, "ether_type: %04x\n",
                                    data->ether_type);
                            if (i < argc) {
                                total_iter = atoi(argv[i]);
                                if (validate_iterations_count(total_iter))
                                    return -1;
                            } else {
                                fprintf(stderr, "No iterations found: consider as"
                                        " a single instance \n");
                                total_iter = 1;
                            }
                            for (iter = 0; iter < total_iter; iter++) {
                                WFC.processCmd(cmdId, data);
                                if (total_iter >= 1 && ((i+1 < argc) &&
                                    atoi(argv[i+1]))) {
                                    sleep_time = atoi(argv[i+1]);
                                    if (validate_sleep_duration(sleep_time))
                                        return -1;
                                    sleep(sleep_time);
                                }
                            }
                            i += 2;
                            break;
                        }
                        case 3:
                        {
                            if( argc <= i+1)
                            {
                                fprintf(stderr, "Insufficient Args for cmdId :"
                                        " %d\n", cmdId);
                                return -1;
                            }
                            fprintf(stderr, "Issuing cmd : %d\n", cmdId);
                            startMonitoringRssi data;
                            sscanf(argv[i], "%hhd", &data.min_rssi);
                            sscanf(argv[i+1], "%hhd", &data.max_rssi);
                            input_buf = &data;

                            if (i+2 < argc) {
                                total_iter = atoi(argv[i+2]);
                                if (validate_iterations_count(total_iter))
                                    return -1;
                            } else {
                                fprintf(stderr, "No iterations found: consider as"
                                        " a single instance \n");
                                total_iter = 1;
                            }
                            for (iter = 0; iter < total_iter; iter++) {
                                WFC.processCmd(cmdId, input_buf);
                                if (total_iter >= 1 && i+3 < argc) {
                                    sleep_time = atoi(argv[i+3]);
                                    if (validate_sleep_duration(sleep_time))
                                        return -1;
                                    sleep(sleep_time);
                                }
                            }
                            i += 4;
                            break;
                        }
                        case 2:
                        case 4:
                            fprintf(stderr, "Issuing cmd : %d\n", cmdId);
                            if (i < argc) {
                                total_iter = atoi(argv[i]);
                                if (validate_iterations_count(total_iter))
                                    return -1;
                            } else {
                                fprintf(stderr, "No iterations found: consider as"
                                        " a single instance \n");
                                total_iter = 1;
                            }
                            for (iter = 0; iter < total_iter; iter++) {
                                WFC.processCmd(cmdId, input_buf);
                                if (total_iter >= 1 && i+1 < argc) {
                                    sleep_time = atoi(argv[i+1]);
                                    if (validate_sleep_duration(sleep_time))
                                        return -1;
                                    sleep(sleep_time);
                                }
                            }
                            i += 2;
                            break;
                        default:
                            fprintf(stderr, "%s: Unknown input.\n", __func__);
                    }
                }
            } else {
                int temp = 0, i = 0;
                void *input_buf;
                while (1) {
                    input_buf = NULL;
                    fprintf(stderr, "**********************\n");
                    fprintf(stderr, "Step 1: Enter Request ID for WFC Cmd\n");
                    /* Set request Id for WFC object. */
                    read_int(&temp);
                    WFC.setRequestId(temp);
                    fprintf(stderr, "Step 2: Enter ID for WFC Cmd, as follows:\n");
                    fprintf(stderr, "1. Start Sending Offloadedpackets\n");
                    fprintf(stderr, "2. Stop sending offloaded packets\n");
                    fprintf(stderr, "3. Start monitoring for RSSI\n");
                    fprintf(stderr, "4. Stop monitoring for RSSI\n");
                    fprintf(stderr, "Type 1000 to exit.\n");
                    fprintf(stderr, "*********************\n");
                    fprintf(stderr, "WFC Cmd : ");
                    read_int(&cmdId);
                    if (cmdId == 1000) {
                        break;
                    } else if (cmdId < 1 || cmdId > 4) {
                        fprintf(stderr, "Invalid Cmd : %d \n", cmdId);
                        continue;
                    }

                    switch(cmdId) {
                        case 1:
                        {
                            u8 buf[MAX_WLAN_MAC_PKT_LENGTH +
                                   sizeof(startSendOffloadPkts)];
                            startSendOffloadPkts *data;
                            u32 length;
                            char addr[20];
                            char *ip_data;
                            char *saveptr = NULL;
                            char *str = NULL;

                            fprintf(stderr, "Enter IP packet length : \n");
                            read_uint(&length);
                            if (length > MAX_WLAN_MAC_PKT_LENGTH) {
                                fprintf(stderr, "Too big lenght provided\n");
                                break;
                            }

                            data = (startSendOffloadPkts *)buf;
                            data->ip_packet_len = length;

                            ip_data = (char *) malloc(3 * length + 1);

                            fprintf(stderr, "Enter IP packet in hex format in a"
                                            " space separated format ");
                            fprintf(stdout, "(Ex: 0A B3 11 2C)\n");
                            if (ip_data != NULL) {
                                read_string(ip_data, (3 * length + 1));
                                str = strtok_r(ip_data, " ", &saveptr);
                            }

                            for (i=0; str && i<data->ip_packet_len; i++) {
                                sscanf(str, "%hhx", &data->ip_packet[i]);
                                str = strtok_r(NULL, " ", &saveptr);
                            }

                            fprintf(stderr, "\nData :");
                            for (i=0; i<data->ip_packet_len; i++)
                                fprintf(stderr, "%hhx ", data->ip_packet[i]);

                            memset(addr, 0, 20 * sizeof(char));
                            fprintf(stderr, "\nEnter source mac address in "
                                            "xx:xx:xx:xx:xx:xx format\n");
                            read_string(addr, sizeof(addr));
                            if (mac_addr_aton(data->src_mac_addr, addr) != 0)
                                break;

                            memset(addr, 0, 20 * sizeof(char));
                            fprintf(stderr, "Enter destination mac address in "
                                            "xx:xx:xx:xx:xx:xx format\n");
                            read_string(addr, sizeof(addr));
                            if (mac_addr_aton(data->dst_mac_addr, addr) != 0)
                                break;

                            fprintf(stderr, "Enter period in msec: \n");
                            read_uint(&data->period_msec);
                            fprintf(stderr, "period in msec: %u\n", data->period_msec);

                            fprintf(stderr, "Enter ether type in hex"
                                    "(e.g. for IPV4, it should be 0800): \n");
                            read_u16_from_hex(&data->ether_type);
                            fprintf(stderr, "Ether type: %04x\n",
                                    data->ether_type);

                            WFC.processCmd(cmdId, data);
                            break;
                        }
                        case 3:
                        {
                            startMonitoringRssi data;
                            int rssi;
                            fprintf(stderr, "Enter Max Rssi : \n");
                            read_int(&rssi);
                            data.max_rssi = (char)rssi;

                            fprintf(stderr, "Enter Min Rssi : \n");
                            read_int(&rssi);
                            data.min_rssi = (char)rssi;

                            input_buf = &data;
                            WFC.processCmd(cmdId, input_buf);
                            break;
                        }
                        case 2:
                        case 4:
                            WFC.processCmd(cmdId, input_buf);
                            break;
                        default:
                            fprintf(stderr, "%s: Unknown input.\n", __func__);
                    }
                }
            }
        }
        else if(strcasecmp(argv[1], WPF_TEST::WPFTestSuite::WPF_CMD) == 0)
        {
            WPF_TEST::WPFTestSuite wpf(wifiHandle);
            return wpf.executeCmd(argc, argv);
        }
        else if(strcasecmp(argv[1], NDO_TEST::NDOTestSuite::NDO_CMD) == 0)
        {
            NDO_TEST::NDOTestSuite ndo(wifiHandle);
            return ndo.executeCmd(argc, argv);
        }
        else
        {
            fprintf(stderr, "%s: Unknown command %s\n", argv[0], argv[1]);
        }
    }
    else
    {
        fprintf(stderr, "%s: Version: " HAL_PROXY_DAEMON_VER "\n", argv[0]);
        fprintf(stderr, "%s: Insufficent args\n", argv[0]);
    }

    if (cmdId != 1000) {
        while (1) {
            usleep(2000000);
            fprintf(stderr, "daemon: Sleep: \n");
        }
    }
    fn.wifi_cleanup(gbl_handle, cleanup_handler);
    usleep(2000000);

    return halProxyDaemon();
}
