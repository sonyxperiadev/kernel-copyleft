/*
 * controller's Nordic Hal Service
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "Nordic.h"
#include "NordicCommandHelper.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "Nordic: "

#undef ALOGW
#undef ALOGE
#undef ALOGD

#include <android/log.h>
#include <hidlmemory/mapping.h>
#include <android/hidl/memory/1.0/IMemory.h>

#define  ALOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  ALOGD(...)  // __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define  ALOGW(...)  // __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)

using ::android::hidl::memory::V1_0::IMemory;
using ::android::hidl::allocator::V1_0::IAllocator;
using ::android::hidl::base::V1_0::IBase;

namespace vendor::shadowcreator::hardware::nordic::implementation {

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
    bool ThreadPause = true;

    uint8_t gs_data[30];
    nordic_command_helper_t *mNordicCommandHelper = nullptr;
    controller_state_buffer_t *m_controller_state_buffer = nullptr;

    pthread_t nordic_pthread;

    bool ThreadExit = true;

    long last_left_data_time = 0;
    long last_right_data_time = 0;

    int32_t get_data(uint8_t st, uint8_t ll) {
        int32_t result = 0;

        uint8_t i;
        uint8_t ss = st >> 3;
        uint8_t ee = (st + ll - 1) >> 3;
        uint8_t mod = st % 8;
        uint32_t m = ((1 << ll) - 1);

        i = ee - ss;

        switch (i) {
            case 0:
                result = (gs_data[ss] >> mod) & m;
                break;

            case 1:
                result = (gs_data[ss] >> mod);
                result |= (gs_data[ee] << (8 - mod));

                result = result & m;
                break;

            case 2:
                result = (gs_data[ss] >> mod);
                result |= (gs_data[ss + 1] << (8 - mod));
                result |= (gs_data[ee] << (16 - mod));

                result = result & m;
                break;

            case 3:
                result = (gs_data[ss] >> mod);
                result |= (gs_data[ss + 1] << (8 - mod));
                result |= (gs_data[ss + 2] << (16 - mod));
                result |= (gs_data[ee] << (24 - mod));

                result = result & m;
                break;

            case 4:
                result = (gs_data[ss] >> mod);
                result |= (gs_data[ss + 1] << (8 - mod));
                result |= (gs_data[ss + 2] << (16 - mod));
                result |= (gs_data[ss + 3] << (24 - mod));
                result |= (gs_data[ee] << (32 - mod));

                result = result & m;
                break;
        }
        return result;
    }

    void formatIMU(float *udata) {
#ifdef IMU16BIT
        int32_t ax = (get_data(AX_OF, AX_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        int32_t ay = (get_data(AY_OF, AY_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        int32_t az = (get_data(AZ_OF, AZ_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;

        float a0 = (float) ax * 0.00488f;
        float a1 = (float) ay * 0.00488f;
        float a2 = (float) az * 0.00488f;
        udata[0] = a0;
        udata[1] = a1;
        udata[2] = a2;

        int32_t gx = (get_data(GX_OF, GX_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        int32_t gy = (get_data(GY_OF, GY_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        int32_t gz = (get_data(GZ_OF, GZ_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;

        float g0 = (float) gx * 0.0024434609111111f;
        float g1 = (float) gy * 0.0024434609111111f;
        float g2 = (float) gz * 0.0024434609111111f;

        udata[3] = g0;
        udata[4] = g1;
        udata[5] = g2;

        int16_t dx = (get_data(MX_OF, MX_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        int16_t dy = (get_data(MY_OF, MY_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        int16_t dz = (get_data(MZ_OF, MZ_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        float d0 = (float) dx / 10000.0f;
        float d1 = (float) dy / 10000.0f;
        float d2 = (float) dz / 10000.0f;
        float theta = sqrt(d0 * d0 + d1 * d1 +
                           d2 * d2);
        float scale = 0;
        if (theta == 0)
            scale = 0;
        else
            scale = sin(theta / 2) / theta;
        float x = d0 * scale;
        float y = d1 * scale;
        float z = d2 * scale;
        float w = sqrt(1 - x * x - y * y - z * z);
        udata[6] = x;
        udata[7] = y;
        udata[8] = z;
        udata[9] = w;
#else
        int32_t ax = get_data(AX_OF, AX_BW);
        if (ax & 0x1000)
            ax = -(ax & 0xfff);
        else
            ax = (ax & 0xfff);

        int32_t ay = get_data(AY_OF, AY_BW);
        if (ay & 0x1000)
            ay = -(ay & 0xfff);
        else
            ay = (ay & 0xfff);

        int32_t az = get_data(AZ_OF, AZ_BW);
        if (az & 0x1000)
            az = -(az & 0xfff);
        else
            az = (az & 0xfff);

        float a0 = (float) ax * 0.01f;
        float a1 = (float) ay * 0.01f;
        float a2 = (float) az * 0.01f;
        udata[0] = a0;
        udata[1] = a1;
        udata[2] = a2;

        int32_t gx = (get_data(GX_OF, GX_BW) << OFFSET_INT32_13) >> OFFSET_INT32_13;
        int32_t gy = (get_data(GY_OF, GY_BW) << OFFSET_INT32_13) >> OFFSET_INT32_13;
        int32_t gz = (get_data(GZ_OF, GZ_BW) << OFFSET_INT32_13) >> OFFSET_INT32_13;

        float g0 = (float) (gx << 3) * 0.0024434609111111f;
        float g1 = (float) (gy << 3) * 0.0024434609111111f;
        float g2 = (float) (gz << 3) * 0.0024434609111111f;
        udata[3] = g0;
        udata[4] = g1;
        udata[5] = g2;

        int16_t dx = (get_data(MX_OF, MX_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        int16_t dy = (get_data(MY_OF, MY_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        int16_t dz = (get_data(MZ_OF, MZ_BW) << OFFSET_INT32_16) >> OFFSET_INT32_16;
        float d0 = (float) dx / 10000.0f;
        float d1 = (float) dy / 10000.0f;
        float d2 = (float) dz / 10000.0f;

        float theta = sqrt(d0 * d0 + d1 * d1 +
                           d2 * d2);

        float scale = 0;
        if (theta == 0)
            scale = 0;
        else
            scale = sin(theta / 2) / theta;

        float x = d0 * scale;
        float y = d1 * scale;
        float z = d2 * scale;

        float w = sqrt(1 - x * x - y * y - z * z);

        udata[6] = x;
        udata[7] = y;
        udata[8] = z;
        udata[9] = w;
#endif
    }

    void toMatrix(float quat[4], float tran[3], float out[16]) {
        float w = quat[3];
        float x = quat[0];
        float y = quat[1];
        float z = quat[2];

        float xx = x * x;
        float yy = y * y;
        float zz = z * z;
        float xy = x * y;
        float wz = w * z;
        float wy = w * y;
        float xz = x * z;
        float yz = y * z;
        float wx = w * x;

        out[0] = -1 * 2 * (xy + wz);
        out[4] = -1 * (1.0f - 2 * (xx + zz));
        out[8] = -1 * 2 * (yz - wx);
        out[12] = -1 * tran[1];

        out[1] = -1 * 2 * (xz - wy);
        out[5] = -1 * 2 * (yz + wx);
        out[9] = -1 * (1.0f - 2 * (xx + yy));
        out[13] = -1 * tran[2];

        out[2] = 1.0f - 2 * (yy + zz);
        out[6] = 2 * (xy - wz);
        out[10] = 2 * (wy + xz);
        out[14] = tran[0];

        out[3] = 0.0f;
        out[7] = 0.0f;
        out[11] = 0.0f;
        out[15] = 1.0f;
    }

    void copyState(scControllerState *from, scControllerState *to) {
        ::memcpy(to, from, sizeof(scControllerState));
    }

    void *nordic_thread_main(void *) {
        uint64_t main_pts;
        int8_t size;
        float imu[10] = {0};
        int32_t cbkt_data;

        scControllerState res;

        std::string name = "controller_service_nordic";
        pthread_setname_np(pthread_self(), name.c_str());

        ALOGD("%s:: controller_service_nordic ThreadExit=%d", __FUNCTION__, ThreadExit);

        while (!ThreadExit) {

            pthread_mutex_lock(&mutex);
            while (ThreadPause) {
                pthread_cond_wait(&cond, &mutex);
            }
            pthread_mutex_unlock(&mutex);

            long current_time = time(NULL);
            if (mNordicCommandHelper) {
                int count = Nordic_Command_Helper_Get_Data_Size(mNordicCommandHelper);

                ALOGW("%s:: get %d data from nordic command helper", __FUNCTION__, count);

                while (count--) {
                    int result = Nordic_Command_Helper_Get_Next_Data(mNordicCommandHelper, gs_data, &main_pts, &size);

                    if (result == 0) {
                        int32_t type = (get_data(R1_OF, R1_BW) << OFFSET_INT32_7) >> OFFSET_INT32_7;

                        formatIMU(imu);
                        uint8_t pi = gs_data[1];

                        res.accelerometer.x = imu[0];
                        res.accelerometer.y = imu[1];
                        res.accelerometer.z = imu[2];
                        res.gyroscope.x = imu[3];
                        res.gyroscope.y = imu[4];
                        res.gyroscope.z = imu[5];
                        res.rotation.x = imu[6];
                        res.rotation.y = imu[7];
                        res.rotation.z = imu[8];
                        res.rotation.w = imu[9];
                        res.timestamp = (main_pts & 0xFFFFFFFFFFFFFF00) + pi;
                        res.connectionState = kConnected;

                        cbkt_data = get_data(TX_OF, COMB_TX2END_BW_K101);

                        if (cbkt_data < 0xFFFF) {
                            continue;
                        }

                        res.analog2D[0].x = (float) (cbkt_data & 0xF);
                        res.analog2D[0].y = (float) ((cbkt_data & 0xF0) >> 4);

                        res.isTouching = (cbkt_data & 0xF00) >> 8;

                        res.buttonState = (cbkt_data & 0xF000) >> 12;

                        res.analog1D[0] = (float) ((cbkt_data & 0xF0000) >> 16);
                        res.analog1D[1] = (float) ((cbkt_data & 0xF00000) >> 20);

                        res.battery = (cbkt_data & 0x7000000) >> 24;

                        if (gs_data[0] == LEFT) {
                            ALOGW("%s:: get cbkt_data from nordic is=%x for left", __FUNCTION__, cbkt_data);

                            int32_t next_position = (m_controller_state_buffer->left_position + 1) % MAX_NORDIC_DATA_NUM;

                            copyState(&res,
                                      &m_controller_state_buffer->left_controller_state[next_position]);

                            m_controller_state_buffer->left_position = next_position;

                            last_left_data_time = current_time;
                        } else {
                            ALOGW("%s:: get cbkt_data from nordic is=%x for right", __FUNCTION__, cbkt_data);

                            int32_t next_position = (m_controller_state_buffer->right_position + 1) % MAX_NORDIC_DATA_NUM;

                            copyState(&res,
                                      &m_controller_state_buffer->right_controller_state[next_position]);

                            m_controller_state_buffer->right_position = next_position;

                            last_right_data_time = current_time;
                        }
                    }
                }
            } else {
                ThreadExit = true;
                m_controller_state_buffer->left_position = -1;
                m_controller_state_buffer->right_position = -1;
            }

            if (current_time - last_left_data_time >= 2) {
                m_controller_state_buffer->left_position = -1;
            }

            if (current_time - last_right_data_time >= 2) {
                m_controller_state_buffer->right_position = -1;
            }

            usleep(5000);
        }

        ALOGD("%s:: controller_service_nordic finish ThreadExit=%d", __FUNCTION__, ThreadExit);

        pthread_exit(NULL);
    }

    static void ControllerHandler(int t) {
        if (!ThreadExit) {
            ThreadExit = true;
            pthread_join(nordic_pthread, NULL);
        }

        if (mNordicCommandHelper) {
            Nordic_Command_Helper_Stop(mNordicCommandHelper);
            Nordic_Command_Helper_Destroy(mNordicCommandHelper);
            mNordicCommandHelper = NULL;
        }
    }

    static void setDieHandler() {
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sigemptyset(&sa.sa_mask);
        sa.sa_handler = ControllerHandler;

        int err = sigaction(SIGABRT, &sa, NULL);
        if (err < 0) {
            ALOGE("%s:: Error setting SIGCHLD handler", __FUNCTION__);
        }
    }

    Nordic::Nordic() {
        ThreadExit = false;

        pthread_create(&nordic_pthread, NULL, nordic_thread_main, NULL);

        setDieHandler();

        ALOGD("%s:: start finish!!", __FUNCTION__);

        int32_t pageSize = getpagesize();

        sp <IAllocator> allocator = IAllocator::getService("ashmem");
        int32_t len = ((mSize + pageSize - 1) & ~(pageSize - 1));
        allocator->allocate(len, [&](bool success, const hidl_memory &mem) {
            if (!success) {
                ALOGE("memory allocate failed!!!");
                return;
            }
            m_hidl_handle = native_handle_clone(mem.handle());
            m_hidl_heap = hidl_memory("ashmem", m_hidl_handle, len);
            m_hidl_heapMemory = mapMemory(m_hidl_heap);
            if (m_hidl_heapMemory == nullptr) {
                ALOGE("memory map failed!!!");
                native_handle_close(m_hidl_handle); // close FD for the shared memory
                native_handle_delete(m_hidl_handle);
                m_hidl_heap = hidl_memory();
                m_hidl_handle = nullptr;
                return;
            }
            m_hidl_heapMemData = m_hidl_heapMemory->getPointer();
            if (m_hidl_heapMemData != NULL) {
                m_controller_state_buffer = (controller_state_buffer_t *) m_hidl_heapMemData;
                if (m_controller_state_buffer) {
                    m_controller_state_buffer->left_position = -1;
                    m_controller_state_buffer->right_position = -1;
                }
            } else {
                ALOGE("get memory data failed!!!");
                native_handle_close(m_hidl_handle); // close FD for the shared memory
                native_handle_delete(m_hidl_handle);
                m_hidl_heap = hidl_memory();
                m_hidl_handle = nullptr;
                return;
            }
        });
    }

    Nordic::~Nordic() {
        if (!ThreadExit) {
            ThreadExit = true;
            pthread_join(nordic_pthread, NULL);
        }

        pthread_mutex_destroy(&mutex);
        pthread_cond_destroy(&cond);

        if (mNordicCommandHelper) {
            Nordic_Command_Helper_Stop(mNordicCommandHelper);
            Nordic_Command_Helper_Destroy(mNordicCommandHelper);
            mNordicCommandHelper = nullptr;
        }

        if (m_controller_state_buffer) {
            m_controller_state_buffer = NULL;
        }

        if (m_hidl_heapMemory != nullptr) {
            m_hidl_heapMemData = nullptr;
            m_hidl_heapMemory.clear(); // The destructor will trigger munmap
        }

        if (m_hidl_handle) {
            native_handle_close(m_hidl_handle); // close FD for the shared memory
            native_handle_delete(m_hidl_handle);
        }
    }

// Methods from ::vendor::shadowcreator::hardware::nordic::V1_0::INordic follow.
    Return<void> Nordic::helloWorld(const hidl_string &name, helloWorld_cb _hidl_cb) {
        char buf[128];
        ::memset(buf, 0, 128);
        ::snprintf(buf, 128, "Starting %s", name.c_str());
        hidl_string result(buf);
        _hidl_cb(result);
        return Void();
    }

    Return <int32_t> Nordic::Nordic_Start() {
        ALOGD("%s:: start create nordic helper mNordicCommandHelper is null %d", __FUNCTION__, mNordicCommandHelper == nullptr);
        if (!mNordicCommandHelper) {
            mNordicCommandHelper = Nordic_Command_Helper_Create();

            if (!mNordicCommandHelper) {
                ALOGE("%s:: create mNordicCommandHelper fail!!", __FUNCTION__);
                return -1;
            }

            int ret = Nordic_Command_Helper_Start(mNordicCommandHelper);

            if (ret < 0) {
                ALOGE("%s:: start nordic helper fail!!", __FUNCTION__);
                if (mNordicCommandHelper) {
                    Nordic_Command_Helper_Stop(mNordicCommandHelper);
                    Nordic_Command_Helper_Destroy(mNordicCommandHelper);
                    mNordicCommandHelper = NULL;
                }
                return -1;
            }
        }

        int32_t ret = Nordic_Get_Bind_State();

        return ret;
    }

    Return<void> Nordic::Nordic_Get_Memory(Nordic_Get_Memory_cb _hidl_cb) {
        if (m_hidl_handle) {
            ALOGD("m_hidl_handle data[0] = %d", m_hidl_handle->data[0]);
        } else {
            ALOGE("m_hidl_handle is NULL!");
        }
        _hidl_cb(m_hidl_handle, mSize);
        return Void();
    }

    Return <int32_t> Nordic::Nordic_Stop() {
        int32_t ret = 0;
        if (!ThreadPause) {
            pthread_mutex_lock(&mutex);
            ThreadPause = true;
            pthread_mutex_unlock(&mutex);
        }
        return ret;
    }

    Return<float> Nordic::Nordic_Get_Nordic_Version() {
        float ret = -1.0f;

        if (mNordicCommandHelper) {
            Nordic_Command_Helper_Get_Nordic_Version(mNordicCommandHelper, &ret);
        }

        return ret;
    }

    Return<float> Nordic::Nordic_Get_Controller_Version(int32_t lr) {
        float ret = -1.0f;

        if (mNordicCommandHelper) {
            Nordic_Command_Helper_Get_Controller_Version(mNordicCommandHelper, &ret, lr);
        }

        return ret;
    }

    Return <int32_t> Nordic::Nordic_Bind_Controller(int32_t lr) {
        int32_t ret = 0;
        if (mNordicCommandHelper) {
            Nordic_Command_Helper_Bind_Controller(mNordicCommandHelper, lr);
        }

        return ret;
    }

    Return <int32_t> Nordic::Nordic_Unbind_Controller(int32_t lr) {
        int32_t ret = 0;
        if (mNordicCommandHelper) {
            Nordic_Command_Helper_Unbind_Controller(mNordicCommandHelper, lr);
        }

        return ret;
    }

    Return <int32_t> Nordic::Nordic_Cancel_Bind() {
        int32_t ret = 0;
        if (mNordicCommandHelper) {
            Nordic_Command_Helper_Cancel_Bind(mNordicCommandHelper);
        }

        return ret;
    }

    Return <int32_t> Nordic::Nordic_Get_Bind_State() {
        int32_t ret = -1;
        if (mNordicCommandHelper) {
            Nordic_Command_Helper_Get_Bind_State(mNordicCommandHelper, &ret);
        }

        if (ThreadPause && ret == 0x11) {
            pthread_mutex_lock(&mutex);
            ThreadPause = false;
            pthread_cond_signal(&cond);
            pthread_mutex_unlock(&mutex);
        }

        return ret;
    }

    Return <int32_t> Nordic::Nordic_Set_Vibration(int32_t value) {
        int32_t ret = -1;
        if (mNordicCommandHelper) {
            ret = Nordic_Command_Helper_Set_Vibration(mNordicCommandHelper, value);
        }

        return ret;
    }
// Methods from ::android::hidl::base::V1_0::IBase follow.

    V1_0::INordic *HIDL_FETCH_INordic(const char * /* name */) {
        return new Nordic();
    }
//
}  // namespace vendor::shadowcreator::hardware::nordic::implementation
