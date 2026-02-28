/*
 * controller's Nordic Hal Service
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#pragma once

#include <vendor/shadowcreator/hardware/nordic/1.0/INordic.h>
#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>

#include <hidlmemory/mapping.h>
#include <android/hidl/allocator/1.0/IAllocator.h>
#include <android/hidl/memory/1.0/IMemory.h>


#define MAX_NORDIC_DATA_NUM 1000

#define OFFSET_INT32_7  25
#define OFFSET_INT32_13 19
#define OFFSET_INT32_16 16

#define TM_BW  8

#define PI_BW  8

#define MX_BW  16
#define MY_BW  16
#define MZ_BW  16

#ifdef IMU16BIT

// ACC
#define AX_BW  16
#define AY_BW  16
#define AZ_BW  16
    // Gyro
#define GX_BW  16
#define GY_BW  16
#define GZ_BW  16
    // Magnetic
#define R1_BW  7
#define R2_BW  7
#define R3_BW  7

#else

// ACC
#define AX_BW  13
#define AY_BW  13
#define AZ_BW  13
// Gyro
#define GX_BW  13
#define GY_BW  13
#define GZ_BW  13
// Magnetic
#define R1_BW  13
#define R2_BW  13
#define R3_BW  13

#endif

// ==== K 102 ====
// {
#define TX_BW  4
#define TY_BW  4

// press key sensor
#define SENSE_BW  4

#define BUTTON_BW_K101  4

#define H1_BW   4
#define H2_BW   4

#define BAT_BW_K101  3
#define COMB_TX2END_BW_K101   27
// }

// ==== K 11 ====
// {
//117
#define TX_BW  4
#define TY_BW  4

// 8 buttons
#define BUTTON_BW  8

#define BAT_BW  8

#define COMB_TX2END_BW   24
//24
// }

#define TM_OF   0
#define PI_OF  (TM_OF+TM_BW)

#define MX_OF  (PI_OF+PI_BW)
#define MY_OF  (MX_OF+MX_BW)
#define MZ_OF  (MY_OF+MY_BW)

#define AX_OF  (MZ_OF+MZ_BW)
#define AY_OF  (AX_OF+AX_BW)
#define AZ_OF  (AY_OF+AY_BW)
#define GX_OF  (AZ_OF+AZ_BW)
#define GY_OF  (GX_OF+GX_BW)
#define GZ_OF  (GY_OF+GY_BW)
#define R1_OF  (GZ_OF+GZ_BW)
#define R2_OF  (R1_OF+R1_BW)
#define R3_OF  (R2_OF+R2_BW)

#define TX_OF  (R3_OF+R3_BW)
#define TY_OF  (TX_OF+TX_BW)
#define BT_OF  (TY_OF+TY_BW)
#define BAT_OF (BT_OF+BUTTON_BW)

namespace vendor::shadowcreator::hardware::nordic::implementation {

    using ::android::hidl::memory::V1_0::IMemory;
    using ::android::hardware::hidl_array;
    using ::android::hardware::hidl_memory;
    using ::android::hardware::hidl_string;
    using ::android::hardware::hidl_vec;
    using ::android::hardware::Return;
    using ::android::hardware::Void;
    using ::android::sp;

    struct scQuaternion {
        float x, y, z, w;
    };

    struct scVector3 {
        float x, y, z;
    };

    struct scVector2 {
        float x, y;
    };

    enum scControllerConnectionState {
        kNotInitialized = 0,
        kDisconnected = 1,
        kConnected = 2,
        kConnecting = 3,
        kError = 4
    };

    struct scControllerState {
        //! Orientation
        scQuaternion rotation;

        //! Gyro
        scVector3 gyroscope;

        //! Accelerometer
        scVector3 accelerometer;

        //! timestamp
        uint64_t timestamp;

        //! All digital button states as bitflags
        uint32_t buttonState;   //!< all digital as bit flags

        //! Touchpads, Joysticks
        scVector2 analog2D[4];       //!< analog 2D's

        //! Triggers
        float analog1D[8];       //!< analog 1D's

        //! Whether the touchpad area is being touched.
        uint32_t isTouching;

        //! battery
        uint8_t battery;

        //! Controller Connection state
        scControllerConnectionState connectionState;

        //! reserved
        uint8_t reserved[128];
    };

    typedef struct controller_state_buffer {
        int32_t left_position;
        int32_t right_position;
        scControllerState left_controller_state[MAX_NORDIC_DATA_NUM];
        scControllerState right_controller_state[MAX_NORDIC_DATA_NUM];
    } controller_state_buffer_t;

    enum ControllerFlag {
        LEFT = 0xFF,
        RIGHT = 0xFE,
    };

    struct Nordic : public V1_0::INordic {
    public:
        Nordic();
        ~Nordic();

        // Methods from ::vendor::shadowcreator::hardware::nordic::V1_0::INordic follow.
        Return<void> helloWorld(const hidl_string &name, helloWorld_cb _hidl_cb) override;

        Return <int32_t> Nordic_Start() override;

        Return<void> Nordic_Get_Memory(Nordic_Get_Memory_cb _hidl_cb) override;

        Return <int32_t> Nordic_Stop() override;

        Return<float> Nordic_Get_Nordic_Version() override;

        Return<float> Nordic_Get_Controller_Version(int32_t lr) override;

        Return <int32_t> Nordic_Bind_Controller(int32_t lr) override;

        Return <int32_t> Nordic_Unbind_Controller(int32_t lr) override;

        Return <int32_t> Nordic_Cancel_Bind() override;

        Return <int32_t> Nordic_Get_Bind_State() override;

        Return <int32_t> Nordic_Set_Vibration(int32_t value) override;
        // Methods from ::android::hidl::base::V1_0::IBase follow.

    private:
        native_handle_t *m_hidl_handle = nullptr;

        hidl_memory m_hidl_heap;
        void *m_hidl_heapMemData;
        sp <IMemory> m_hidl_heapMemory;
        int32_t mSize = sizeof(controller_state_buffer);
    };

// FIXME: most likely delete, this is only for passthrough implementations
    extern "C" V1_0::INordic *HIDL_FETCH_INordic(const char *name);

}  // namespace vendor::shadowcreator::hardware::nordic::implementation
