/*
 * controller's State Buffer for Test bin
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef ANDROID12_VENDOR_STATEBUFFER_H
#define ANDROID12_VENDOR_STATEBUFFER_H

#define MAX_NORDIC_DATA_NUM 1000

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
#endif //ANDROID12_VENDOR_STATEBUFFER_H
