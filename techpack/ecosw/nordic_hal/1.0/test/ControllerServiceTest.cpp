/*
 * controller's Test bin for Nordic Service
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <vendor/shadowcreator/hardware/nordic/1.0/INordic.h>
#include <hidl/Status.h>
#include <hidl/LegacySupport.h>
#include <utils/misc.h>
#include <hidl/HidlSupport.h>
#include <iostream>
#include <stdio.h>
#include <thread>
#include <binder/IPCThreadState.h>
#include <binder/ProcessState.h>
#include "StateBuffer.h"

using ::android::hardware::hidl_string;
using ::android::sp;
using vendor::shadowcreator::hardware::nordic::V1_0::INordic;
using android::hardware::hidl_string;
using android::hardware::hidl_handle;
using ::android::hidl::memory::V1_0::IMemory;
using namespace std;
using namespace android;

void showMenu() {
    cout << "****Option Menu****" << endl;
    cout << "1. Bind Left Controller" << endl;
    cout << "2. Bind Right Controller" << endl;
    cout << "3. Unbind Left Controller" << endl;
    cout << "4. Unbind Right Controller" << endl;
    cout << "5. Cancel Bind Controller" << endl;
    cout << "6. Get Bind State" << endl;
    cout << "7. Get Nordic FW Version" << endl;
    cout << "8. Get Controller Version" << endl;
    cout << "9. Get Controller Connection State" << endl;
    cout << "10. Get Controller Battery Level" << endl;
    cout << "11. Get Left Controller IMU Data" << endl;
    cout << "12. Get Right Controller IMU Data" << endl;
    cout << "13. Stop Getting IMU Data" << endl;
    cout << "14. Perform Haptic Feedback" << endl;
    cout << "0. Quit" << endl;
    cout << "*******************" << endl;
}

int main() {
    showMenu();

    ProcessState::self()->setThreadPoolMaxThreadCount(4);

    // start the thread pool
    sp <ProcessState> ps(ProcessState::self());
    ps->startThreadPool();

    android::sp <INordic> service = INordic::getService();
    if (service == nullptr) {
        printf("Failed to get service\n");
        return -1;
    }

    service->helloWorld("Controller Service Test", [&](hidl_string result) {
        printf("%s\n", result.c_str());
    });

    std::unique_ptr <std::thread> thread_;
    std::atomic_bool is_running_{false};
    std::atomic_bool continue_running_{true};

    service->Nordic_Start();

    int32_t fd = -1;
    int32_t m_size = 0;

    service->Nordic_Get_Memory([&](hidl_handle _handle, int32_t _size) {
        fd = dup(_handle->data[0]);
        m_size = _size;
    });

    printf("fd is %d and size is %d\n", fd, m_size);

    controller_state_buffer_t *m_state_buf;

    if (fd >= 0 && m_size > 0) {
        void *address = (mmap(NULL, m_size, PROT_READ, MAP_SHARED, fd, 0));

        if (address == MAP_FAILED) {
            fd = -1;
            m_size = 0;
            printf("address is MAP_FAILED so fd is %d and size is %d\n", fd, m_size);
        } else {
            m_state_buf = (controller_state_buffer_t *) address;
        }
    }

    bool exit = 0;
    while (!exit) {
        cout << "choose: ";
        int n;
        cin >> n;
        cin.get();
        switch (n) {
            // Bind Left Controller
            case 1: {
                service->Nordic_Bind_Controller(0);
                cout << "Please check bind state with option 6" << endl;
                break;
            }
                // Bind Right Controller
            case 2: {
                service->Nordic_Bind_Controller(1);
                cout << "Please check bind state with option 6" << endl;
                break;
            }
                // Unbind Left Controller
            case 3: {
                int ret = service->Nordic_Unbind_Controller(0);
                cout << "ret: " << ret << endl;
                break;
            }
                // Unbind Right Controller
            case 4: {
                int ret = service->Nordic_Unbind_Controller(1);
                cout << "ret: " << ret << endl;
                break;
            }
                // Cancel Bind Controller
            case 5: {
                int ret = service->Nordic_Cancel_Bind();
                cout << "ret: " << ret << endl;
                break;
            }
                // Get Bind State
            case 6: {
                int ret = service->Nordic_Get_Bind_State();
                bool left = ret & 0xF;
                bool right = (ret & 0xF0) >> 4;
                cout << "Controller Bind State: " << (left ? "true" : "false") << "; " << (right ? "true" : "false") << endl;
                break;
            }
                // Get Nordic FW Version
            case 7: {
                float ret = service->Nordic_Get_Nordic_Version();
                cout << "Nordic Version: " << ret << endl;
                break;
            }
                // Get Controller Version
            case 8: {
                float left = service->Nordic_Get_Controller_Version(0);
                float right = service->Nordic_Get_Controller_Version(1);
                cout << "Controller Version: " << left << "; " << right << endl;
                break;
            }
                // Get Controller Connection State
            case 9: {
                if (m_state_buf) {
                    bool left = false;
                    bool right = false;
                    if (m_state_buf->left_position >= 0) {
                        left = true;
                    }
                    if (m_state_buf->right_position >= 0) {
                        right = true;
                    }
                    cout << "Controller Connection State: " << (left ? "true" : "false") << "; " << (right ? "true" : "false") << endl;
                } else {
                    cout << "ret: -1" << endl;
                }
                break;
            }
                // Get Controller Battery Level
            case 10: {
                if (m_state_buf) {
                    float left = 0.0f;
                    float right = 0.0f;
                    if (m_state_buf->left_position >= 0) {
                        left = m_state_buf->left_controller_state[m_state_buf->left_position].battery;
                    }

                    if (m_state_buf->right_position >= 0) {
                        right = m_state_buf->right_controller_state[m_state_buf->right_position].battery;
                    }

                    cout << "Battery level: " << left << "; " << right << endl;
                } else {
                    cout << "ret: -1" << endl;
                }
                break;
            }
                // Get Left Controller IMU Data
            case 11: {
                if (is_running_) {
                    break;
                }

                if (m_state_buf) {
                    continue_running_.store(true);
                    thread_.reset(new std::thread([&] {
                        while (continue_running_) {
                            if (m_state_buf->left_position >= 0) {
                                scControllerState state = m_state_buf->left_controller_state[m_state_buf->left_position];
                                cout << " quaternion: " << state.rotation.x << "  " << state.rotation.y << "  " << state.rotation.z << "  " << state.rotation.w
                                     << endl;
                            } else {
                                cout << "no data" << endl;
                            }
                            usleep(500000);
                        }
                    }));
                    is_running_.store(true);
                } else {
                    cout << "ret: -1" << endl;
                }
                break;
            }
                // Get Right Controller IMU Data
            case 12: {
                if (is_running_) {
                    break;
                }

                if (m_state_buf) {
                    continue_running_.store(true);
                    thread_.reset(new std::thread([&] {
                        while (continue_running_) {
                            if (m_state_buf->right_position >= 0) {
                                scControllerState state = m_state_buf->right_controller_state[m_state_buf->right_position];
                                cout << " quaternion: " << state.rotation.x << "  " << state.rotation.y << "  " << state.rotation.z << "  " << state.rotation.w
                                     << endl;
                            } else {
                                cout << "no data" << endl;
                            }
                            usleep(500000);
                        }
                    }));
                    is_running_.store(true);
                } else {
                    cout << "ret: -1" << endl;
                }
                break;
            }
                // Stop Getting IMU Data
            case 13: {
                if (!is_running_) {
                    break;
                }
                continue_running_.store(false);
                thread_->join();
                thread_ = nullptr;
                is_running_.store(false);
                cout << " Getting IMU Data Stopped! " << endl;
                showMenu();
                break;
            }
                // Perform Haptic Feedback
            case 14: {
                float ret = service->Nordic_Set_Vibration(0x8200ff0e);
                cout << "Perform Haptic Feedback: " << ret << endl;
                break;
            }
            default: {
                exit = 1;
                if (is_running_) {
                    continue_running_.store(false);
                    thread_->join();
                    thread_ = nullptr;
                    is_running_.store(false);
                }
            }
        }
    }

    if (fd > 0) {
        munmap(m_state_buf, m_size);
        close(fd);
        fd = -1;
        m_size = 0;
    }

    service->Nordic_Stop();

    IPCThreadState::self()->shutdown();

    return 0;
}