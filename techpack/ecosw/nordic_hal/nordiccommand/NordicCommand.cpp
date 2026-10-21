/*
 * controller's Command Interface for Nordic Service
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "NordicCommand.h"

#include "commom.h"
#include <errno.h>
#include <fcntl.h>
#include <ion/ion.h>
#include <linux/dma-buf.h>
#include <linux/msm_ion.h>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

struct ion_buf_handle {
    void *buffer;
    uint32_t buffer_len;
    int ion_fd;
    int ifd_data_fd;
};

struct ion_buf_handle handlebuf;

typedef struct {
    struct ion_buf_handle *handle;
    uint32_t len;
    int32_t map_fd;
    long size;
    int ion_fd;
    uint8_t *addr;
} __buffer_t;

static __buffer_t g_buffer;

typedef struct {
    uint64_t ts;
    uint32_t size;
    uint8_t data[MAX_DATA_SIZE];
} d_packet_t;

typedef struct {
    volatile int8_t c_head;
    volatile int8_t p_head;
    volatile int8_t packDS;
    d_packet_t data[MAX_PACK_SIZE];
} cp_buffer_t;

cp_buffer_t *u_packet = NULL;

int memInit = 0;
std::mutex mutex_t;

int bondState = -1;
int expectBond = -1;
int needCheckBond = -1;
int curBond = -1;

static int write_int(char const *path, int value) {
    int fd;
    static int already_warned = 0;

    fd = open(path, O_RDWR);
    if (fd >= 0) {
        char buffer[20];
        int bytes = snprintf(buffer, sizeof(buffer), "%d\n", value);
        ssize_t amt = write(fd, buffer, (size_t) bytes);
        close(fd);
        return amt == -1 ? -errno : 0;
    } else {
        if (already_warned == 0) {
            ALOGE("write_int failed to open %s\n", path);
            already_warned = 1;
        }
        return -errno;
    }
}

int buffer_alloc(__buffer_t *p_buffer) {
    void *l_buffer = NULL;

    struct dma_buf_sync buf_sync;

    int32_t ret = 0;
    p_buffer->handle = &handlebuf;
    p_buffer->len = (65536 + 4095) & (~4095);

    p_buffer->ion_fd = ion_open();
    if (p_buffer->ion_fd < 0) {
        ALOGE("buffer_alloc file open failed, error=%s", strerror(errno));
        goto ION_ALLOC_FAILED;
    }

    ret = ion_alloc_fd(p_buffer->ion_fd, p_buffer->len, 4096, ION_HEAP(ION_SYSTEM_HEAP_ID), 0, &(p_buffer->map_fd));
    if (ret) {
        ALOGE("ion_alloc_fd for heap failed %s", strerror(errno));
        ret = -1;
        goto ION_ALLOC_FAILED;
    }

    l_buffer = (unsigned char *) mmap(NULL, p_buffer->len, PROT_READ | PROT_WRITE, MAP_SHARED, p_buffer->map_fd, 0);
    if (l_buffer == MAP_FAILED) {
        ALOGE("Ion memory map failed (%s)\n", strerror(errno));
        ret = -1;
        goto ION_MAP_FAILED;
    }

    p_buffer->handle->ion_fd = p_buffer->ion_fd;
    p_buffer->handle->ifd_data_fd = p_buffer->map_fd;
    p_buffer->handle->buffer = l_buffer;
    p_buffer->handle->buffer_len = p_buffer->len;

    buf_sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
    ret = ioctl(p_buffer->map_fd, DMA_BUF_IOCTL_SYNC, &buf_sync);
    if (ret) {
        ALOGE("ION buffer mapfd failed (%s)\n", strerror(errno));
        ret = -1;
        goto ION_SYNC_FAILED;
    }

    p_buffer->addr = (uint8_t *) l_buffer;

    ALOGD("p_buffer address are %d , %d", p_buffer->addr[0], p_buffer->addr[1]);

    if (!access(MEM_FILE, F_OK)) {
        write_int(MEM_FILE, p_buffer->handle->ifd_data_fd);
    }

    ALOGD("p_buffer address are inited as %d , %d", p_buffer->addr[0], p_buffer->addr[1]);

    return 0;

    ION_SYNC_FAILED:
    if (l_buffer) {
        munmap(l_buffer, p_buffer->len);
        p_buffer->handle->buffer = NULL;
    }
    return -1;
    ION_MAP_FAILED:
    if (p_buffer->map_fd >= 0) {
        ion_close(p_buffer->map_fd);
        p_buffer->handle->ifd_data_fd = -1;
    }
    return -2;
    ION_ALLOC_FAILED:
    if (p_buffer->ion_fd >= 0) {
        ion_close(p_buffer->ion_fd);
        p_buffer->handle->ion_fd = -1;
    }
    return -3;
}

int buffer_release(__buffer_t *p_buffer) {
    struct dma_buf_sync buf_sync;
    uint32_t len = (p_buffer->handle->buffer_len + 4095) & (~4095);
    int ret = 0;

    if (!access(MEM_FILE, F_OK)) {
        write_int(MEM_FILE, -1);
    }

    buf_sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
    ret = ioctl(p_buffer->handle->ifd_data_fd, DMA_BUF_IOCTL_SYNC, &buf_sync);
    if (ret) {
        ALOGD("ioctl failed (%s)\n", strerror(errno));
    }
    if (p_buffer->handle->buffer) {
        munmap(p_buffer->handle->buffer, len);
        p_buffer->handle->buffer = NULL;
    }
    if (p_buffer->handle->ifd_data_fd >= 0) {
        ion_close(p_buffer->handle->ifd_data_fd);
        p_buffer->handle->ifd_data_fd = -1;
    }
    if (p_buffer->handle->ion_fd >= 0) {
        ion_close(p_buffer->handle->ion_fd);
        p_buffer->handle->ion_fd = -1;
    }
    return 0;
}

int mem_init(void **addr) {
    int ret = 0;
    if (g_buffer.addr) {
        *addr = g_buffer.addr;
        return 0;
    }

    ret = buffer_alloc(&g_buffer);

    *addr = g_buffer.addr;

    return ret;
}

int get_bind_state_from_buffer(char *info, int type) {
    const char *d = ":";
    char *p;
    p = strtok(info, d);
    int i = 0;
    int bind_state = 0;
    switch (type) {
        case 1:
            while (p) {
                if (i == 1) {
                    bind_state = atoi(p);
                }
                p = strtok(NULL, d);
                i++;
            }
            break;
        case 2 :
            while (p) {
                if (i == 2) {
                    bind_state |= atoi(p) << 4;
                } else if (i == 1) {
                    bind_state |= atoi(p);
                }
                p = strtok(NULL, d);
                i++;
            }
            break;
    }
    return bind_state;
}

int get_version_from_buffer(char *info) {
    const char *d = ":.";
    char *p;
    p = strtok(info, d);
    int i = 0;
    int f = 0;
    int s = 0;
    while (p) {
        if (i == 1) {
            f = atoi(p);
        } else if (i == 2) {
            s = atoi(p);
        }
        p = strtok(NULL, d);
        i++;
    }
    return (f * 10 + s);
}

static int write_command(char const *path, int commond, int needAck, int data) {
    int fd;
    static int already_warned = 0;

    if (access(REQUEST_FILE, F_OK)) {
        return -1;
    }

    mutex_t.lock();
    fd = open(path, O_RDWR);

    if (fd >= 0) {
        char buffer[TEMP_BUFFER_SIZE];
        int response = 0;
        int value = 0;

        value = needAck << 31 | commond << 24 | data;

        ALOGD("write_command command:%x\n", value);

        int bytes = snprintf(buffer, sizeof(buffer), "%x\n", value);
        ssize_t amt = write(fd, buffer, (size_t) bytes);

        ALOGD("write_command command:%x amt:%d", value, (int) amt);
        if (amt != -1 && needAck) {
            if (curBond == 17) {
                usleep(4000);
            } else {
                usleep(600000);
            }
            memset(buffer, 0, (size_t) TEMP_BUFFER_SIZE);
            amt = read(fd, buffer, (size_t) TEMP_BUFFER_SIZE);
            ALOGE("write_command response buffer:%s ", buffer);
            if (amt > -1 && commond == JS_COMMAND_BOND_STATE) {
                response = get_bind_state_from_buffer(buffer, 2);
                if (needCheckBond == -1 && response > 0) {
                    expectBond = response;
                    needCheckBond = 0;
                } else if (response > 0 && needCheckBond != -1) {
                    if (response == expectBond) {
                        needCheckBond = 0;
                    }
                }
            } else if (amt > -1 && (commond == JS_COMMAND_LEFT_PRODUCT_NAME ||
                                    commond == JS_COMMAND_RIGHT_PRODUCT_NAME)) {
                response = get_bind_state_from_buffer(buffer, 1);
            } else if (amt > -1 && (commond == JS_COMMAND_GET_VERSION ||
                                    commond == JS_COMMAND_LEFT_FW_VERSION ||
                                    commond == JS_COMMAND_RIGHT_FW_VERSION)) {
                response = get_version_from_buffer(buffer);
            } else if (amt > -1 && commond == JS_COMMAND_NORDIC_STATE) {
                response = get_bind_state_from_buffer(buffer, 1);
            } else if (amt > 0) {
                sscanf(buffer, "%d", &response);
            }
        }

        close(fd);
        mutex_t.unlock();
        return amt == -1 ? -errno : response;
    } else {
        if (already_warned == 0) {
            ALOGE("write_command failed to open %s\n", path);
            already_warned = 1;
        }
        return -errno;
    }
}

#ifdef __cplusplus
extern "C" {
#endif

int init(void) {
    int fd1 = -1;
    int fd2 = -1;
    while (!memInit) {
        if (access(MEM_FILE, F_OK) || access(REQUEST_FILE, F_OK)) {
            ALOGE("access %s or %s failed", MEM_FILE, REQUEST_FILE);
            usleep(500000);
        } else {
            fd1 = open(MEM_FILE, O_RDWR);
            fd2 = open(REQUEST_FILE, O_RDWR);
            if (fd1 >= 0 && fd2 >= 0) {
                usleep(5000);
                close(fd1);
                close(fd2);
                memInit = 1;
            } else {
                usleep(5000);
                if (fd1 >= 0) {
                    close(fd1);
                }
                if (fd2 >= 0) {
                    close(fd2);
                }
                usleep(500000);
                ALOGE("open %s or %s failed", MEM_FILE, REQUEST_FILE);
            }
        }
    }
    int ret = 0;
    ret = mem_init((void **) &u_packet);
    return ret;
}

int deinit(void) {
    if (g_buffer.addr == NULL)
        return -1;

    if (g_buffer.ion_fd > 0)
        buffer_release(&g_buffer);

    memset(&g_buffer, 0x0, sizeof(__buffer_t));

    return 0;
}

int get_nordic_version(float *ver) {
    int res = write_command(REQUEST_FILE, JS_COMMAND_GET_VERSION, JS_COMMAND_NEEDACK, 0);
    if (res > 0) {
        *ver = res * 0.1;
    } else {
        *ver = 0.0f;
    }
    return 0;
}

int get_controller_version(float *ver, int lr) {
    int res = 0;
    if (lr == 0) {
        res = write_command(REQUEST_FILE, JS_COMMAND_LEFT_FW_VERSION, JS_COMMAND_NEEDACK, lr);
    } else if (lr == 1) {
        res = write_command(REQUEST_FILE, JS_COMMAND_RIGHT_FW_VERSION, JS_COMMAND_NEEDACK, lr);
    }
    if (res > 0) {
        *ver = res * 0.1;
    } else {
        *ver = 0.0f;
    }
    return 0;
}

int bind_controller(int lr) {
    if (expectBond == -1) {
        expectBond = 0;
    }
    expectBond |= (lr ? 1 << 4 : 1);
    needCheckBond = 1;
    return write_command(REQUEST_FILE, JS_COMMAND_BOND_HAND, JS_COMMAND_NO_NEEDACK, lr);
}

int unbind_controller(int lr) {
    expectBond &= (~(lr ? 1 << 4 : 1));
    needCheckBond = 1;
    return write_command(REQUEST_FILE, JS_COMMAND_DISCONNECT_BOND, JS_COMMAND_NO_NEEDACK, lr);
}

int cancel_bind_controller(void) {
    return write_command(REQUEST_FILE, JS_COMMAND_CANCEL_BOND, JS_COMMAND_NO_NEEDACK, 0);
}

int get_bind_state(int *bind_state) {
    if (needCheckBond) {
        int ret = write_command(REQUEST_FILE, JS_COMMAND_BOND_STATE, JS_COMMAND_NEEDACK, 0);
        *bind_state = (ret >= 0 ? ret : 0);
        curBond = *bind_state;
    } else {
        *bind_state = curBond;
    }
    return 0;
}

int enter_dfu_mode(int object) {
    return write_command(REQUEST_FILE, JS_COMMAND_DFU_STATE, JS_COMMAND_NO_NEEDACK, object);
}

int set_led(int enable) {
    return write_command(REQUEST_FILE, JS_COMMAND_LED_STATE, JS_COMMAND_NO_NEEDACK, enable);
}

int set_vibration(int value) {
    return write_command(REQUEST_FILE, JS_COMMAND_VIB_STATE, JS_COMMAND_NO_NEEDACK, value);
}

int get_next_data(void *udata, uint64_t *pts, int8_t *size) {
    int8_t p_head;
    d_packet_t *pdata;
    int8_t cidx;

    //cp_buffer_t *u_packet= g_app_info.u_packet;
    p_head = u_packet->p_head;
    if (p_head < 0)return -1;

    if (u_packet->c_head == u_packet->p_head)return -2;

    cidx = (u_packet->c_head + 1) % MAX_PACK_SIZE;
    pdata = &u_packet->data[cidx];
    *pts = pdata->ts;
    *size = pdata->size;

    if (*size != 26) {
        u_packet->c_head = cidx;
        return -3;
    }
    memcpy((void *) udata, (char *) pdata->data, pdata->size);

    u_packet->c_head = cidx;

    return 0;
}

int get_data_size(void) {
    //cp_buffer_t *u_packet= g_app_info.u_packet;

    int8_t p_head = u_packet->p_head;
    int8_t c_head = u_packet->c_head;

    if (c_head == p_head)return 0;

    if (p_head < 0)return 0;

    if (p_head > c_head) {
        return p_head - c_head;
    } else if (p_head < c_head) {
        return (MAX_PACK_SIZE - c_head + p_head);

    }
    return 0;
}

int get_the_last_data(void *udata, uint64_t *pts, int8_t *size) {
    int8_t p_head;
    int8_t c_head;

    d_packet_t *pdata;
    int8_t cidx;

    c_head = u_packet->c_head;
    p_head = u_packet->p_head;

    if (p_head < 0)return -1;

    if (c_head == p_head)return -2;

    cidx = p_head;

    pdata = &u_packet->data[cidx];

    *pts = pdata->ts;

    *size = pdata->size;

    if (*size != 26) {
        u_packet->c_head = cidx;
        return -3;
    }

    memcpy((void *) udata, (char *) pdata->data, pdata->size);

    u_packet->c_head = cidx;

    return 0;
}

#ifdef __cplusplus
};
#endif