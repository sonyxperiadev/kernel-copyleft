/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
*
* File Name: focaltech_debug.c
*
* Author: Focaltech Driver Team
*
* Created: 2023-12-01
*
* Abstract: Fw Debug
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_REG_FW_DEBUG_EN                 0x9E
#define FTS_REG_TPINFO                      0x96
#define FTS_REG_DBGCFG                      0x9D

#define DEFAULT_VAL_REG01                   0xFFFF
#define DEFAULT_MAX_FRAME_NUM               10000
#define MAX_SIZE_TP_INFO                    8
#define MAX_SIZE_DBG_CFG                    32
#define MAX_COUNT_READ_REGFB                3

#define ST_VER                              0x10
#define DDT_FWDBG_TOUCH                     0x400709F0
#define DDT_FWDBG_STYLUS                    0x400708F0


/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
typedef enum {
    FRAME_WAITQ_DEFAULT,
    FRAME_WAITQ_WAIT,
    FRAME_WAITQ_WAKEUP,
} FRAME_WAITQ_FLAG;

/* If evt_cnt is EVT_CNT_TOUCH, then use touch size to read, if evt_cnt is greater
 * than EVT_CNT_STYLUS, then use stylus size to read.
 * If event is touch event, then evt_cnt = EVT_CNT_TOUCH, if event is stylus event,
 * evt_cnt = EVT_CNT_PEN; If evt_cnt = EVT_CNT_PEN, and next event is touch, then evt_cnt++
 */
typedef enum {
    EVT_CNT_TOUCH = 0,
    EVT_CNT_STYLUS = 2,
    EVT_CNT_STYLUS_TO_TOUCH = 4,
} EVENT_TOUCH_PEN_COUNT;

struct fwdbg_frame {
    u32 size    : 24;
    u32 st      : 8;
    u32 type;
    u64 tv;
    unsigned char value[0];
};

struct fwdbg_queue {
    int head;
    int tail;
    int count;
    int max_count;
    int elem_size;
    u8 *buffer;
    struct mutex mutexq;
};

struct fwdbg_config {
    int total_len;
    int stylus_total_len;
    int dbgoff;
    int dbghdr_len;
    int diff_len;
    int addinfo_len;
    int regfa_len;
    int regfb_len;
    int tx;
    int rx;
};

struct fts_fwdbg {
    struct fts_ts_data *ts_data;
    struct proc_dir_entry *proc_fwdbg;
    struct mutex mutex;
    wait_queue_head_t frame_waitq;
    struct fwdbg_queue q;
    struct fwdbg_config cfg;
    int max_frame_num; //maximum frame number kept in memory
    int frame_size; // is equal to size of one frame
    int touch_size_bak;
    int proc_ppos;
    int frame_waitq_flag;
    int reg01_val;
    int evt_cnt; // touch/pen event count, used for setting touch size
    u8 tp_info[MAX_SIZE_TP_INFO];
    u8 dbg_cfg[MAX_SIZE_DBG_CFG];
    u8 *dbg_touch_buf;
    char *proc_frame; /* save a frame value comes from queue */
    unsigned char *regfb_val;
    unsigned char *regfa_val;

    bool queue_stop;
    bool frame_logging;
    bool frame_block;
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
static struct fts_fwdbg *fts_fwdbg_data;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static u64 fwdbg_get_timestamp(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
    ktime_t tv;
    tv = ktime_get_real() / 1000;
    return (u64)tv;
#else
    struct timeval tv;
    do_gettimeofday(&tv);
    return (u64)(((u64)tv.tv_sec * 1000000) + tv.tv_usec);
#endif
}

static int dbgq_open(struct fwdbg_queue *q, int max_framenum, int frame_size)
{
    if (!q || !max_framenum || !frame_size) {
        FTS_ERROR("q is null/max_framenum(%d)/frame_size(%d) is invalid", max_framenum, frame_size);
        return -EINVAL;
    }

    if (q->buffer) {
        vfree(q->buffer);
        q->buffer = NULL;
    }

    if (!q->buffer) {
        q->head = q->tail = q->count = 0;
        q->max_count = max_framenum;
        q->elem_size = frame_size;
        FTS_INFO("quque,max_count=%d,elem_size=%d", q->max_count, q->elem_size);
        q->buffer = vmalloc(q->max_count * q->elem_size);
        if (!q->buffer) {
            FTS_ERROR("malloc queue buffer failed");
            return -ENOMEM;
        }
        memset(q->buffer, 0, q->max_count * q->elem_size);
    }
    return 0;
}

static bool dbgq_full(struct fwdbg_queue *q)
{
    return q->count == q->max_count;
}

static bool dbgq_empty(struct fwdbg_queue *q)
{
    return q->count == 0;
}

static int dbgq_enqueue(struct fwdbg_queue *q, int evt_type, u8 *val, u32 size, u64 timestamp)
{
    struct fwdbg_frame *tail_elem = NULL;

    if (!q || !val || !q->buffer || ((size + sizeof(struct fwdbg_frame)) > q->elem_size)) {
        FTS_ERROR("q/val/buffer is null, size:%d+%d>%d", size, (int)sizeof(struct fwdbg_frame), q->elem_size);
        return -EINVAL;
    }

    mutex_lock(&(q->mutexq));
    tail_elem = (struct fwdbg_frame *)&q->buffer[q->tail * q->elem_size];
    tail_elem->size = (size & 0x00FFFFFF) + sizeof(struct fwdbg_frame) - sizeof(u32);
    tail_elem->st = ST_VER;
    if ((evt_type == TOUCH_PEN) || (evt_type == TOUCH_PEN_v2))
        tail_elem->type = DDT_FWDBG_STYLUS;
    else
        tail_elem->type = DDT_FWDBG_TOUCH;
    tail_elem->tv = timestamp;
    memcpy(tail_elem->value, val, size);
    q->tail = (q->tail + 1) % q->max_count;
    if (dbgq_full(q)) {
        q->head = (q->head + 1) % q->max_count;
    } else {
        q->count++;
    }
    mutex_unlock(&(q->mutexq));
    return 0;
}

static int dbgq_dequeue(struct fwdbg_queue *q, u8 *val)
{
    if (!q || !val || !q->buffer) {
        FTS_ERROR("q/val/buffer is null");
        return -EINVAL;
    }

    mutex_lock(&(q->mutexq));
    if (dbgq_empty(q)) {
        mutex_unlock(&(q->mutexq));
        return 1;
    }
    memcpy(val, &q->buffer[q->head * q->elem_size], q->elem_size);
    q->head = (q->head + 1) % q->max_count;
    q->count--;
    mutex_unlock(&(q->mutexq));
    return 0;
}

/*****************************************************************************
*  Name: dbgq_dequeue_to_proc
*  Brief: dequeue frames from queue, and send it to proc read buffer.
*  Input: @q
*         @buff_maxcount, the maximum frame count
*  Output:@buff, address of user space buffer
*  Return:size that send to the buff, or 0 if queue is null, or error code
*****************************************************************************/
static int dbgq_dequeue_to_proc(struct fwdbg_queue *q, char __user *buff, int buff_maxcount)
{
    int valid_count = 0;
    int i = 0;
    int offset = 0;
    int active_size = 0;
    struct fwdbg_frame *head_elem = NULL;
    if (!q || !buff || !q->buffer || !buff_maxcount) {
        FTS_ERROR("q/buff/buffer is null/buff_maxcount is 0");
        return -EINVAL;
    }

    mutex_lock(&(q->mutexq));
    if (dbgq_empty(q)) {
        mutex_unlock(&(q->mutexq));
        return 0;
    }
    valid_count = (buff_maxcount > q->count) ? q->count : buff_maxcount;
    for (i = 0; i < valid_count; i++) {
        head_elem = (struct fwdbg_frame *)&q->buffer[q->head * q->elem_size];
        active_size = head_elem->size + sizeof(u32);
        if (copy_to_user(buff + offset, (char *)head_elem, active_size)) {
            FTS_ERROR("copy debug frame(%d) to user failed", i);
            mutex_unlock(&(q->mutexq));
            return offset;
        }
        offset += active_size;
        q->head = (q->head + 1) % q->max_count;
        q->count--;
    }
    mutex_unlock(&(q->mutexq));
    return offset;
}

static int dbgq_release(struct fwdbg_queue *q)
{
    q->head = q->tail = q->count = 0;
    q->max_count = q->elem_size = 0;
    if (q && q->buffer) {
        vfree(q->buffer);
        q->buffer = NULL;
    }
    return 0;
}

/*****************************************************************************
*  Name: proc_get_one_frame
*  Brief: Get a frame, and send the frame data to proc read buffer.
*  Input: @dbg
*         @buff_maxsize, the maximum size of buff
*  Output:@buff, address of user space buffer
*  Return:size that send to the buff, or 0 if queue is null, or error code
*****************************************************************************/
static int proc_get_one_frame(struct fts_fwdbg *dbg, char __user *buff, int buff_maxsize)
{
    int ret = 0;
    int frame_remaining_size = 0;
    int valid_size = 0;
    int active_size = 0;
    struct fwdbg_frame *elem = NULL;

    if (!dbg || !buff || !dbg->proc_frame || !buff_maxsize) {
        FTS_ERROR("dbg/buff/proc_frame is null/buff_maxsize is 0");
        return -EINVAL;
    }

    if (dbg->proc_ppos == 0) {
        ret = dbgq_dequeue(&dbg->q, dbg->proc_frame);
        if (ret < 0) {
            FTS_ERROR("get a frame from queue failed");
            return ret;
        } else if (ret == 1) {
            /* queque is null */
            return 0;
        }
    }

    elem = (struct fwdbg_frame *)dbg->proc_frame;
    active_size = elem->size + sizeof(u32);
    frame_remaining_size = active_size - dbg->proc_ppos;
    valid_size = (frame_remaining_size > buff_maxsize) ? buff_maxsize : frame_remaining_size;
    if (copy_to_user(buff, &dbg->proc_frame[dbg->proc_ppos], valid_size)) {
        FTS_ERROR("copy debug frame to user failed");
        return -EFAULT;
    }
    dbg->proc_ppos = (dbg->proc_ppos + valid_size) % active_size;

    return valid_size;
}

static int fts_fwdbg_get_cfg(struct fts_fwdbg *dbg)
{
    int ret = 0;
    u8 cmd = 0;
    u8 *tp_info = dbg->tp_info;
    u8 *dbg_cfg = dbg->dbg_cfg;

    cmd = FTS_REG_DBGCFG;
    ret = fts_read(&cmd, 1, dbg_cfg, MAX_SIZE_DBG_CFG);
    if (ret < 0) {
        FTS_ERROR("read debug config failed");
        return ret;
    }

    dbg->cfg.total_len = (dbg_cfg[2] << 8) + dbg_cfg[3];
    dbg->cfg.stylus_total_len = (dbg_cfg[14] << 8) + dbg_cfg[15];
    dbg->cfg.dbgoff = dbg_cfg[4];
    dbg->cfg.dbghdr_len = dbg_cfg[5];
    dbg->cfg.diff_len = (dbg_cfg[6] << 8) + dbg_cfg[7];
    dbg->cfg.addinfo_len = (dbg_cfg[8] << 8) + dbg_cfg[9];
    dbg->cfg.regfb_len = (dbg_cfg[10] << 8) + dbg_cfg[11];
    dbg->cfg.regfa_len = (dbg_cfg[12] << 8) + dbg_cfg[13];
    if (dbg_cfg[16] && dbg_cfg[17]) {
        dbg->cfg.tx = dbg_cfg[16];
        dbg->cfg.rx = dbg_cfg[17];
    } else {
        cmd = FTS_REG_TPINFO;
        ret = fts_read(&cmd, 1, tp_info, MAX_SIZE_TP_INFO);
        if (ret < 0) {
            FTS_ERROR("read tp info failed");
            return ret;
        }
        dbg->cfg.tx = tp_info[2];
        dbg->cfg.rx = tp_info[3];
    }
    FTS_INFO("cfg,total=%d,%d,hdr=%d,diff=%d,addinfo=%d", dbg->cfg.total_len, dbg->cfg.stylus_total_len,
             dbg->cfg.dbghdr_len, dbg->cfg.diff_len, dbg->cfg.addinfo_len);
    FTS_INFO("cfg,tx=%d,rx=%d,fb=%d,fa=%d", dbg->cfg.tx, dbg->cfg.rx, dbg->cfg.regfb_len, dbg->cfg.regfa_len);
    return 0;
}

static int fts_fwdbg_enable(struct fts_fwdbg *dbg, int value)
{
    int ret = 0;
    if (!dbg || !dbg->ts_data || !value) {
        FTS_ERROR("fwdbg/ts_data is null/value(%d) is invalid", value);
        return -EINVAL;
    }

    ret = fts_fwdbg_get_cfg(dbg);
    if (ret < 0) {
        FTS_ERROR("get cfg from tp failed");
        return ret;
    }

    if (dbg->cfg.stylus_total_len > dbg->cfg.total_len) {
        FTS_ERROR("stylus len(%d) > touch(%d)", dbg->cfg.stylus_total_len, dbg->cfg.total_len);
        return -EIO;
    }

    dbg->dbg_touch_buf = kzalloc(dbg->cfg.total_len, GFP_KERNEL);
    if (NULL == dbg->dbg_touch_buf) {
        FTS_ERROR("dbg_touch_buf kzalloc fail");
        return -ENOMEM;
    }

    dbg->frame_size = dbg->cfg.total_len + sizeof(struct fwdbg_frame);
    FTS_INFO("FwDebug enable,max_frame_num=%d,frame_size=%d", dbg->max_frame_num, dbg->frame_size);
    if (!dbg->frame_logging) {
        ret = dbgq_open(&dbg->q, dbg->max_frame_num, dbg->frame_size);
        if (ret < 0) {
            FTS_ERROR("dbgq_open failed");
            goto dbgen_err;
        }
    }

    if (dbg->cfg.regfb_len) {
        if (dbg->regfb_val) {
            vfree(dbg->regfb_val);
            dbg->regfb_val = NULL;
        }
        dbg->regfb_val = vmalloc(dbg->cfg.regfb_len);
        if (!dbg->regfb_val) {
            ret = -ENOMEM;
            goto dbgen_err;
        }
    }

    if (dbg->cfg.regfa_len) {
        if (dbg->regfa_val) {
            vfree(dbg->regfa_val);
            dbg->regfa_val = NULL;
        }
        dbg->regfa_val = vmalloc(dbg->cfg.regfa_len);
        if (!dbg->regfa_val) {
            ret = -ENOMEM;
            goto dbgen_err;
        }
    }

    ret = fts_write_reg(FTS_REG_FW_DEBUG_EN, value);
    if (ret < 0) {
        FTS_ERROR("write FwDebug to enable failed");
        goto dbgen_err;
    }

    return 0;

dbgen_err:
    if (dbg->dbg_touch_buf) {
        kfree(dbg->dbg_touch_buf);
        dbg->dbg_touch_buf = NULL;
    }

    if (dbg->regfa_val) {
        vfree(dbg->regfa_val);
        dbg->regfa_val = NULL;
    }
    if (dbg->regfb_val) {
        vfree(dbg->regfb_val);
        dbg->regfb_val = NULL;
    }
    dbgq_release(&dbg->q);
    fts_write_reg(FTS_REG_FW_DEBUG_EN, 0);
    return ret;
}

static int fts_fwdbg_disable(struct fts_fwdbg *dbg)
{
    int ret = 0;
    if (!dbg || !dbg->ts_data) {
        FTS_ERROR("fwdbg/ts_data is null");
        return -EINVAL;
    }

    if (!dbg->frame_logging) {
        dbgq_release(&dbg->q);
    }

    if (dbg->dbg_touch_buf) {
        kfree(dbg->dbg_touch_buf);
        dbg->dbg_touch_buf = NULL;
    }

    if (dbg->regfa_val) {
        vfree(dbg->regfa_val);
        dbg->regfa_val = NULL;
    }
    if (dbg->regfb_val) {
        vfree(dbg->regfb_val);
        dbg->regfb_val = NULL;
    }

    ret = fts_write_reg(FTS_REG_FW_DEBUG_EN, 0);
    if (ret < 0) {
        FTS_ERROR("write FwDebug to disable failed");
    }
    return ret;
}

static void fts_logging_frame(struct fwdbg_config *cfg, u8 *frame_buf, u32 size, u64 timestamp)
{
    int i = 0;
    int n = 0;
    int index = 0;
    char logbuf[512] = { 0 };

    if (!cfg | !frame_buf || (size < cfg->dbgoff))
        return ;

    if ((size != cfg->total_len) && (size != cfg->stylus_total_len))
        return;

    FTS_DEBUG("logging a frame,timestamp=%lld", timestamp);
    for (i = 0; i < cfg->dbgoff; i++) {
        n += snprintf(logbuf + n, 512 - n, "%02x,", frame_buf[i]);
        if (n >= 512) break;
    }
    FTS_DEBUG("%s", logbuf);

    n = 0;
    index = cfg->dbgoff;
    for (i = 0; i < cfg->dbghdr_len; i++) {
        n += snprintf(logbuf + n, 512 - n, "%02x,", frame_buf[index + i]);
        if (n >= 512) break;
    }
    FTS_DEBUG("%s", logbuf);

    index = cfg->dbgoff + cfg->dbghdr_len;
    FTS_DEBUG("%d", frame_buf[index]);

    n = 0;
    index = cfg->dbgoff + cfg->dbghdr_len + 1;
    /* print diff and addinfo */
    for (i = 0; i < size - index; i += 2) {
        n += snprintf(logbuf + n, 512 - n, "%d,", (short)((frame_buf[index + i] << 8) + frame_buf[index + i + 1]));
        if (n >= 512) break;
        else if ((((i + 2) >> 1) % cfg->rx) == 0) {
            FTS_DEBUG("%s", logbuf);
            n = 0;
        }
    }
}

static void fts_logging_regfb(struct fts_fwdbg *dbg, u64 timestamp)
{
    int ret = 0;
    int i = 0;
    int n = 0;
    u8 cmd = 0xFB;
    char logbuf[512] = { 0 };

    if (!dbg || ! dbg->regfb_val || !dbg->cfg.regfb_len || !dbg->cfg.rx) {
        FTS_ERROR("dbg/regfb_val/regfb_len(%d)/rx(%d) is invalid", dbg->cfg.regfb_len, dbg->cfg.rx);
        return ;
    }

    ret = fts_read(&cmd, 1, dbg->regfb_val, dbg->cfg.regfb_len);
    if (ret < 0) {
        FTS_ERROR("read regfb failed,ret=%d", ret);
        return ;
    }

    FTS_DEBUG("logging regfb,timestamp=%lld", timestamp);
    for (i = 0; i < dbg->cfg.regfb_len; i += 2) {
        n += snprintf(logbuf + n, 512 - n, "%d,", (short)((dbg->regfb_val[i] << 8) + dbg->regfb_val[i + 1]));
        if (n >= 512) break;
        else if ((((i + 2) >> 1) % dbg->cfg.rx) == 0) {
            FTS_DEBUG("%s", logbuf);
            n = 0;
        }
    }
}

static void fts_logging_regfa(struct fts_fwdbg *dbg, u64 timestamp)
{
    int ret = 0;
    int i = 0;
    int n = 0;
    u8 cmd = 0xFA;
    int line_count = 0;
    char logbuf[512] = { 0 };

    if (!dbg || ! dbg->regfa_val || !dbg->cfg.regfa_len || !dbg->cfg.rx) {
        FTS_ERROR("dbg/regfa_val/regfa_len(%d)/rx(%d) is invalid", dbg->cfg.regfa_len, dbg->cfg.rx);
        return ;
    }

    ret = fts_read(&cmd, 1, dbg->regfa_val, dbg->cfg.regfa_len);
    if (ret < 0) {
        FTS_ERROR("read regfa failed,ret=%d", ret);
        return ;
    }

    FTS_DEBUG("logging regfa,timestamp=%lld", timestamp);
    line_count = dbg->cfg.rx * 2;
    for (i = 0; i < dbg->cfg.regfa_len; i++) {
        n += snprintf(logbuf + n, 512 - n, "%02X,", dbg->regfa_val[i]);
        if (n >= 512) break;
        else if (((i + 1) % line_count) == 0) {
            FTS_DEBUG("%s", logbuf);
            n = 0;
        }
    }
}

static void fts_logging_reg(struct fts_fwdbg *dbg, u8 addr, u8 *buf, u32 len)
{
    int i = 0;
    int n = 0;
    char logbuf[512] = { 0 };

    if (!dbg || !buf || !len) {
        FTS_ERROR("dbg/buf/len(%d) is invalid", len);
        return ;
    }

    n += snprintf(logbuf + n, 512 - n, "reg%02X=", addr);
    for (i = 0; i < len; i++) {
        n += snprintf(logbuf + n, 512 - n, "%02X ", buf[i]);
        if (n >= 512) break;
    }

    FTS_DEBUG("%s", logbuf);
}

/* proc node:fts_fwdbg */
static ssize_t fts_fwdbg_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
    int read_byte_num = (int)count;
    int cnt = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
    struct fts_fwdbg *dbg = pde_data(file_inode(filp));
#else
    struct fts_fwdbg *dbg = PDE_DATA(file_inode(filp));
#endif
    if (!dbg || !dbg->ts_data || !dbg->ts_data->fwdbg_support) {
        FTS_ERROR("dbg/ts_data is null/fwdbg isn't support");
        return -EINVAL;
    }

    if (dbg->frame_logging) {
        FTS_ERROR("frame logging is null,not return frame data");
        return -EINVAL;
    }

    if ((dbg->frame_block) && (read_byte_num < dbg->frame_size)) {
        FTS_ERROR("in block mode, proc count(%d) < frame size(%d)", read_byte_num, dbg->frame_size);
        return -EINVAL;
    }

    if (dbgq_empty(&dbg->q) && dbg->frame_block) {
        dbg->queue_stop = false;
        dbg->frame_waitq_flag = FRAME_WAITQ_WAIT;
        wait_event_interruptible(dbg->frame_waitq, dbg->frame_waitq_flag == FRAME_WAITQ_WAKEUP);
    }

    if (read_byte_num >= dbg->frame_size)
        cnt = dbgq_dequeue_to_proc(&dbg->q, buff, read_byte_num / dbg->frame_size);
    else
        cnt = proc_get_one_frame(dbg, buff, read_byte_num);

    //FTS_DEBUG("cnt=%d", cnt);
    return cnt;
}

static int fts_fwdbg_open(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
    struct fts_fwdbg *dbg = pde_data(inode);
#else
    struct fts_fwdbg *dbg = PDE_DATA(inode);
#endif
    if (!dbg || !dbg->ts_data || !dbg->ts_data->fwdbg_support) {
        FTS_ERROR("dbg/ts_data is null");
        return -EINVAL;
    }

    FTS_DEBUG("frame,block=%d,logging=%d,size=%d,macount=%d,queuecount=%d", dbg->frame_block,
              dbg->frame_logging, dbg->frame_size, dbg->max_frame_num, dbg->q.count);
    if ((!dbg->frame_logging) && (dbg->q.elem_size != dbg->frame_size)) {
        FTS_ERROR("elem_size(%d) != frame_size(%d)", dbg->q.elem_size, dbg->frame_size);
        return -EINVAL;
    }

    dbg->proc_ppos = 0;
    dbg->queue_stop = false;
    if (!dbg->frame_block) {
        dbg->queue_stop = true;
        fts_logging_reg(dbg, FTS_REG_FW_DEBUG_EN, (u8 *)&dbg->ts_data->fwdbg_value, 1);
        fts_logging_reg(dbg, FTS_REG_DBGCFG, dbg->dbg_cfg, MAX_SIZE_DBG_CFG);
        fts_logging_reg(dbg, FTS_REG_TPINFO, dbg->tp_info, MAX_SIZE_TP_INFO);
        /* get fa/fb info */
        if (dbg->cfg.regfb_len) {
            fts_logging_regfb(dbg, fwdbg_get_timestamp());
            fts_logging_regfb(dbg, fwdbg_get_timestamp());
            fts_logging_regfb(dbg, fwdbg_get_timestamp());
        }
        if (dbg->cfg.regfa_len) fts_logging_regfa(dbg, fwdbg_get_timestamp());

        if (dbg->proc_frame) {
            vfree(dbg->proc_frame);
            dbg->proc_frame = NULL;
        }
        dbg->proc_frame = vmalloc(dbg->frame_size);
    }

    return 0;
}

static int fts_fwdbg_release(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
    struct fts_fwdbg *dbg = pde_data(inode);
#else
    struct fts_fwdbg *dbg = PDE_DATA(inode);
#endif
    if (!dbg) {
        FTS_ERROR("dbg is null");
        return -EINVAL;
    }

    if (dbg->proc_frame) {
        vfree(dbg->proc_frame);
        dbg->proc_frame = NULL;
    }
    dbg->proc_ppos = 0;
    dbg->queue_stop = false;
    return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops fts_fwdbg_fops = {
    .proc_open = fts_fwdbg_open,
    .proc_read = fts_fwdbg_read,
    .proc_release = fts_fwdbg_release,
};
#else
static const struct file_operations fts_fwdbg_fops = {
    .open = fts_fwdbg_open,
    .read = fts_fwdbg_read,
    .release = fts_fwdbg_release,
};
#endif


/* sysfs node:fts_fwdbg_mode */
static ssize_t fts_fwdbg_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    if (!dbg || !ts_data) {
        FTS_ERROR("dbg/ts_data is null");
        return count;
    }
    mutex_lock(&dbg->mutex);
    fts_read_reg(FTS_REG_FW_DEBUG_EN, &val);
    count = snprintf(buf, PAGE_SIZE, "FwDebug support:%d,value:0x%x\n", ts_data->fwdbg_support, ts_data->fwdbg_value);
    count += snprintf(buf + count, PAGE_SIZE, "Reg(0x9E)=0x%x\n", val);
    mutex_unlock(&dbg->mutex);
    return count;
}

static ssize_t fts_fwdbg_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int n = 0;
    int value = 0;
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    if (!dbg || !ts_data) {
        FTS_ERROR("dbg/ts_data is null");
        return count;
    }

    mutex_lock(&dbg->mutex);
    n = sscanf(buf, "%d", &value);
    if ((n == 1) && (!!value ^ ts_data->fwdbg_support)) {
        if (value) {
            if (0 == fts_fwdbg_enable(dbg, value)) {
                ts_data->fwdbg_value = (u8)value;
                ts_data->fwdbg_support = ENABLE;
            }
        } else {
            ts_data->fwdbg_support = DISABLE;
            fts_fwdbg_disable(dbg);
        }
    } else FTS_INFO("n(%d)!=/value(%d)==fwdbg_support(%d)", n, !!value, ts_data->fwdbg_support);
    mutex_unlock(&dbg->mutex);
    return count;
}

/* sysfs node:fts_fwdbg_maxcount */
static ssize_t fts_fwdbg_maxcount_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    if (!dbg) {
        FTS_ERROR("dbg is null");
        return count;
    }
    mutex_lock(&dbg->mutex);
    count = snprintf(buf, PAGE_SIZE, "FwDebug,maximum frame count:%d\n", dbg->max_frame_num);
    mutex_unlock(&dbg->mutex);
    return count;
}

static ssize_t fts_fwdbg_maxcount_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int n = 0;
    int value = 0;
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    if (!dbg || !dbg->ts_data) {
        FTS_ERROR("dbg is null");
        return count;
    }

    mutex_lock(&dbg->mutex);
    n = sscanf(buf, "%d", &value);
    if ((n == 1) && (value > 0) && !dbg->ts_data->fwdbg_support) {
        FTS_INFO("maximum frame count: %d->%d", dbg->max_frame_num, value);
        dbg->max_frame_num = value;
    } else FTS_INFO("n(%d)!=1/value(%d)=0/fwdbg_support(%d)!=0", n, value, dbg->ts_data->fwdbg_support);
    mutex_unlock(&dbg->mutex);
    return count;
}

/* sysfs node:fts_fwdbg_logging */
static ssize_t fts_fwdbg_logging_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    if (!dbg) {
        FTS_ERROR("dbg is null");
        return count;
    }
    mutex_lock(&dbg->mutex);
    count = snprintf(buf, PAGE_SIZE, "FwDebug,frame logging:%d\n", dbg->frame_logging);
    mutex_unlock(&dbg->mutex);
    return count;
}

static ssize_t fts_fwdbg_logging_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int n = 0;
    int value = 0;
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    if (!dbg) {
        FTS_ERROR("dbg is null");
        return count;
    }

    mutex_lock(&dbg->mutex);
    n = sscanf(buf, "%d", &value);
    if ((n == 1) && !dbg->ts_data->fwdbg_support) {
        FTS_INFO("frame logging: %d->%d", dbg->frame_logging, !!value);
        dbg->frame_logging = !!value;
    } else FTS_INFO("n(%d)!=1/fwdbg_support(%d)!=0", n, dbg->ts_data->fwdbg_support);
    mutex_unlock(&dbg->mutex);
    return count;
}

/* sysfs node:fts_fwdbg_block */
static ssize_t fts_fwdbg_block_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    if (!dbg) {
        FTS_ERROR("dbg is null");
        return count;
    }
    mutex_lock(&dbg->mutex);
    count = snprintf(buf, PAGE_SIZE, "frame block:%d\n", dbg->frame_block);
    mutex_unlock(&dbg->mutex);
    return count;
}

static ssize_t fts_fwdbg_block_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int n = 0;
    int value = 0;
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    if (!dbg) {
        FTS_ERROR("dbg is null");
        return count;
    }

    mutex_lock(&dbg->mutex);
    n = sscanf(buf, "%d", &value);
    if (n == 1) {
        FTS_INFO("frame block: %d->%d", dbg->frame_block, !!value);
        dbg->frame_block = !!value;
    }
    mutex_unlock(&dbg->mutex);
    return count;
}

static DEVICE_ATTR(fts_fwdbg_mode, S_IRUGO | S_IWUSR, fts_fwdbg_mode_show, fts_fwdbg_mode_store);
static DEVICE_ATTR(fts_fwdbg_maxcount, S_IRUGO | S_IWUSR, fts_fwdbg_maxcount_show, fts_fwdbg_maxcount_store);
static DEVICE_ATTR(fts_fwdbg_logging, S_IRUGO | S_IWUSR, fts_fwdbg_logging_show, fts_fwdbg_logging_store);
static DEVICE_ATTR(fts_fwdbg_block, S_IRUGO | S_IWUSR, fts_fwdbg_block_show, fts_fwdbg_block_store);
static struct attribute *fts_fwdbg_attrs[] = {
    &dev_attr_fts_fwdbg_mode.attr,
    &dev_attr_fts_fwdbg_maxcount.attr,
    &dev_attr_fts_fwdbg_logging.attr,
    &dev_attr_fts_fwdbg_block.attr,
    NULL,
};
static struct attribute_group fts_fwdbg_group = {.attrs = fts_fwdbg_attrs,};

static void fts_fwdbg_work_func(struct work_struct *work)
{
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data, fwdbg_work.work);
    if (ts_data && ts_data->fwdbg_support && dbg) {
        if (dbg->cfg.regfb_len) fts_logging_regfb(dbg, fwdbg_get_timestamp());
        if (dbg->cfg.regfa_len) fts_logging_regfa(dbg, fwdbg_get_timestamp());

    }
}

void fts_fwdbg_handle_reset(struct fts_ts_data *ts_data)
{
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    if (ts_data && ts_data->ts_workqueue && ts_data->fwdbg_support && dbg) {
        dbg->reg01_val = DEFAULT_VAL_REG01;
        if (dbg->cfg.regfb_len || dbg->cfg.regfa_len)
            queue_delayed_work(ts_data->ts_workqueue, &ts_data->fwdbg_work, msecs_to_jiffies(200));
    }
}

static int fts_fwdbg_recovery(struct fts_ts_data *ts_data)
{
    int i = 0;
    u8 regval = 0xFF;

    if (ts_data->fwdbg_support) {
        for (i = 0; i < FTS_MAX_RETRIES_WRITEREG; i++) {
            fts_read_reg(FTS_REG_FW_DEBUG_EN, &regval);
            if (regval == ts_data->fwdbg_value)
                break;
            fts_write_reg(FTS_REG_FW_DEBUG_EN, ts_data->fwdbg_value);
            fts_msleep(1);
        }

        if (i >= FTS_MAX_RETRIES_WRITEREG)
            FTS_ERROR("set fwdbg mode to %x failed,reg_val:%x", ts_data->fwdbg_value, regval);
        else if (i > 0)
            FTS_INFO("set fwdbg mode to %x successfully", ts_data->fwdbg_value);
    }
    return 0;
}

static int fts_fwdbg_readdata(struct fts_fwdbg *dbg, u8 *buf, u32 size, int input_result)
{
    u64 timestamp = 0;
    int touch_etype = 0;

    if (!dbg || !buf || (size > dbg->cfg.total_len) || (size < dbg->cfg.stylus_total_len)) {
        FTS_ERROR("ts_data/buf/dbg is null");
        return -EINVAL;
    }

    touch_etype = ((buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F);
    timestamp = fwdbg_get_timestamp();
    if (dbg->frame_logging) {
        fts_logging_frame(&dbg->cfg, buf, size, timestamp);
    } else if (!dbg->queue_stop) {
        dbgq_enqueue(&dbg->q, touch_etype, buf, size, timestamp);
        if (dbg->frame_waitq_flag == FRAME_WAITQ_WAIT) {
            dbg->frame_waitq_flag = FRAME_WAITQ_WAKEUP;
            wake_up_interruptible(&dbg->frame_waitq);
        }
    }

    if (dbg->reg01_val == DEFAULT_VAL_REG01) {
        /*set reg01_val when touch input is valid*/
        if (input_result == 0) dbg->reg01_val = buf[0];
    } else if (buf[0] != dbg->reg01_val) {
        FTS_DEBUG("reg01:%x->%x", dbg->reg01_val, buf[0]);
        if (!dbg->frame_logging) fts_logging_frame(&dbg->cfg, buf, size, timestamp);
        if (dbg->cfg.regfb_len) fts_logging_regfb(dbg, timestamp);
        dbg->reg01_val = buf[0];
    }

    /* check the touch size of next frame */
    if ((touch_etype == TOUCH_PEN) || (touch_etype == TOUCH_PEN_v2)) {
        dbg->evt_cnt = EVT_CNT_STYLUS;
    } else if (dbg->evt_cnt >= EVT_CNT_STYLUS) {
        dbg->evt_cnt++;
        if (dbg->evt_cnt >= EVT_CNT_STYLUS_TO_TOUCH) dbg->evt_cnt = EVT_CNT_TOUCH;
    }
    return 0;
}

int fts_fwdbg_irq_handler(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    u8 reg_addr = 0x01;
    u8 *touch_buf = NULL;
    u32 touch_size = 0;

    if (!ts_data || !dbg || !dbg->dbg_touch_buf) {
        FTS_ERROR("ts_data/dbg/dbg_touch_buf is null");
        return -EINVAL;
    }

    touch_buf = dbg->dbg_touch_buf;
    touch_size = (dbg->evt_cnt == EVT_CNT_TOUCH) ? dbg->cfg.total_len :  dbg->cfg.stylus_total_len;
    ret = fts_input_read_data(ts_data, reg_addr, touch_buf, touch_size);
    if (ret)
        return TOUCH_IGNORE;

#if FTS_PSENSOR_EN
    if (ts_data->proximity_mode) {
        if (fts_proximity_readdata(ts_data) == FTS_RETVAL_IGNORE_TOUCHES) {
            ret = TOUCH_IGNORE;
            goto ret_irq;
        }
    }
#endif

#if FTS_FOD_EN
    if (ts_data->fod_mode) {
        if (fts_fod_readdata(ts_data) == FTS_RETVAL_IGNORE_TOUCHES) {
            ret = TOUCH_IGNORE;
            goto ret_irq;
        }
    }
#endif

    if (ts_data->suspended && ts_data->gesture_support) {
        if (fts_gesture_readdata(ts_data, touch_buf) == FTS_RETVAL_IGNORE_TOUCHES) {
            ret = TOUCH_IGNORE;
            goto ret_irq;
        }
    }

    if ((touch_buf[1] == 0xFF) && (touch_buf[2] == 0xFF) && (touch_buf[3] == 0xFF) && (touch_buf[4] == 0xFF)) {
        FTS_INFO("touch buff is 0xff, FW initialized");
        fts_release_all_finger();
        fts_tp_state_recovery(ts_data);
        fts_fwdbg_recovery(ts_data);
        fts_fwdbg_handle_reset(ts_data);
        ret = TOUCH_FW_INIT;
        goto ret_irq;
    }

    if (ts_data->suspended) {
        FTS_INFO("In suspend state, not report touch points");
        ret = TOUCH_IGNORE;
        goto ret_irq;
    }

    if (((touch_buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F) == TOUCH_FWDBG) {
        ret = TOUCH_IGNORE;
        goto ret_irq;
    }

    ret = fts_input_report_buffer(ts_data, touch_buf);
    if (ret < 0) {
        FTS_ERROR("report buffer failed");
        goto ret_irq;
    }

    ret = 0;
ret_irq:
    fts_fwdbg_readdata(dbg, touch_buf, touch_size, ret);
    return ret;
}

int fts_fwdbg_init(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct fts_fwdbg *dbg = NULL;

    FTS_FUNC_ENTER();
    dbg = kzalloc(sizeof(struct fts_fwdbg), GFP_KERNEL);
    if (!dbg) {
        FTS_ERROR("allocate memory for fwdbg failed");
        return -ENOMEM;
    }
    fts_fwdbg_data = dbg;
    dbg->ts_data = ts_data;
    dbg->max_frame_num = DEFAULT_MAX_FRAME_NUM;
    dbg->frame_block = false;
    dbg->frame_logging = false;
    dbg->frame_waitq_flag = FRAME_WAITQ_DEFAULT;
    dbg->reg01_val = DEFAULT_VAL_REG01;
    mutex_init(&dbg->mutex);
    mutex_init(&dbg->q.mutexq);
    init_waitqueue_head(&dbg->frame_waitq);

    dbg->proc_fwdbg = proc_create_data("fts_fwdbg", 0777, NULL, &fts_fwdbg_fops, dbg);
    if (NULL == dbg->proc_fwdbg) {
        FTS_ERROR("create proc_fwdbg entry failed");
    }

    ret = sysfs_create_group(&ts_data->dev->kobj, &fts_fwdbg_group);
    if (ret) {
        FTS_ERROR("create fwdebug sysfs node failed");
        sysfs_remove_group(&ts_data->dev->kobj, &fts_fwdbg_group);
    }

    if (ts_data->ts_workqueue) INIT_DELAYED_WORK(&ts_data->fwdbg_work, fts_fwdbg_work_func);
    FTS_FUNC_EXIT();
    return 0;
}

int fts_fwdbg_exit(struct fts_ts_data *ts_data)
{
    struct fts_fwdbg *dbg = fts_fwdbg_data;
    FTS_FUNC_ENTER();
    if (dbg) {
        if (dbg->proc_fwdbg) proc_remove(dbg->proc_fwdbg);
        if (dbg->regfa_val) {
            vfree(dbg->regfa_val);
            dbg->regfa_val = NULL;
        }

        if (dbg->regfb_val) {
            vfree(dbg->regfb_val);
            dbg->regfb_val = NULL;
        }

        if (dbg->dbg_touch_buf) {
            kfree(dbg->dbg_touch_buf);
            dbg->dbg_touch_buf = NULL;
        }

        if (dbg->q.buffer) {
            vfree(dbg->q.buffer);
            dbg->q.buffer = NULL;
        }

        if (dbg->proc_frame) {
            vfree(dbg->proc_frame);
            dbg->proc_frame = NULL;
        }

        kfree_safe(dbg);
    }

    if (ts_data) {
        sysfs_remove_group(&ts_data->dev->kobj, &fts_fwdbg_group);
        cancel_delayed_work_sync(&ts_data->fwdbg_work);
    }
    FTS_FUNC_EXIT();
    return 0;
}
