/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2022-07-21     Rbbb666      first version
 */

#include "board.h"
#include "drv_sdio.h"

#ifdef BSP_USING_SDIO1
    #include <dfs_fs.h>
#endif

#ifdef BSP_USING_SDMMC

#define DRV_DEBUG
#define DBG_TAG              "drv.sdio"
#ifdef DRV_DEBUG
    #define DBG_LVL               DBG_LOG
#else
    #define DBG_LVL               DBG_INFO
#endif /* DRV_DEBUG */
#include <rtdbg.h>

#define RTHW_SDIO_LOCK(_sdio)   rt_mutex_take(&_sdio->mutex, RT_WAITING_FOREVER)
#define RTHW_SDIO_UNLOCK(_sdio) rt_mutex_release(&_sdio->mutex);

static struct rt_mmcsd_host *host1;

struct sdio_pkg
{
    struct rt_mmcsd_cmd *cmd;
    void *buff;
    rt_uint32_t flag;
};

struct rthw_sdio
{
    struct rt_mmcsd_host *host;
    struct ifx_sdio_des sdio_des;
    struct rt_event event;
    struct rt_mutex mutex;
    struct sdio_pkg *pkg;
};

ALIGN(SDIO_ALIGN_LEN)
static rt_uint8_t cache_buf[SDIO_BUFF_SIZE];

/**
  * @brief  This function send command.
  * @param  sdio rthw_sdio
  * @param  pkg  sdio package
  * @retval None
  */
static void rthw_sdio_send_command(struct rthw_sdio *sdio, struct sdio_pkg *pkg)
{
    struct rt_mmcsd_cmd *cmd = pkg->cmd;
    struct rt_mmcsd_data *data = cmd->data;
    rt_uint32_t reg_cmd;

    rt_event_control(&sdio->event, RT_IPC_CMD_RESET, RT_NULL);
    /* save pkg */
    sdio->pkg = pkg;

    LOG_D("CMD:%d ARG:0x%08x RES:%s%s%s%s%s%s%s%s%s rw:%c len:%d blksize:%d\n",
          cmd->cmd_code,
          cmd->arg,
          resp_type(cmd) == RESP_NONE ? "NONE"  : "",
          resp_type(cmd) == RESP_R1  ? "R1"  : "",
          resp_type(cmd) == RESP_R1B ? "R1B"  : "",
          resp_type(cmd) == RESP_R2  ? "R2"  : "",
          resp_type(cmd) == RESP_R3  ? "R3"  : "",
          resp_type(cmd) == RESP_R4  ? "R4"  : "",
          resp_type(cmd) == RESP_R5  ? "R5"  : "",
          resp_type(cmd) == RESP_R6  ? "R6"  : "",
          resp_type(cmd) == RESP_R7  ? "R7"  : "",
          data ? (data->flags & DATA_DIR_WRITE ?  'w' : 'r') : '-',
          data ? data->blks * data->blksize : 0,
          data ? data->blksize : 0
         );

    /* data pre configuration */
    if (data != RT_NULL)
    {
        cyhal_sdhc_data_config_t  data_config;

        data_config.data_ptr = sdio->pkg->buff;
        data_config.block_size = data->blksize;
        data_config.number_of_blocks = data->blks * data->blksize;

        cyhal_sdhc_config_data_transfer(&sdio->sdio_des.hsd, &data_config);

    }
}

/**
  * @brief  This function send sdio request.
  * @param  sdio  rthw_sdio
  * @param  req   request
  * @retval None
  */
static void rthw_sdio_request(struct rt_mmcsd_host *host, struct rt_mmcsd_req *req)
{
    struct sdio_pkg pkg;
    struct rthw_sdio *sdio = host->private_data;
    struct rt_mmcsd_data *data;

    RTHW_SDIO_LOCK(sdio);

    if (req->cmd != RT_NULL)
    {
        rt_memset(&pkg, 0, sizeof(pkg));
        data = req->cmd->data;
        pkg.cmd = req->cmd;

        if (data != RT_NULL)
        {
            rt_uint32_t size = data->blks * data->blksize;

            RT_ASSERT(size <= SDIO_BUFF_SIZE);

            if (data->flags & DATA_DIR_WRITE)
            {
                rt_memcpy(cache_buf, data->buf, size);
            }
        }

        rthw_sdio_send_command(sdio, &pkg);
    }

    if (req->stop != RT_NULL)
    {
        rt_memset(&pkg, 0, sizeof(pkg));
        pkg.cmd = req->stop;
        rthw_sdio_send_command(sdio, &pkg);
    }

    RTHW_SDIO_UNLOCK(sdio);

    mmcsd_req_complete(sdio->host);
}

/**
  * @brief  This function config sdio.
  * @param  host    rt_mmcsd_host
  * @param  io_cfg  rt_mmcsd_io_cfg
  * @retval None
  */
static void rthw_sdio_iocfg(struct rt_mmcsd_host *host, struct rt_mmcsd_io_cfg *io_cfg)
{
    rt_uint32_t temp, clk_src;
    rt_uint32_t clk = io_cfg->clock;
    struct rthw_sdio *sdio = host->private_data;

    LOG_D("clk:%dK width:%s%s%s power:%s%s%s",
          clk / 1000,
          io_cfg->bus_width == MMCSD_BUS_WIDTH_8 ? "8" : "",
          io_cfg->bus_width == MMCSD_BUS_WIDTH_4 ? "4" : "",
          io_cfg->bus_width == MMCSD_BUS_WIDTH_1 ? "1" : "",
          io_cfg->power_mode == MMCSD_POWER_OFF ? "OFF" : "",
          io_cfg->power_mode == MMCSD_POWER_UP ? "UP" : "",
          io_cfg->power_mode == MMCSD_POWER_ON ? "ON" : ""
         );

    RTHW_SDIO_LOCK(sdio);

    if (io_cfg->bus_width == MMCSD_BUS_WIDTH_8)
    {
        temp = SDMMC_BUS_WIDE_8B;
    }
    else if (io_cfg->bus_width == MMCSD_BUS_WIDTH_4)
    {
        temp = SDMMC_BUS_WIDE_4B;
    }
    else
    {
        temp = SDMMC_BUS_WIDE_1B;
    }

    RTHW_SDIO_UNLOCK(sdio);
}

void HAL_SD_MspInit(sd_bd_config_t *bd_cfg)
{
    rt_kprintf("sd_bd_get_default_config(%p)\n", (void *)bd_cfg);
    RT_ASSERT(NULL != bd_cfg);

    bd_cfg->sdhc_config.enableLedControl = false;
    bd_cfg->sdhc_config.lowVoltageSignaling = false;
    bd_cfg->sdhc_config.isEmmc   = false;
    bd_cfg->sdhc_config.busWidth = SDMMC_BUS_WIDE_4B;

    bd_cfg->cmd = NC;
    bd_cfg->clk = NC;
    bd_cfg->data0 = NC;
    bd_cfg->data1 = NC;
    bd_cfg->data2 = NC;
    bd_cfg->data3 = NC;
    bd_cfg->data4 = NC;
    bd_cfg->data5 = NC;
    bd_cfg->data6 = NC;
    bd_cfg->data7 = NC;
    bd_cfg->card_detect = NC;
    bd_cfg->io_volt_sel = NC;
    bd_cfg->card_if_pwr_en = NC;
    bd_cfg->card_mech_write_prot = NC;
    bd_cfg->led_ctrl = NC;
    bd_cfg->card_emmc_reset = NC;
    bd_cfg->block_clk = NULL;

#ifdef CYBSP_SDHC_CMD
    bd_cfg->cmd = CYBSP_SDHC_CMD;
#endif

#ifdef CYBSP_SDHC_CLK
    bd_cfg->clk = CYBSP_SDHC_CLK;
#endif

#ifdef CYBSP_SDHC_IO0
    bd_cfg->data0 = CYBSP_SDHC_IO0;
#endif

#ifdef CYBSP_SDHC_IO1
    bd_cfg->data1 = CYBSP_SDHC_IO1;
#endif

#ifdef CYBSP_SDHC_IO2
    bd_cfg->data2 = CYBSP_SDHC_IO2;
#endif

#ifdef  CYBSP_SDHC_IO3
    bd_cfg->data3 = CYBSP_SDHC_IO3;
#endif

#ifdef CYBSP_SDHC_DETECT
    bd_cfg->card_detect = CYBSP_SDHC_DETECT;
#endif

    int result = RT_EOK;

    /* Initialize the SD card */
    result = cyhal_sdhc_init((cyhal_sdhc_t *)&bd_cfg->sdhc_obj, &bd_cfg->sdhc_config, bd_cfg->cmd, bd_cfg->clk,
                             bd_cfg->data0, bd_cfg->data1, bd_cfg->data2, bd_cfg->data3,
                             bd_cfg->data4, bd_cfg->data5, bd_cfg->data6, bd_cfg->data7,
                             bd_cfg->card_detect, bd_cfg->io_volt_sel, bd_cfg->card_if_pwr_en,
                             bd_cfg->card_mech_write_prot, bd_cfg->led_ctrl, bd_cfg->card_emmc_reset, bd_cfg->block_clk);

    if (result != RT_EOK)
    {
        RT_ASSERT(0);
    }
}

static const struct rt_mmcsd_host_ops ops =
{
    rthw_sdio_request,
    rthw_sdio_iocfg,
    RT_NULL,
    RT_NULL,
};

struct rt_mmcsd_host *sdio_host_create(struct ifx_sdio_des *sdio_des)
{
    struct rt_mmcsd_host *host;
    struct rthw_sdio *sdio = RT_NULL;

    if (sdio_des == RT_NULL)
    {
        return RT_NULL;
    }

    sdio = rt_malloc(sizeof(struct rthw_sdio));

    if (sdio == RT_NULL)
    {
        LOG_E("malloc rthw_sdio fail");
        return RT_NULL;
    }

    rt_memset(sdio, 0, sizeof(struct rthw_sdio));

    host = mmcsd_alloc_host();

    if (host == RT_NULL)
    {
        LOG_E("alloc host fail");
        goto err;
    }

    rt_memcpy(&sdio->sdio_des, sdio_des, sizeof(struct ifx_sdio_des));

    rt_event_init(&sdio->event, "sdio1", RT_IPC_FLAG_FIFO);
    rt_mutex_init(&sdio->mutex, "sdio1", RT_IPC_FLAG_PRIO);

    /* set host defautl attributes */
    host->ops = &ops;
    host->freq_min = 400 * 1000;
    host->freq_max = SDIO_MAX_FREQ;
    host->valid_ocr = 0X00FFFF80; /* The voltage range supported is 1.65v-3.6v */
#ifndef SDIO_USING_1_BIT
    host->flags = MMCSD_BUSWIDTH_4 | MMCSD_MUTBLKWRITE | MMCSD_SUP_SDIO_IRQ;
#else
    host->flags = MMCSD_MUTBLKWRITE | MMCSD_SUP_SDIO_IRQ;
#endif
    host->max_seg_size = SDIO_BUFF_SIZE;
    host->max_dma_segs = 1;
    host->max_blk_size = 512;
    host->max_blk_count = 512;

    /* link up host and sdio */
    sdio->host = host;
    host->private_data = sdio;

    /* ready to change */
    mmcsd_change(host);

    return host;
err:

    if (sdio)
    {
        rt_free(sdio);
    }

    return RT_NULL;
}

int rt_hw_sdio_init(void)
{
#ifdef BSP_USING_SDIO1
    struct ifx_sdio_des sdio_des1;

    sd_bd_config_t sd_bd_cfg;
    HAL_SD_MspInit(&sd_bd_cfg);

    host1 = sdio_host_create(&sdio_des1);

    if (host1 == RT_NULL)
    {
        LOG_E("host create fail");
        return RT_NULL;
    }

#endif

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_sdio_init);

#ifdef BSP_USING_SDIO1
int mnt_init(void)
{
    rt_device_t sd = RT_NULL;

    rt_thread_delay(RT_TICK_PER_SECOND);

    sd = rt_device_find("sd0");

    if (sd == RT_NULL)
    {
        rt_kprintf("can't find sd0 device!\n");
        return RT_ERROR;
    }

    if (dfs_mount("sd0", "/", "elm", 0, 0) != 0)
    {
        rt_kprintf("file system mount failed!\n");
    }
    else
    {
        rt_kprintf("file system mount success!\n");
    }

    return RT_EOK;
}
//INIT_APP_EXPORT(mnt_init);
#endif /* BSP_USING_SDIO1 */

#endif /* BSP_USING_SDMMC */
