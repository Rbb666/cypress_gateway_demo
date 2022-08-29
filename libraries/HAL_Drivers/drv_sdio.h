/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-21     Rbbb666      first version
 */

#ifndef __DRV_SDIO_H__
#define __DRV_SDIO_H__

#include <rtthread.h>
#include "rtdevice.h"
#include <drv_common.h>
#include <drivers/mmcsd_core.h>
#include <drivers/sdio.h>

#define SDIO_ALIGN_LEN 4

#define SDMMC_BUS_WIDE_8B   8U
#define SDMMC_BUS_WIDE_4B   4U
#define SDMMC_BUS_WIDE_1B   1U

#ifndef SDIO_BUFF_SIZE
    #define SDIO_BUFF_SIZE      (4096)
#endif

#ifndef SDIO_MAX_FREQ
    #define SDIO_MAX_FREQ       (100UL * 1000UL * 1000UL)
#endif

typedef struct
{
    cyhal_sdhc_t sdhc_obj; /**< Object for use with the SDHC HAL driver. */
    cyhal_sdhc_config_t sdhc_config; /**< Card configuration structure to be passed to the HAL driver. */
    cyhal_gpio_t cmd;   /**< The pin connected to the command signal. */
    cyhal_gpio_t clk;   /**< The pin connected to the clock signal. */
    cyhal_gpio_t data0; /**< The pin connected to the data0 signal. */
    cyhal_gpio_t data1; /**< The pin connected to the data1 signal. */
    cyhal_gpio_t data2; /**< The pin connected to the data2 signal. */
    cyhal_gpio_t data3; /**< The pin connected to the data3 signal. */
    cyhal_gpio_t data4; /**< The pin connected to the data4 signal; pass NC when unused. */
    cyhal_gpio_t data5; /**< The pin connected to the data5 signal; pass NC when unused. */
    cyhal_gpio_t data6; /**< The pin connected to the data6 signal; pass NC when unused. */
    cyhal_gpio_t data7; /**< The pin connected to the data7 signal; pass NC when unused. */
    cyhal_gpio_t card_detect; /**< The pin connected to the card detect signal. */
    cyhal_gpio_t io_volt_sel; /**< The pin connected to the voltage select signal. */
    cyhal_gpio_t card_if_pwr_en; /**< The pin connected to the card interface power enable signal. */
    cyhal_gpio_t card_mech_write_prot; /**< The pin connected to the write protect signal. */
    cyhal_gpio_t led_ctrl; /**< The pin connected to the LED control signal. */
    cyhal_gpio_t card_emmc_reset; /**< The pin connected to the eMMC card reset signal. */
    cyhal_clock_t *block_clk;
} sd_bd_config_t;

struct ifx_sdio_des
{
    struct ifx_sdio *hw_sdio;
    cyhal_sdhc_t hsd;
};

/* ifx sdio dirver class */
struct ifx_sdio_class
{
    struct ifx_sdio_des *des;
    struct rt_mmcsd_host host;
};

struct sdhci_cmd_t
{
    rt_uint32_t cmdidx;
    rt_uint32_t cmdarg;
    rt_uint32_t resptype;
    rt_uint32_t response[4];
};

struct sdhci_data_t
{
    rt_uint8_t * buf;
    rt_uint32_t flag;
    rt_uint32_t blksz;
    rt_uint32_t blkcnt;
};

#endif /* __DRV_SDIO_H__ */
