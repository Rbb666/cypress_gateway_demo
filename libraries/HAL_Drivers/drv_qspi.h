/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     shelton      first version
 */

#ifndef __DRV_QSPI_H__
#define __DRV_QSPI_H__

#include <rtthread.h>
#include <board.h>

#include "cycfg_qspi_memslot.h"
#include "cy_serial_flash_qspi.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MEM_SLOT_NUM            (0u)    /* Slot number of the memory to use */
#define QSPI_BUS_FREQUENCY_HZ   (50000000lu)

#ifdef BSP_USING_QSPI
#ifndef QSPI1_BUS_CONFIG
#define QSPI1_BUS_CONFIG     \
    {                        \
        .qspi_x = QSPI1,     \
                  .bus_name = "qspi1", \
    }
#endif /* QSPI1_BUS_CONFIG */
#endif /* BSP_USING_QSPI */

rt_err_t ifx_qspi_bus_attach_device(const char *bus_name, const char *device_name, rt_uint32_t pin, rt_uint8_t data_line_width, void (*enter_qspi_mode)(), void (*exit_qspi_mode)());

#ifdef __cplusplus
}
#endif

#endif /* __DRV_QSPI_H__ */
