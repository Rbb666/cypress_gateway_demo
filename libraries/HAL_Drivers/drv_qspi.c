/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-06     Rbb666       first version
 */

#include "drv_common.h"
#include "drv_qspi.h"

#ifdef RT_USING_QSPI
#if defined(BSP_USING_QSPI)
    #error "Please define at least one BSP_USING_QSPIx"
#endif

#define DRV_DEBUG
#define LOG_TAG "drv.qspi"
#include <drv_log.h>

#define BUS_WIDTH_ONE               (1u)
#define BUS_WIDTH_TWO               (2u)
#define BUS_WIDTH_FOUR              (4u)

#define NUM_BITS_PER_BYTE           (8u)

struct ifx_qspi_bus
{
    struct rt_spi_bus qspi_device;

    cyhal_qspi_t obj;
    char *bus_name;
};

struct rt_spi_bus _qspi_bus1;
struct ifx_qspi_bus _ifx_qspi_bus;

struct ifx_hw_spi_cs
{
    rt_uint16_t pin;
};

static cyhal_qspi_bus_width_t _get_bus_width(rt_uint8_t bus_width_in)
{
    cyhal_qspi_bus_width_t bus_width_out;

    switch(bus_width_in)
    {
        case BUS_WIDTH_ONE:
            bus_width_out = CYHAL_QSPI_CFG_BUS_SINGLE;
            break;

        case BUS_WIDTH_TWO:
            bus_width_out = CYHAL_QSPI_CFG_BUS_DUAL;
            break;

        case BUS_WIDTH_FOUR:
            bus_width_out = CYHAL_QSPI_CFG_BUS_QUAD;
            break;

        default:
            bus_width_out = CYHAL_QSPI_CFG_BUS_SINGLE;
    }

    return bus_width_out;
}

static void qspi_send_cmd(struct ifx_qspi_bus *qspi_bus, struct rt_qspi_message *message)
{
    RT_ASSERT(qspi_bus != RT_NULL);
    RT_ASSERT(message != RT_NULL);

    cyhal_qspi_command_t *qspi_cmd;

}

static rt_uint32_t qspi_xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    int len;
    int result = RT_EOK;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    struct rt_qspi_message *qspi_message = (struct rt_qspi_message *)message;
    struct ifx_qspi_bus *qspi_bus = device->bus->parent.user_data;

    const rt_uint8_t *sndb = message->send_buf;
    rt_uint8_t *rcvb = message->recv_buf;
    rt_int32_t length = message->length;

    /* send data */
    if (sndb)
    {
        qspi_send_cmd(qspi_bus, qspi_message);

        if (cy_serial_flash_qspi_write(qspi_message->address.content, length, sndb) == RT_EOK)
        {
            len = length;
        }
        else
        {
            LOG_E("QSPI send data failed!");
            goto __exit;
        }
    }
    else if (rcvb) /* recv data */
    {
        if (cy_serial_flash_qspi_read(qspi_message->address.content, length, rcvb) == RT_EOK)
        {
            len = length;
        }
        else
        {
            LOG_E("QSPI recv data failed!");
            goto __exit;
        }
    }

__exit:
    return len;
}

static int ifx_qspi_init(struct rt_qspi_device *device, struct rt_qspi_configuration *qspi_cfg)
{
    int result = RT_EOK;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(qspi_cfg != RT_NULL);

    struct rt_spi_configuration *cfg = &qspi_cfg->parent;
    struct ifx_qspi_bus *qspi_bus = device->parent.bus->parent.user_data;

    result = cy_serial_flash_qspi_init(smifMemConfigs[MEM_SLOT_NUM], CYBSP_QSPI_D0,
                                       CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC,
                                       CYBSP_QSPI_SCK, CYBSP_QSPI_SS, QSPI_BUS_FREQUENCY_HZ);

    if (result != RT_EOK)
    {
        LOG_E("qspi init failed (%d)!", result);
    }

    return result;
}

static rt_err_t qspi_configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    struct rt_qspi_device *qspi_device = (struct rt_qspi_device *)device;

    return ifx_qspi_init(qspi_device, &qspi_device->config);
}

static const struct rt_spi_ops ifx_qspi_ops =
{
    .configure = qspi_configure,
    .xfer = qspi_xfer,
};

static int ifx_qspi_register_bus(struct ifx_qspi_bus *qspi_bus, const char *name)
{
    RT_ASSERT(qspi_bus != RT_NULL);
    RT_ASSERT(name != RT_NULL);

    _qspi_bus1.parent.user_data = qspi_bus;
    return rt_qspi_bus_register(&_qspi_bus1, name, &ifx_qspi_ops);
}

/**
 * @brief  this function attach device to qspi bus.
 * @param  device_name      qspi device name
 * @param  pin              qspi cs pin number
 * @param  data_line_width  qspi data lines width, such as 1, 2, 4
 * @param  enter_qspi_mode  callback function that lets flash enter qspi mode
 * @param  exit_qspi_mode   callback function that lets flash exit qspi mode
 * @retval 0 : success
 *        -1 : failed
 */
rt_err_t ifx_qspi_bus_attach_device(const char *bus_name, const char *device_name, rt_uint32_t pin, rt_uint8_t data_line_width, void (*enter_qspi_mode)(), void (*exit_qspi_mode)())
{
    struct rt_qspi_device *qspi_device = RT_NULL;
    rt_err_t result = RT_EOK;

    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);
    RT_ASSERT(data_line_width == 1 || data_line_width == 2 || data_line_width == 4);

    qspi_device = (struct rt_qspi_device *)rt_malloc(sizeof(struct rt_qspi_device));

    if (qspi_device == RT_NULL)
    {
        LOG_E("no memory, qspi bus attach device failed!");
        result = RT_ENOMEM;
        goto __exit;
    }

    qspi_device->enter_qspi_mode = enter_qspi_mode;
    qspi_device->exit_qspi_mode = exit_qspi_mode;
    qspi_device->config.qspi_dl_width = data_line_width;

    result = rt_spi_bus_attach_device(&qspi_device->parent, device_name, bus_name, RT_NULL);

__exit:

    if (result != RT_EOK)
    {
        if (qspi_device)
        {
            rt_free(qspi_device);
        }
    }

    return result;
}

static int rt_hw_qspi_bus_init(void)
{
    return ifx_qspi_register_bus(&_ifx_qspi_bus, "qspi1");
}

INIT_BOARD_EXPORT(rt_hw_qspi_bus_init);

static void test_spi_falsh(void)
{
    cy_serial_flash_qspi_init(smifMemConfigs[MEM_SLOT_NUM], CYBSP_QSPI_D0,
                              CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC,
                              CYBSP_QSPI_SCK, CYBSP_QSPI_SS, QSPI_BUS_FREQUENCY_HZ);

    rt_kprintf("\r\nTotal Flash Size: %u bytes\r\n", cy_serial_flash_qspi_get_size());
}
MSH_CMD_EXPORT(test_spi_falsh, test spi falsh);
#endif /* BSP_USING_QSPI */
