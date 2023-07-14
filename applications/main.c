/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-29     Rbb666       first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "drv_gpio.h"

#include "sht3x.h"
#include "bh1750.h"

#define LED_PIN     GET_PIN(0, 1)
#define I2C_BUS     "i2c4"

sht3x_device_t sht3x_dev = RT_NULL;
bh1750_device_t bh1750_dev = RT_NULL;
float light_data;

int main(void)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
}

void sensor_thread_entry(void *parameter)
{
    while (1)
    {
        sht3x_read_singleshot(sht3x_dev);
        light_data = bh1750_read_light(bh1750_dev);

        rt_thread_mdelay(1000);
    }
}

static int sensor_init(void)
{
    rt_thread_t tid;
    rt_uint8_t sht_addr = SHT3X_ADDR_PD ;

    sht3x_dev = sht3x_init(I2C_BUS, sht_addr);
    if (!sht3x_dev)
    {
        rt_kprintf("sht3x probe failed, check input args\n");
    }
    else
    {
        rt_kprintf("sht3x probed, addr:0x%x\n", sht_addr) ;
    }

    bh1750_dev = bh1750_init(I2C_BUS);
    if (!bh1750_dev)
    {
        rt_kprintf("sht3x probe failed, check input args\n");
    }
    else
    {
        rt_kprintf("sht3x probed, addr:0x%x\n", sht_addr) ;
    }

    tid = rt_thread_create("sensor", sensor_thread_entry, RT_NULL,
                           RT_MAIN_THREAD_STACK_SIZE, 25, 20);
    RT_ASSERT(tid != RT_NULL);

    rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(sensor_init);
