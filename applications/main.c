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

#define LED_PIN     GET_PIN(0, 1)

int main(void)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    for (;;)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
#include "sensor_st_hts221.h"

int hts221_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.dev_name = "i2c4";
    cfg.intf.arg = (void *)HTS221_ADDR_DEFAULT;
    cfg.irq_pin.pin = PIN_NONE;

    rt_hw_hts221_init("hts221", &cfg);
    return 0;
}
INIT_APP_EXPORT(hts221_port);

//#if defined(PKG_USING_LSM6DSM)
//#include "sensor_lsm6dsm.h"

//int lsm6dsm_port(void)
//{
//    struct rt_sensor_config cfg;
//    cfg.intf.dev_name = "i2c4";
//    rt_hw_lsm6dsm_init("lsm6dsm", &cfg);
//    return 0;
//}
//INIT_APP_EXPORT(lsm6dsm_port);
//#endif /*PKG_USING_LSM6DSl*/
