/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-25     Rbb666       the first version
 */

#include <rtdevice.h>
#include <board.h>
#include <onenet.h>
#include <math.h>

#include "ui.h"

#define DBG_ENABLE
#define DBG_COLOR
#define DBG_SECTION_NAME "onenet.demo"
#if ONENET_DEBUG
    #define DBG_LEVEL DBG_LOG
#else
    #define DBG_LEVEL DBG_INFO
#endif /* ONENET_DEBUG */

#include <rtdbg.h>
#ifdef FINSH_USING_MSH
#include <finsh.h>

#define WIFI_INIT_THREAD_STACK_SIZE      (1024 * 1)
#define WIFI_INIT_THREAD_PRIORITY        (RT_THREAD_PRIORITY_MAX / 2)
#define WIFI_INIT_WAIT_TIME              (rt_tick_from_millisecond(1000))
#define ONENRT_INIT_WAIT_TIME            (rt_tick_from_millisecond(500))

#define PWM_DEV_NAME "pwm0"
#define PWM_DEV_CHANNEL 0
#define GET_DUTY_CYCLE(x)       (1 * 1000 * 1000 - x * 10 * 1000)

#define LED1_PIN     GET_PIN(0, 1)

struct wifi_info
{
    char *wifi_ssid;
    char *wifi_password;
};
static struct wifi_info wifi;

static struct rt_device_pwm *pwm_dev;

rt_bool_t onenet_sync_flag = RT_FALSE;
static rt_bool_t led_rev_data = RT_FALSE;

/* onenet mqtt command response callback and analysis function */
static void onenet_cmd_rsp_cb(uint8_t *recv_data, size_t recv_size, uint8_t **resp_data, size_t *resp_size)
{
    LOG_D("Recv data is %.*s\n", recv_size, recv_data);

    cJSON *cjson_root = RT_NULL, *cjson_ctl = RT_NULL;
    cjson_root = cJSON_Parse((const char *) recv_data);

    cjson_ctl = cJSON_GetObjectItem(cjson_root, "Led");
    if (cjson_ctl->type == cJSON_Number)
    {
        led_rev_data = cjson_ctl->valueint;

        LOG_D("led state:%d\n", led_rev_data);

        led_rev_data == RT_TRUE ?
        rt_pin_write(LED1_PIN, PIN_LOW) :
        rt_pin_write(LED1_PIN, PIN_HIGH);

        if (onenet_mqtt_upload_digit("Led_status", led_rev_data))
        {
            LOG_E("upload has an error, stop uploading");
        }
    }

    cjson_ctl = cJSON_GetObjectItem(cjson_root, "Led_pwm");
    if (cjson_ctl->type == cJSON_Number)
    {
        uint8_t led_rev_data = 0;

        led_rev_data = cjson_ctl->valueint;

        LOG_D("led pwm value:%d\n", led_rev_data);

        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 1 * 1000 * 1000, GET_DUTY_CYCLE(led_rev_data));

        if (onenet_mqtt_upload_digit("Led_pwm", led_rev_data))
        {
            LOG_E("upload has an error, stop uploading");
        }
    }

    if (NULL != cjson_root)
    {
        cJSON_Delete(cjson_root);
        cjson_root = NULL;
    }
}

static void onenet_upload_entry(void *parameter)
{
    int humi_val = 0;
    int tempature_val = 0;
    int lux_val = 0;

    lv_chart_series_t *ui_Chart_series_1 = lv_chart_add_series(ui_Chart,
                                           lv_color_hex(0xFE1068), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t *ui_Chart_series_2 = lv_chart_add_series(ui_Chart,
                                           lv_color_hex(0x43FF88), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t *ui_Chart_series_3 = lv_chart_add_series(ui_Chart,
                                           lv_color_hex(0xFCFF00), LV_CHART_AXIS_PRIMARY_Y);

    while (1)
    {
        humi_val = round(1.0 * rand() / RAND_MAX * 20 + 40);
        tempature_val = round(1.0 * rand() / RAND_MAX * 10 + 20);
        lux_val = round(1.0 * rand() / RAND_MAX * 5 + 50);

        lv_chart_set_next_value(ui_Chart, ui_Chart_series_1, humi_val);
        lv_chart_set_next_value(ui_Chart, ui_Chart_series_2, tempature_val);
        lv_chart_set_next_value(ui_Chart, ui_Chart_series_3, lux_val);

        if (onenet_mqtt_upload_digit("temperature", tempature_val) < 0)
        {
            LOG_E("upload has an error, stop uploading");
        }

        if (onenet_mqtt_upload_digit("humidity", humi_val) < 0)
        {
            LOG_E("upload has an error, stop uploading");
        }

        rt_thread_delay(rt_tick_from_millisecond(5 * 1000));
    }
}

static int onenet_upload_cycle(void)
{
    rt_thread_t tid;

    pwm_dev = (struct rt_device_pwm *) rt_device_find(PWM_DEV_NAME);

    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("PWM init failed! can't find %s device!\n", PWM_DEV_NAME);
        RT_ASSERT(0);
    }

    tid = rt_thread_create("onenet_send", onenet_upload_entry, RT_NULL, 2 * 1024, 20, 5);
    if (tid)
    {
        rt_thread_startup(tid);
    }

    return RT_EOK;
}

static void clear_onenet_ui_sta(void)
{
    onenet_mqtt_upload_digit("Led_pwm", 0);
    onenet_mqtt_upload_digit("Led_status", 0);
}

#define KEY0_PIN_NUM    GET_PIN(6, 2)

static void irq_callback()
{
    rt_kprintf("key down\n");
    _ui_state_modify(ui_BTN_Power, LV_STATE_CHECKED, _UI_MODIFY_STATE_TOGGLE);
}

int key_example(void)
{
    rt_pin_mode(KEY0_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(KEY0_PIN_NUM, PIN_IRQ_MODE_FALLING, irq_callback, RT_NULL);
    rt_pin_irq_enable(KEY0_PIN_NUM, PIN_IRQ_ENABLE);
    return 0;
}
MSH_CMD_EXPORT(key_example, key_example);

static void wifi_init_thread_entry(void *parameter)
{
    LOG_I("Try to connect SSID[%s] Password[%s]", wifi.wifi_ssid, wifi.wifi_password);

    rt_wlan_connect(wifi.wifi_ssid, wifi.wifi_password);

    while (!rt_wlan_is_connected())
    {
        rt_kprintf("@reconnect ap...\n");
        rt_wlan_connect(wifi.wifi_ssid, wifi.wifi_password);
        rt_thread_delay(WIFI_INIT_WAIT_TIME);
    }

    rt_thread_delay(ONENRT_INIT_WAIT_TIME);

    /* set wi-fi icon show */
    lv_obj_clear_flag(ui_IMG_Wifi, LV_OBJ_FLAG_HIDDEN);

    while (RT_EOK != onenet_mqtt_init())
    {
        LOG_E("OneNET initialize failed.");
        rt_thread_delay(ONENRT_INIT_WAIT_TIME);
    }

    /* blocking the thread,and the other tasks can run */
    rt_completion_wait(&onenet_cpt, RT_WAITING_FOREVER);

    onenet_set_cmd_rsp_cb(onenet_cmd_rsp_cb);

    onenet_upload_cycle();

    key_example();

    onenet_sync_flag = RT_TRUE;
}

int rt_hw_wlan_init(void)
{
    rt_thread_t tid = RT_NULL;

    tid = rt_thread_create("wifi_init", wifi_init_thread_entry, RT_NULL, WIFI_INIT_THREAD_STACK_SIZE,
                           WIFI_INIT_THREAD_PRIORITY, 20);
    if (tid)
    {
        rt_thread_startup(tid);
    }
    else
    {
        LOG_E("Create wifi initialization thread fail!");
        return -RT_ERROR;
    }

    return RT_EOK;
}

int onenet_mqtt_start(void)
{
    wifi.wifi_ssid = rt_strdup(IFX_RW007_WIFI_SSID);
    wifi.wifi_password = rt_strdup(IFX_RW007_WIFI_PASSWORD);

    rt_hw_wlan_init();

    return RT_EOK;
}
INIT_APP_EXPORT(onenet_mqtt_start);

static int mqtt_demo_start(int argc, char *argv[])
{
    wifi.wifi_ssid = rt_strdup(IFX_RW007_WIFI_SSID);
    wifi.wifi_password = rt_strdup(IFX_RW007_WIFI_PASSWORD);

    rt_hw_wlan_init();

    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(mqtt_demo_start, onenet_mqtt_demo_start, start one - net mqtt demo);
#endif /* FINSH_USING_MSH */
