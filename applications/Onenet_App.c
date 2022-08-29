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
#define PWM_DEV_CHANNEL 3
#define GET_DUTY_CYCLE(x)       (1 * 1000 * 1000 - x * 10 * 1000)

struct wifi_info
{
    char *wifi_ssid;
    char *wifi_password;
};
static struct wifi_info wifi;

static struct rt_device_pwm *pwm_dev;

bool onenet_sync_flag = false;

/* onenet mqtt command response callback and analysis function */
static void onenet_cmd_rsp_cb(uint8_t *recv_data, size_t recv_size, uint8_t **resp_data, size_t *resp_size)
{
    LOG_D("Recv data is %.*s\n", recv_size, recv_data);

    cJSON *cjson_root = RT_NULL, *cjson_ctl = RT_NULL;
    cjson_root = cJSON_Parse((const char *) recv_data);

    cjson_ctl = cJSON_GetObjectItem(cjson_root, "Led");
    if (cjson_ctl->type == cJSON_Number)
    {
        rt_bool_t led_rev_data = RT_FALSE;

        led_rev_data = cjson_ctl->valueint;

        LOG_D("led state:%d\n", led_rev_data);

        led_rev_data == RT_TRUE ?
                rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 1 * 1000 * 1000, GET_DUTY_CYCLE(100)) :
                rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 1 * 1000 * 1000, GET_DUTY_CYCLE(0));

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

    while (1)
    {
        humi_val = round(1.0 * rand() / RAND_MAX * 20 + 40);
        tempature_val = round(1.0 * rand() / RAND_MAX * 10 + 20);

        if (onenet_mqtt_upload_digit("temperature", tempature_val) < 0)
        {
            LOG_E("upload has an error, stop uploading");
            goto _exit;
        }

        if (onenet_mqtt_upload_digit("humidity", humi_val) < 0)
        {
            LOG_E("upload has an error, stop uploading");
            goto _exit;
        }

        rt_thread_delay(rt_tick_from_millisecond(5 * 1000));

_exit:
        rt_thread_delay(rt_tick_from_millisecond(1 * 1000));
        continue;
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

static void wifi_init_thread_entry(void *parameter)
{
    log_i("Try to connect SSID[%s] Password[%s]", wifi.wifi_ssid, wifi.wifi_password);

    rt_wlan_connect(wifi.wifi_ssid, wifi.wifi_password);

    while (!rt_wlan_is_connected())
    {
        rt_kprintf("@reconnect ap...\n");
        rt_wlan_connect(wifi.wifi_ssid, wifi.wifi_password);
        rt_thread_delay(WIFI_INIT_WAIT_TIME);
    }

    rt_thread_delay(ONENRT_INIT_WAIT_TIME);

    while (RT_EOK != onenet_mqtt_init())
    {
        LOG_E("OneNET initialize failed.");
        rt_thread_delay(ONENRT_INIT_WAIT_TIME);
    }
    onenet_set_cmd_rsp_cb(onenet_cmd_rsp_cb);

    rt_thread_delay(rt_tick_from_millisecond(5 * 1000));

    onenet_upload_cycle();

    onenet_sync_flag = true;
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

int mqtt_demo_start(int argc, char *argv[])
{
    if (argc != 3)
    {
        LOG_E("wifi [ssid]  [password]  - input wifi ssid and password to start demo.");
        return -RT_ERROR;
    }

    wifi.wifi_ssid = rt_malloc(sizeof(argv[1]));
    wifi.wifi_password = rt_malloc(sizeof(argv[2]));

    rt_memset(wifi.wifi_ssid, 0x00, sizeof(argv[1]));
    rt_memset(wifi.wifi_password, 0x00, sizeof(argv[2]));

    strcpy(wifi.wifi_ssid, argv[1]);
    strcpy(wifi.wifi_password, argv[2]);

    LOG_HEX("wifi_ssid", 16, wifi.wifi_ssid, sizeof(wifi.wifi_ssid));
    LOG_HEX("wifi_password", 16, wifi.wifi_password, sizeof(wifi.wifi_password));

    rt_hw_wlan_init();

    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(mqtt_demo_start, onenet_mqtt_demo_start, start one-net mqtt demo);
#endif /* FINSH_USING_MSH */
