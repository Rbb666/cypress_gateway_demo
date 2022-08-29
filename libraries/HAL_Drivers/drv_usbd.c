/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-04-10     ZYH          first version
 */

#include <rtthread.h>

#ifdef BSP_USING_USBD
#include <rtdevice.h>
#include "board.h"
#include <string.h>

#include "cyhal_clock.h"

static struct udcd _ifx_udc;

#define USB_SYS_CLOCK_HZ  48000000u

/* USBDEV context variables */
cy_stc_usbfs_dev_drv_context_t  usb_drvContext;
cy_stc_usb_dev_context_t        usb_devContext;

cyhal_usb_dev_t usb_dev_obj;
cyhal_clock_t pll_clock;
cyhal_clock_t usb_clock;

static struct ep_id _ehci0_ep_pool[] =
{
    {0x0,  USB_EP_ATTR_CONTROL,     USB_DIR_INOUT,  64, ID_ASSIGNED  },
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_BULK,        USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x4,  USB_EP_ATTR_INT,         USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x4,  USB_EP_ATTR_INT,         USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x5,  USB_EP_ATTR_BULK,        USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x5,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x6,  USB_EP_ATTR_INT,         USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x6,  USB_EP_ATTR_INT,         USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x7,  USB_EP_ATTR_BULK,        USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x7,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0xFF, USB_EP_ATTR_TYPE_MASK,   USB_DIR_MASK,   0,  ID_ASSIGNED  },
};

/* USB Interrupt Configuration */
const cy_stc_sysint_t usb_high_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_hi_IRQn,
    .intrPriority = 5U,
};
const cy_stc_sysint_t usb_medium_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_med_IRQn,
    .intrPriority = 6U,
};
const cy_stc_sysint_t usb_low_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_lo_IRQn,
    .intrPriority = 7U,
};

static void usb_high_isr(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USBDEV_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseHi(CYBSP_USBDEV_HW),
                               &usb_drvContext);
    /* leave interrupt */
    rt_interrupt_leave();
}

static void usb_medium_isr(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USBDEV_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseMed(CYBSP_USBDEV_HW),
                               &usb_drvContext);
    /* leave interrupt */
    rt_interrupt_leave();
}

static void usb_low_isr(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USBDEV_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseLo(CYBSP_USBDEV_HW),
                               &usb_drvContext);

    /* leave interrupt */
    rt_interrupt_leave();
}

cy_en_usb_dev_status_t usb_dev_bus_event_callback(
    cy_en_usb_dev_callback_events_t event,
    uint32_t wValue,
    uint32_t wIndex,
    struct cy_stc_usb_dev_context *devContext)
{

    // Handle USB Events here
    if (event == CY_USB_DEV_EVENT_BUS_RESET)
    {

    }
    // SET_CONFIGURATION request received
    else if (event == CY_USB_DEV_EVENT_SET_CONFIG)
    {
//        cy_stc_usb_dev_control_transfer_t *transfer = &devContext->ControlTransfer;
//
//        rt_usbd_ep0_setup_handler(&_ifx_udc, (struct urequest*)transfer->buffer);
    }
    // SET_INTERFACE request received
    else if (event == CY_USB_DEV_EVENT_SET_INTERFACE)
    {
        cy_stc_usb_dev_control_transfer_t *transfer = &devContext->ControlTransfer;

//        rt_usbd_ep_in_handler(&_ifx_udc, ep_addr, message->length);
    }

    return CY_USB_DEV_SUCCESS;
}

void usb_irq_handler()
{
    // Calling the default USB handler
    cyhal_usb_dev_process_irq(&usb_dev_obj);
}

static void usb_dev_bus_reset_callback(void)
{

}

static void usb_dev_ep0_setup_callback(void)
{

}

static void usb_dev_ep0_in_callback(void)
{

}

static void usb_dev_ep0_out_callback(void)
{

}

static rt_err_t drv_ehci0_usbd_init(rt_device_t device)
{
    cy_rslt_t result;
//    /* Initialize the USB device */
//    result = Cy_USB_Dev_Init(CYBSP_USBDEV_HW, &CYBSP_USBDEV_config, &usb_drvContext,
//                             &usb_devices[0], &usb_devConfig, &usb_devContext);
//
//    /* Initialize the USB interrupts */
//    Cy_SysInt_Init(&usb_high_interrupt_cfg,   &usb_high_isr);
//    Cy_SysInt_Init(&usb_medium_interrupt_cfg, &usb_medium_isr);
//    Cy_SysInt_Init(&usb_low_interrupt_cfg,    &usb_low_isr);
//
//    /* Enable the USB interrupts */
//    NVIC_EnableIRQ(usb_high_interrupt_cfg.intrSrc);
//    NVIC_EnableIRQ(usb_medium_interrupt_cfg.intrSrc);
//    NVIC_EnableIRQ(usb_low_interrupt_cfg.intrSrc);
//
//    Cy_USB_Dev_RegisterEventsCallback(usb_dev_bus_event_callback, &usb_devContext);
//
//    Cy_USB_Dev_Connect(true, CY_USB_DEV_WAIT_FOREVER, &usb_devContext);

    /* Initialize the PLL */
//    cyhal_clock_get_frequency(&CYHAL_CLOCK_PLL[0]);
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
//    cyhal_clock_init(&pll_clock);
    cyhal_clock_set_frequency(&pll_clock, USB_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (HFCLK1) */
//    cyhal_clock_get_frequency(&CYHAL_CLOCK_HF[3]);
    cyhal_clock_reserve(&usb_clock, &CYHAL_CLOCK_HF[3]);
//    cyhal_clock_init(&usb_clock);
    cyhal_clock_set_source(&usb_clock, &pll_clock);

    /* Drop HFCK1 frequency for power savings */
    cyhal_clock_set_divider(&usb_clock, 4);
    cyhal_clock_set_enabled(&usb_clock, true, true);

    result = cyhal_usb_dev_init(&usb_dev_obj, USBDP, USBDM, NULL);
    RT_ASSERT(result == CY_RSLT_SUCCESS);

    cyhal_usb_dev_irq_enable(&usb_dev_obj, true);
//    cyhal_usb_dev_register_irq_callback(&usb_dev_obj, usb_irq_handler);
//
//    cyhal_usb_dev_register_event_callback(&usb_dev_obj, CYHAL_USB_DEV_EVENT_BUS_RESET, usb_dev_bus_reset_callback);
//    cyhal_usb_dev_register_event_callback(&usb_dev_obj, CYHAL_USB_DEV_EVENT_EP0_SETUP, usb_dev_ep0_setup_callback);
//    cyhal_usb_dev_register_event_callback(&usb_dev_obj, CYHAL_USB_DEV_EVENT_EP0_IN,    usb_dev_ep0_in_callback);
//    cyhal_usb_dev_register_event_callback(&usb_dev_obj, CYHAL_USB_DEV_EVENT_EP0_OUT,   usb_dev_ep0_out_callback);
//
//    (void) cyhal_usb_dev_endpoint_add(&usb_dev_obj, true, false, 1U, 64, CYHAL_USB_DEV_EP_TYPE_BULK);

    cyhal_usb_dev_connect(&usb_dev_obj);

    if (result != CY_USB_DEV_SUCCESS)
    {
        rt_kprintf("USB_DeviceInit ehci0 error\r\n");
        RT_ASSERT(0);
        return RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _set_address(rt_uint8_t address)
{
    Cy_USBFS_Dev_Drv_SetAddress(CYBSP_USBDEV_HW, address, usb_devContext.drvContext);

    return RT_EOK;
}

static rt_err_t _set_config(rt_uint8_t address)
{
    return RT_EOK;
}

static rt_err_t _ep_set_stall(rt_uint8_t address)
{
    Cy_USBFS_Dev_Drv_StallEndpoint(CYBSP_USBDEV_HW, address, usb_devContext.drvContext);

    return RT_EOK;
}

static rt_err_t _ep_clear_stall(rt_uint8_t address)
{
    Cy_USBFS_Dev_Drv_UnStallEndpoint(CYBSP_USBDEV_HW, address, usb_devContext.drvContext);

    return RT_EOK;
}

static rt_err_t _ep_enable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);

    rt_uint32_t param = ep->ep_desc->bEndpointAddress;
    /* Enable endpoint to be written by host */
    Cy_USBFS_Dev_Drv_EnableOutEndpoint(CYBSP_USBDEV_HW, param, &usb_drvContext);
    return RT_EOK;
}

static rt_err_t _ep_disable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
    rt_uint32_t param = ep->ep_desc->bEndpointAddress;
    Cy_USBFS_Dev_Drv_RemoveEndpoint(CYBSP_USBDEV_HW, param, &usb_drvContext);
    return RT_EOK;
}

static rt_size_t _ep_read_prepare(rt_uint8_t address, void *buffer, rt_size_t size)
{
    uint32_t read_count;
    Cy_USBFS_Dev_Drv_Ep0Read(CYBSP_USBDEV_HW, buffer, size, &usb_drvContext);
    return size;
}

static rt_size_t _ep_read(rt_uint8_t address, void *buffer)
{
    rt_size_t size = 0;
    RT_ASSERT(buffer != RT_NULL);
    return size;
}

static rt_size_t _ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    /* Write data back to the host */
    Cy_USBFS_Dev_Drv_Ep0Write(CYBSP_USBDEV_HW, buffer, size, &usb_drvContext);
//    Cy_USB_Dev_WriteEpNonBlocking(address, buffer, size, &usb_devContext);
    return size;
}

static rt_err_t _ep0_send_status(void)
{
    Cy_USBFS_Dev_Drv_Ep0Write(CYBSP_USBDEV_HW, NULL, 0, &usb_drvContext);
//    Cy_USB_Dev_WriteEpNonBlocking(0x00, NULL, 0, &usb_devContext);
    return RT_EOK;
}

static rt_err_t _suspend(void)
{
    return RT_EOK;
}

static rt_err_t _wakeup(void)
{
    return RT_EOK;
}

const static struct udcd_ops _ehci0_udc_ops =
{
    _set_address,
    _set_config,
    _ep_set_stall,
    _ep_clear_stall,
    _ep_enable,
    _ep_disable,
    _ep_read_prepare,
    _ep_read,
    _ep_write,
    _ep0_send_status,
    _suspend,
    _wakeup,
};

static int rt_usbd_init(void)
{
    rt_memset((void *)&_ifx_udc, 0, sizeof(struct udcd));
    _ifx_udc.parent.type = RT_Device_Class_USBDevice;
    _ifx_udc.parent.init = drv_ehci0_usbd_init;
    _ifx_udc.ops = &_ehci0_udc_ops;
    /* Register endpoint infomation */
    _ifx_udc.ep_pool = _ehci0_ep_pool;
    _ifx_udc.ep0.id = &_ehci0_ep_pool[0];

#ifdef BSP_USBD_SPEED_HS
    _ifx_udc.device_is_hs = RT_TRUE;
#endif
    rt_device_register((rt_device_t)&_ifx_udc, "usbd", 0);
    rt_usb_device_init();

    return RT_EOK;
}
//INIT_DEVICE_EXPORT(rt_usbd_init);
#endif
