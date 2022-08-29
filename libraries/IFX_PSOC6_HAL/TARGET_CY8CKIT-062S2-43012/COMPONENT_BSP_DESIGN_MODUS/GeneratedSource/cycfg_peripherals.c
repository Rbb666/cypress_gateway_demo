/*******************************************************************************
* File Name: cycfg_peripherals.c
*
* Description:
* Peripheral Hardware Block configuration
* This file was automatically generated and should not be modified.
* Tools Package 2.4.0.5972
* mtb-pdl-cat1 2.4.0.13881
* personalities 6.0.0.0
* udd 3.0.0.1974
*
********************************************************************************
* Copyright 2022 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/
#include <rtthread.h>

#include "cycfg_peripherals.h"

#define CYBSP_USBDEV_INTR_LVL_SEL (CY_USBFS_DEV_DRV_SET_SOF_LVL(0x1U) | \
                 CY_USBFS_DEV_DRV_SET_BUS_RESET_LVL(0x2U) | \
                 CY_USBFS_DEV_DRV_SET_EP0_LVL(0x2U) | \
                 CY_USBFS_DEV_DRV_SET_LPM_LVL(0x0U) | \
                 CY_USBFS_DEV_DRV_SET_ARB_EP_LVL(0x0U) | \
                 CY_USBFS_DEV_DRV_SET_EP1_LVL(0x1U) | \
                 CY_USBFS_DEV_DRV_SET_EP2_LVL(0x1U) | \
                 CY_USBFS_DEV_DRV_SET_EP3_LVL(0x1U) | \
                 CY_USBFS_DEV_DRV_SET_EP4_LVL(0x1U) | \
                 CY_USBFS_DEV_DRV_SET_EP5_LVL(0x1U) | \
                 CY_USBFS_DEV_DRV_SET_EP6_LVL(0x1U) | \
                 CY_USBFS_DEV_DRV_SET_EP7_LVL(0x1U) | \
                 CY_USBFS_DEV_DRV_SET_EP8_LVL(0x1U))

cy_stc_csd_context_t cy_csd_0_context =
{
    .lockKey = CY_CSD_NONE_KEY,
};

const cy_stc_usbfs_dev_drv_config_t CYBSP_USBDEV_config =
{
    .mode = CY_USBFS_DEV_DRV_EP_MANAGEMENT_CPU,
    .epAccess = CY_USBFS_DEV_DRV_USE_8_BITS_DR,
    .epBuffer = NULL,
    .epBufferSize = 0U,
    .dmaConfig[0] = NULL,
    .dmaConfig[1] = NULL,
    .dmaConfig[2] = NULL,
    .dmaConfig[3] = NULL,
    .dmaConfig[4] = NULL,
    .dmaConfig[5] = NULL,
    .dmaConfig[6] = NULL,
    .dmaConfig[7] = NULL,
    .enableLpm = false,
    .intrLevelSel = CYBSP_USBDEV_INTR_LVL_SEL,
};
#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_USBDEV_obj =
{
    .type = CYHAL_RSC_USB,
    .block_num = 0U,
    .channel_num = 0U,
};
#endif //defined (CY_USING_HAL)

void init_cycfg_peripherals(void)
{
    Cy_SysClk_PeriphAssignDivider(PCLK_CSD_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);

#ifdef BSP_USING_UART0
    /* UART0 Device Clock*/
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB0_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
#endif
#ifdef BSP_USING_UART1
    /* UART1 Device Clock*/
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB1_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
#endif
#ifdef BSP_USING_UART2
    /* UART2 Device Clock*/
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB2_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
#endif
#if defined(BSP_USING_UART3) || defined(BSP_USING_HW_I2C3)
    /* UART3 Device Clock*/
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB3_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
#endif
#ifdef BSP_USING_UART4
    /* UART4 Device Clock*/
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB4_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
#endif
#ifdef BSP_USING_UART5
    /* UART5 Device Clock*/
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB5_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
#endif
#ifdef BSP_USING_USBD
    /* USB Device Clock*/
    Cy_SysClk_PeriphAssignDivider(PCLK_USB_CLOCK_DEV_BRS, CY_SYSCLK_DIV_16_BIT, 0U);
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&CYBSP_USBDEV_obj);
#endif //defined (CY_USING_HAL)
#endif
}
