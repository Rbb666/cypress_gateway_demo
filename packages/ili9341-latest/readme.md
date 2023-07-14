#  ILI9341

## 简介

本软件包是 TFT-LCD-ILI9341 SPI接口屏幕的驱动包，本软件包已经对接到 SPI 框架。通过 SPI 框架 API，开发者可以快速的将此屏幕驱动起来。

##  使用说明

### 依赖

- RT-Thread 5.0.0+
- SPI 驱动：ILI9341 屏幕使用 SPI 进行数据通讯，需要系统SPI 驱动支持；

###  获取软件包

使用 ILI9341 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```shell
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    TFT-LCD ILI9341 SPI screen driver software package --->
        (spi0) spi bus name
        (spi00) spi device name
        (240) Width of the LCD display
        (320) Height of the LCD display
        (-1) DC pin connected to the LCD display
	(-1) RESET pin connected to the LCD display
	(-1) CS pin connected to the LCD display
	(-1) Backlight pin connected to the LCD display
		Version (latest)  --->
```

**spi bus name**：连接 LCD 所使用的 SPI 总线名称

**spi device name**：连接 LCD 所使用的 SPI 设备名称

**Width of the LCD display**： LCD  屏幕宽度参数

**Height of the LCD display**： LCD 屏幕高度参数

**DC pin connected to the LCD display**： LCD 屏幕数据引脚

**RESET pin connected to the LCD display**： LCD 屏幕复位引脚

**CS pin connected to the LCD display**： LCD 屏幕片选引脚（若硬件无CS引脚则填写 -1）

**Backlight pin connected to the LCD display**： LCD 屏幕背光引脚

###  使用软件包

ILI9341 软件包初始化函数如下所示：

```c
rt_err_t spi_lcd_init(uint32_t freq)
```

该函数需要由用户调用，函数主要完成的功能有：

- 设备配置和初始化（根据传入的配置信息，配置 SPI 的频率，初始化 LCD 参数）；
- 注册相应的传感器设备，完成 SPI 设备的注册；

> 参数：SPI 的频率。例如：单片机的 SPI 外设频率为 20 MHZ，则填写 20

```c
void lcd_fill_array_spi(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, void *Image)
```

该函数需要由用户调用，函数主要完成的功能有：

- 填充颜色数组到 LCD 显存

> 参数：要填充的x起始坐标，y起始坐标，x终止坐标，y终止坐标，颜色数组

####  初始化示例

```c
#include "lcd_ili9341.h"

int rt_hw_ili9341_port(void)
{
    /* LCD init by 20MHZ SPI */
    spi_lcd_init(20);
    /* LCD fill blue color */
    LCD_Clear(0x001F);
    rt_thread_mdelay(1000);
    /* LCD fill red color */
    LCD_Clear(0xF800);
    return 0;
}
INIT_APP_EXPORT(rt_hw_ili9341_port);
```

#### LVGL V8.2.0 接口示例

```c
#include "lcd_ili9341.h"

static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    lcd_fill_array_spi(area->x1, area->y1, area->x2, area->y2, color_p);
    lv_disp_flush_ready(disp_drv);
}

void lv_port_disp_init(void)
{
    spi_lcd_init(20);
    ...
    /*Set the resolution of the display*/
    disp_drv.hor_res = PKG_ILI_9341_WIDTH;
    disp_drv.ver_res = PKG_ILI_9341_HEIGHT;
    ...
}
```

## 注意事项

暂无

## 联系人信息

维护人:

- [Rbb66](https://github.com/Rbb666)
- 主页：[TFT-LCD-ILI9341](https://github.com/Rbb666/ILI9341)
