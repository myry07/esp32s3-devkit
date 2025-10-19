#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <stdint.h>
#include <stdbool.h>

#define BSP_LCD_PIN_MOSI 4
#define BSP_LCD_PIN_SCLK 5
#define BSP_LCD_PIN_CS 6
#define BSP_LCD_PIN_DC 7
#define BSP_LCD_PIN_RST 15
#define BSP_LCD_PIN_BL 16

#define BSP_SDMMC_D0 40
#define BSP_SDMMC_D1 -1
#define BSP_SDMMC_D2 -1
#define BSP_SDMMC_D3 -1
#define BSP_SDMMC_CMD 38
#define BSP_SDMMC_CLK 39

#define BSP_BOOT      0

#define BSP_LCD_HOST SPI2_HOST
#define BSP_LCD_WIDTH 240
#define BSP_LCD_HEIGHT 280

#define BSP_SD_MOUNT "/sdcard"

// 常见颜色宏
#define BSP_COLOR_BLACK 0x0000
#define BSP_COLOR_WHITE 0xFFFF
#define BSP_COLOR_RED 0xF800
#define BSP_COLOR_GREEN 0x07E0
#define BSP_COLOR_BLUE 0x001F
#define BSP_COLOR_YELLOW 0xFFE0

typedef struct
{
    uint16_t width;
    uint16_t height;
    uint16_t offset_x;
    uint16_t offset_y;
    spi_device_handle_t spi;
} bsp_st7789_t;

// 屏幕控制
esp_err_t bsp_st7789_init(bsp_st7789_t *lcd);
void bsp_st7789_fill_screen(bsp_st7789_t *lcd, uint16_t color);
void bsp_st7789_draw_pixel(bsp_st7789_t *lcd, uint16_t x, uint16_t y, uint16_t color);
void bsp_st7789_backlight_set(uint8_t percent);
void bsp_st7789_send_cmd(bsp_st7789_t *lcd, uint8_t cmd);
void bsp_st7789_send_data(bsp_st7789_t *lcd, const uint8_t *data, size_t len);
void bsp_st7789_set_window(bsp_st7789_t *lcd, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void bsp_st7789_set_rotation(bsp_st7789_t *lcd, uint8_t rot);

// SD Card
esp_err_t bsp_init_sdmmc_card(void);
esp_err_t bsp_sdcard_print_info(const char *mount_path);
esp_err_t bsp_sdcard_deinit(void);
