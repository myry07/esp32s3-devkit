#include "bsp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"

#include "dirent.h"
#include "sys/stat.h"
#include <inttypes.h>

#define TAG "bsp"

static bsp_st7789_t lcd_dev;
static sdmmc_card_t *card;

// SPI 传输封装
static void spi_send(const uint8_t *data, size_t len, bool is_cmd)
{
    gpio_set_level(BSP_LCD_PIN_DC, is_cmd ? 0 : 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    spi_device_polling_transmit(lcd_dev.spi, &t);
}

static inline void write_cmd(uint8_t cmd)
{
    spi_send(&cmd, 1, true);
}

static inline void write_data(uint8_t data)
{
    spi_send(&data, 1, false);
}

static void delay_ms(int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}


void bsp_st7789_set_rotation(bsp_st7789_t *lcd, uint8_t rot)
{
    rot &= 3; // 0..3
    uint8_t madctl = 0x00;
    uint16_t w = BSP_LCD_WIDTH;
    uint16_t h = BSP_LCD_HEIGHT;
    uint16_t offx = 0, offy = 0;

    // 如果你的模组是 240x240（IC是240x320），用下面这组偏移；
    // 如果是 240x320，offx/offy 都设为 0 即可。
    switch (rot) {
    case 0:
        madctl = 0x00;        // MY=0, MX=0, MV=0
        offx = 0; offy = 20;  // 竖屏时把 80 像素留给 Y
        lcd->width = w;  lcd->height = h;
        break;
    case 1:
        madctl = 0x60;        // MX=1, MV=1
        offx = 20; offy = 0;  // 90° 时偏移到 X
        lcd->width = h;  lcd->height = w; // 交换逻辑宽高
        break;
    case 2:
        madctl = 0xC0;        // MY=1, MX=1
        offx = 0; offy = 20;
        lcd->width = w;  lcd->height = h;
        break;
    case 3:
        madctl = 0xA0;        // MY=1, MV=1
        offx = 20; offy = 0;
        lcd->width = h;  lcd->height = w;
        break;
    }

    // 写 MADCTL
    write_cmd(0x36);
    write_data(madctl);

    // 更新偏移（供 set_window 使用）
    lcd->offset_x = offx;
    lcd->offset_y = offy;
}


esp_err_t bsp_st7789_init(bsp_st7789_t *lcd)
{
    ESP_LOGI(TAG, "ST7789 init start...");

    // 用 local 来保存句柄，避免使用未初始化的全局变量
    bsp_st7789_t local = {
        .width = BSP_LCD_WIDTH,
        .height = BSP_LCD_HEIGHT,
        .offset_x = 0,
        .offset_y = 20,
        .spi = NULL,
    };

    // DC/RST/BL 引脚初始化
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BSP_LCD_PIN_DC) |
                        (1ULL << BSP_LCD_PIN_RST) |
                        (1ULL << BSP_LCD_PIN_BL),
    };
    gpio_config(&io_conf);

    // 硬件复位
    gpio_set_level(BSP_LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BSP_LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 背光 PWM 初始化
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 5000,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 1023,
        .gpio_num = BSP_LCD_PIN_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
    };
    ledc_channel_config(&ledc_channel);

    // SPI 初始化
    spi_bus_config_t buscfg = {
        .mosi_io_num = BSP_LCD_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = BSP_LCD_PIN_SCLK,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(BSP_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_40M,
        .mode = 0,
        .spics_io_num = BSP_LCD_PIN_CS,
        .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(BSP_LCD_HOST, &devcfg, &local.spi));
    lcd_dev = local;
    lcd_dev.spi = local.spi;
    

    // ===== 初始化命令 =====
    write_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(120));
    write_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    write_cmd(0x3A);
    write_data(0x55); // 16-bit color

    write_cmd(0x36);
    write_data(0x00); // RGB 顺序

    // 设置默认窗口
    uint16_t x_start = local.offset_x;
    uint16_t y_start = local.offset_y;
    uint16_t x_end = x_start + local.width - 1;
    uint16_t y_end = y_start + local.height - 1;

    // Column
    write_cmd(0x2A);
    write_data(x_start >> 8);
    write_data(x_start & 0xFF);
    write_data(x_end >> 8);
    write_data(x_end & 0xFF);

    // Row
    write_cmd(0x2B);
    write_data(y_start >> 8);
    write_data(y_start & 0xFF);
    write_data(y_end >> 8);
    write_data(y_end & 0xFF);

    write_cmd(0x21);
    vTaskDelay(pdMS_TO_TICKS(10));
    write_cmd(0x13);
    vTaskDelay(pdMS_TO_TICKS(10));
    write_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 把 local 的值复制到全局和传入指针
    lcd_dev = local;
    *lcd = local;

    ESP_LOGI(TAG, "ST7789 init done. offset_y = %d", local.offset_y);
    return ESP_OK;
}

void bsp_st7789_backlight_set(uint8_t percent)
{
    if (percent > 100)
        percent = 100;
    uint32_t duty = percent * 1023 / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void bsp_st7789_draw_pixel(bsp_st7789_t *lcd, uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= lcd->width || y >= lcd->height)
        return;

    x += lcd->offset_x;
    y += lcd->offset_y;

    uint8_t data[4];

    // 列地址
    write_cmd(0x2A);
    data[0] = x >> 8;
    data[1] = x & 0xFF;
    data[2] = x >> 8;
    data[3] = x & 0xFF;
    spi_send(data, 4, false);

    // 行地址
    write_cmd(0x2B);
    data[0] = y >> 8;
    data[1] = y & 0xFF;
    data[2] = y >> 8;
    data[3] = y & 0xFF;
    spi_send(data, 4, false);

    // 写入像素
    write_cmd(0x2C);
    uint8_t pixel[2] = {color >> 8, color & 0xFF};
    spi_send(pixel, 2, false);
}

void bsp_st7789_fill_screen(bsp_st7789_t *lcd, uint16_t color)
{
    uint8_t data[4];
    uint32_t total = lcd->width * lcd->height;

    uint16_t x_start = lcd->offset_x;
    uint16_t y_start = lcd->offset_y;
    uint16_t x_end = x_start + lcd->width - 1;
    uint16_t y_end = y_start + lcd->height - 1;

    write_cmd(0x2A);
    data[0] = x_start >> 8;
    data[1] = x_start & 0xFF;
    data[2] = x_end >> 8;
    data[3] = x_end & 0xFF;
    spi_send(data, 4, false);

    write_cmd(0x2B);
    data[0] = y_start >> 8;
    data[1] = y_start & 0xFF;
    data[2] = y_end >> 8;
    data[3] = y_end & 0xFF;
    spi_send(data, 4, false);

    write_cmd(0x2C);

    uint8_t pixel[512];
    for (int i = 0; i < 256; i++)
    {
        pixel[i * 2] = color >> 8;
        pixel[i * 2 + 1] = color & 0xFF;
    }

    while (total > 0)
    {
        uint32_t batch = (total > 256) ? 256 : total;
        spi_send(pixel, batch * 2, false);
        total -= batch;
    }
}

static inline void st_send(spi_device_handle_t spi, const uint8_t *data, size_t len, bool is_cmd)
{
    if (!spi)
    {
        ESP_EARLY_LOGE(TAG, "spi NULL");
        return;
    }
    gpio_set_level(BSP_LCD_PIN_DC, is_cmd ? 0 : 1);
    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = data;
    esp_err_t r = spi_device_polling_transmit(spi, &t);
    if (r != ESP_OK)
        ESP_EARLY_LOGE(TAG, "tx err %d", r);
}

void bsp_st7789_send_cmd(bsp_st7789_t *lcd, uint8_t cmd)
{
    if (!lcd || !lcd->spi)
    {
        ESP_LOGE(TAG, "cmd: invalid lcd");
        return;
    }
    st_send(lcd->spi, &cmd, 1, true);
}

void bsp_st7789_send_data(bsp_st7789_t *lcd, const uint8_t *data, size_t len)
{
    if (!lcd || !lcd->spi)
    {
        ESP_LOGE(TAG, "data: invalid lcd");
        return;
    }
    st_send(lcd->spi, data, len, false);
}

void bsp_st7789_set_window(bsp_st7789_t *lcd, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (!lcd || !lcd->spi)
    {
        ESP_LOGE(TAG, "win: invalid lcd");
        return;
    }

    uint8_t buf[4];

    bsp_st7789_send_cmd(lcd, 0x2A);
    buf[0] = (x1 >> 8) & 0xFF;
    buf[1] = x1 & 0xFF;
    buf[2] = (x2 >> 8) & 0xFF;
    buf[3] = x2 & 0xFF;
    bsp_st7789_send_data(lcd, buf, 4);

    bsp_st7789_send_cmd(lcd, 0x2B);
    buf[0] = (y1 >> 8) & 0xFF;
    buf[1] = y1 & 0xFF;
    buf[2] = (y2 >> 8) & 0xFF;
    buf[3] = y2 & 0xFF;
    bsp_st7789_send_data(lcd, buf, 4);

    bsp_st7789_send_cmd(lcd, 0x2C);
}

void bsp_st7789_draw_bitmap_le(bsp_st7789_t *lcd, uint16_t x, uint16_t y,
                               uint16_t w, uint16_t h, const uint16_t *color_data_le)
{
    if (x >= lcd->width || y >= lcd->height)
        return;
    if (x + w > lcd->width)
        w = lcd->width - x;
    if (y + h > lcd->height)
        h = lcd->height - y;

    uint16_t x1 = x + lcd->offset_x;
    uint16_t y1 = y + lcd->offset_y;
    uint16_t x2 = x1 + w - 1;
    uint16_t y2 = y1 + h - 1;

    // 设置窗口
    bsp_st7789_set_window(lcd, x1, y1, x2, y2);

    // 发送数据
    uint32_t pixels = w * h;
    const uint32_t batch = 1024;
    static uint8_t *buf = NULL;
    if (!buf)
        buf = heap_caps_malloc(batch * 2, MALLOC_CAP_DMA);

    uint32_t sent = 0;
    while (sent < pixels)
    {
        uint32_t n = (pixels - sent > batch) ? batch : pixels - sent;
        for (uint32_t i = 0; i < n; i++)
        {
            uint16_t p = color_data_le[sent + i];
            buf[2 * i] = p >> 8;
            buf[2 * i + 1] = p & 0xFF;
        }
        bsp_st7789_send_data(lcd, buf, n * 2);
        sent += n;
    }
}

esp_err_t bsp_init_sdmmc_card(void)
{
    esp_err_t ret;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_1;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; // 可用高速模式

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    slot_config.clk = BSP_SDMMC_CLK;
    slot_config.cmd = BSP_SDMMC_CMD;
    slot_config.d0 = BSP_SDMMC_D0;
    slot_config.d1 = BSP_SDMMC_D1;
    slot_config.d2 = BSP_SDMMC_D2;
    slot_config.d3 = BSP_SDMMC_D3;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false, // 不自动格式化
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    ret = esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SD card mounted successfully");

    return ESP_OK;
}

esp_err_t bsp_sdcard_print_info(const char *mount_path)
{
    if (!card)
    {
        ESP_LOGE(TAG, "SD card not initialized");
        return ESP_FAIL;
    }

    // 打印卡信息
    ESP_LOGI(TAG, "==== SD Card Info ====");
    ESP_LOGI(TAG, "Name: %s", card->cid.name);
    ESP_LOGI(TAG, "Capacity: %llu MB",
             ((uint64_t)card->csd.capacity * card->csd.sector_size) / (1024 * 1024));
    ESP_LOGI(TAG, "Speed: %" PRIu32 " kHz", card->max_freq_khz);
    ESP_LOGI(TAG, "Type: %s", (card->ocr & (1 << 30)) ? "SDHC/SDXC" : "SDSC");

    // 打印文件列表
    ESP_LOGI(TAG, "==== Listing files under: %s ====", mount_path);

    DIR *dir = opendir(mount_path);
    if (!dir)
    {
        ESP_LOGE(TAG, "Failed to open directory: %s", mount_path);
        return ESP_FAIL;
    }

    struct dirent *entry;
    struct stat st;
    char path[256];

    while ((entry = readdir(dir)) != NULL)
    {
        if (snprintf(path, sizeof(path), "%s/%s", mount_path, entry->d_name) >= sizeof(path))
        {
            ESP_LOGW(TAG, "Path too long, skipped: %s/%s", mount_path, entry->d_name);
            continue;
        }
        if (stat(path, &st) == 0)
        {
            if (S_ISDIR(st.st_mode))
            {
                ESP_LOGI(TAG, "[DIR ] %s", entry->d_name);
            }
            else
            {
                ESP_LOGI(TAG, "[FILE] %s (%ld bytes)", entry->d_name, st.st_size);
            }
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "==== End of list ====");

    return ESP_OK;
}

esp_err_t bsp_sdcard_deinit(void)
{
    if (card == NULL)
    {
        ESP_LOGW(TAG, "No SD card mounted, skip deinit.");
        return ESP_FAIL;
    }

    esp_err_t ret = esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT, card);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "SD card unmounted from %s", BSP_SD_MOUNT);
        card = NULL;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
    }
    return ret;
}