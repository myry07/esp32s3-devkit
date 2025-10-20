#include <stdio.h>
#include "esp_log.h"
#include "bsp.h"
#include "app.h"

#define TAG "main"

static bsp_st7789_t lcd;

void app_main(void)
{
    ESP_LOGI(TAG, "Init LCD + SD...");
    bsp_st7789_init(&lcd);

    bsp_i2s_init();

    bsp_init_sdmmc_card();
    bsp_sdcard_print_info("/sdcard/test");

    // 构建播放列表
    // int n = build_playlist("/sdcard/test");
    // ESP_LOGI(TAG, "playlist count=%d", n);

    // 初始化 BOOT 按键（下一段）
    init_boot_button();

    // 初始化解码/推屏流水线（旋转90°, 背光80%, 右移20px）
    // decode_pipeline_init(&lcd, /*rotation=*/1, /*backlight*/80, /*right_nudge_px*/20);

    // 开始播放
    // decode_play_dir("/sdcard/test");

    bsp_i2s_play_wav("/sdcard/test/test.wav");





}