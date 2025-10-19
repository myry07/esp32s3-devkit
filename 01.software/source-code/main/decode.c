#include <string.h>
#include <stdint.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "bsp.h"
#include "avi_player.h"
#include "esp_jpeg_dec.h"

#include "driver/gpio.h"

#include "esp_timer.h"

#include "app.h"

#define TAG "decode"

/* ================== 可调参数 ================== */
// DMA 小块（内部SRAM），可按 SPI max_transfer_sz 提升到 32k/64k
#ifndef TX_CHUNK_BYTES
#define TX_CHUNK_BYTES 4092
#endif
/* ============================================== */

// 全局状态（仅本模块可见）
static bsp_st7789_t *s_lcd = NULL;
static jpeg_dec_handle_t s_jpeg = NULL;
static QueueHandle_t s_q_ready = NULL;   // 仅“就绪帧”队列
static uint8_t *s_dma_chunk = NULL;      // 内部SRAM DMA发送缓存
static int s_right_nudge = 0;            // 右移像素
static avi_player_handle_t s_avi = NULL; // 播放器句柄

/* ====== 播放列表 ====== */
char **g_playlist = NULL;
int g_count = 0;
int g_index = 0;

/* ====== 按键（BOOT）事件队列 ====== */
static QueueHandle_t g_btn_q = NULL;

static volatile bool g_skip_req = false;
static volatile uint64_t s_last_frame_us = 0;

// 一个已解码帧（RGB565 LE 格式），送显后释放 outbuf
typedef struct
{
    uint8_t *buf; // jpeg_calloc_align() 得到的对齐缓冲（LE）
    uint16_t w, h;
    uint16_t x, y; // 逻辑坐标（居中 + 偏移）
    size_t bytes;  // w*h*2
} frame_t;

static void button_monitor_task(void *arg)
{
    for (;;)
    {
        uint32_t sig;
        if (g_btn_q && xQueueReceive(g_btn_q, &sig, portMAX_DELAY) == pdPASS)
        {
#ifdef AVI_PLAYER_HAS_STOP
            if (s_avi)
            {
                avi_player_stop(s_avi); // 立即打断当前播放
            }
#else
            g_skip_req = true; // 没有 stop，只能标记：当前文件播完立刻切下一段
#endif
        }
    }
}

static bool has_ext_avi(const char *name)
{
    size_t n = strlen(name);
    if (n < 4)
        return false;
    const char *ext = name + n - 4;
    return strcasecmp(ext, ".avi") == 0;
}

// 传入目录，如 "/sdcard/stv"；返回找到的文件数
int build_playlist(const char *dirpath)
{
    DIR *dir = opendir(dirpath);
    if (!dir)
    {
        ESP_LOGE(TAG, "opendir(%s) failed: %d (%s)", dirpath, errno, strerror(errno));
        return 0;
    }

    // 先统计数量
    int cnt = 0;
    struct dirent *ent;
    struct stat st;
    char path[256];

    while ((ent = readdir(dir)) != NULL)
    {
        if (!has_ext_avi(ent->d_name))
            continue;
        // 用 stat 判断是否是普通文件（不要用 d_type）
        if (snprintf(path, sizeof(path), "%s/%s", dirpath, ent->d_name) >= (int)sizeof(path))
        {
            continue;
        }
        if (stat(path, &st) == 0 && S_ISREG(st.st_mode))
        {
            cnt++;
        }
    }

    if (cnt == 0)
    {
        closedir(dir);
        return 0;
    }

    // 分配列表并填充
    g_playlist = (char **)heap_caps_malloc(cnt * sizeof(char *), MALLOC_CAP_DEFAULT);
    if (!g_playlist)
    {
        closedir(dir);
        ESP_LOGE(TAG, "malloc playlist failed");
        return 0;
    }

    // 从头再遍历一次
    rewinddir(dir);
    int i = 0;
    while ((ent = readdir(dir)) != NULL && i < cnt)
    {
        if (!has_ext_avi(ent->d_name))
            continue;
        if (snprintf(path, sizeof(path), "%s/%s", dirpath, ent->d_name) >= (int)sizeof(path))
        {
            continue;
        }
        if (stat(path, &st) == 0 && S_ISREG(st.st_mode))
        {
            size_t len = strlen(path) + 1;
            g_playlist[i] = (char *)heap_caps_malloc(len, MALLOC_CAP_DEFAULT);
            if (!g_playlist[i])
            {
                ESP_LOGE(TAG, "malloc path failed");
                // 简单回收已分配
                for (int k = 0; k < i; k++)
                    free(g_playlist[k]);
                free(g_playlist);
                g_playlist = NULL;
                closedir(dir);
                return 0;
            }
            memcpy(g_playlist[i], path, len);
            i++;
        }
    }
    closedir(dir);

    g_count = i;
    g_index = 0;
    ESP_LOGI(TAG, "playlist built: %d files", g_count);
    for (int j = 0; j < g_count; ++j)
    {
        ESP_LOGI(TAG, "[%02d] %s", j, g_playlist[j]);
    }
    return g_count;
}

static void IRAM_ATTR boot_isr(void *arg)
{
    uint32_t sig = 1;
    BaseType_t hp = pdFALSE;
    if (g_btn_q)
        xQueueSendFromISR(g_btn_q, &sig, &hp);
    if (hp)
        portYIELD_FROM_ISR();
}

void init_boot_button(void) {
    static bool s_isr_installed = false;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BSP_BOOT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io);

    if (!s_isr_installed) {
        gpio_install_isr_service(0);
        s_isr_installed = true;
    }
    gpio_isr_handler_add(BSP_BOOT, boot_isr, NULL);

    if (!g_btn_q) g_btn_q = xQueueCreate(4, sizeof(uint32_t));
}

/* 小工具：把 LE 拷贝并转成 BE 到 DMA 块（ST7789 多数是BE） */
static inline void copy_swap_chunk(uint8_t *dst_dma, const uint8_t *src_psram, size_t bytes)
{
    size_t n = bytes & ~((size_t)1); // 保证偶数
    for (size_t i = 0; i < n; i += 2)
    {
        dst_dma[i] = src_psram[i + 1];
        dst_dma[i + 1] = src_psram[i];
    }
}

/* 打开 JPEG 解码器（一次） */
static esp_err_t jpeg_open_once(void)
{
    if (s_jpeg)
        return ESP_OK;
    jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
    cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE; // 解码为 LE（最快）
    jpeg_error_t je = jpeg_dec_open(&cfg, &s_jpeg);
    if (je != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG, "jpeg_dec_open failed: %d", je);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* 推屏任务（Core0） */
static void blit_task(void *arg)
{
    ESP_LOGI(TAG, "blit_task start (Core=%d)", xPortGetCoreID());
    for (;;)
    {
        frame_t fb;
        if (xQueueReceive(s_q_ready, &fb, portMAX_DELAY) != pdPASS)
            continue;

        // 一帧只设一次窗口（bsp 内部会处理面板 offset 与 0x2C）
        bsp_st7789_set_window(s_lcd, fb.x, fb.y, fb.x + fb.w - 1, fb.y + fb.h - 1);

        // PSRAM(LE) -> 内部 DMA 块(LE->BE) -> SPI 发送
        size_t left = fb.bytes;
        const uint8_t *p = fb.buf;
        while (left)
        {
            size_t n = (left > TX_CHUNK_BYTES) ? TX_CHUNK_BYTES : left;
            if (n & 1)
                n--; // 偶数
            copy_swap_chunk(s_dma_chunk, p, n);
            bsp_st7789_send_data(s_lcd, s_dma_chunk, n); // 只发数据
            p += n;
            left -= n;
        }

        // 释放这帧的输出缓冲
        if (fb.buf)
            jpeg_free_align(fb.buf);
    }
}

/* AVI 回调：解码（Core1） */
static void video_cb(frame_data_t *data, void *arg)
{
    if (!data || !data->data || data->data_bytes == 0)
        return;
    if (jpeg_open_once() != ESP_OK)
        return;

    jpeg_dec_io_t io = {
        .inbuf = (uint8_t *)data->data,
        .inbuf_len = (int)data->data_bytes,
        .outbuf = NULL,
    };

    jpeg_dec_header_info_t hi;
    if (jpeg_dec_parse_header(s_jpeg, &io, &hi) != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG, "jpeg header parse failed");
        return;
    }

    // 关键：问库要这帧真正需要的 out_len（不等于 w*h*2）
    int out_len = 0;
    if (jpeg_dec_get_outbuf_len(s_jpeg, &out_len) != JPEG_ERR_OK || out_len <= 0)
    {
        ESP_LOGE(TAG, "jpeg_dec_get_outbuf_len failed");
        return;
    }

    // 每帧按 out_len 申请，并确保 16B 对齐（库函数保证）
    uint8_t *outbuf = (uint8_t *)jpeg_calloc_align(out_len, 16);
    if (!outbuf)
    {
        ESP_LOGE(TAG, "jpeg_calloc_align %d bytes failed", out_len);
        return;
    }
    io.outbuf = outbuf;


    if (jpeg_dec_process(s_jpeg, &io) != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG, "jpeg decode failed");
        jpeg_free_align(outbuf);
        return;
    }

    // ---- 新增：帧心跳（用于判断播放是否结束）----
    s_last_frame_us = esp_timer_get_time();

    const uint16_t img_w = hi.width;
    const uint16_t img_h = hi.height;
    const size_t bytes = (size_t)img_w * img_h * 2; // 实际像素字节数

    // 居中 + 右偏移
    const uint16_t lcd_w = s_lcd->width; // 已随旋转更新
    const uint16_t lcd_h = s_lcd->height;

    int x = (lcd_w > img_w) ? ((int)lcd_w - (int)img_w) / 2 : 0;
    int y = (lcd_h > img_h) ? ((int)lcd_h - (int)img_h) / 2 : 0;
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    int max_x = (int)lcd_w - (int)img_w;
    int max_y = (int)lcd_h - (int)img_h;
    if (x > max_x)
        x = max_x;
    if (y > max_y)
        y = max_y;
    x += s_right_nudge;

    frame_t fb = {
        .buf = outbuf, // 传 LE 缓冲给送显任务
        .w = img_w,
        .h = img_h,
        .x = (uint16_t)x,
        .y = (uint16_t)y,
        .bytes = bytes,
    };

    // 队列满就丢旧帧（记得释放旧帧）
    if (xQueueSend(s_q_ready, &fb, 0) != pdPASS)
    {
        frame_t old;
        if (xQueueReceive(s_q_ready, &old, 0) == pdPASS)
        {
            if (old.buf)
                jpeg_free_align(old.buf);
        }
        if (xQueueSend(s_q_ready, &fb, 0) != pdPASS)
        {
            jpeg_free_align(outbuf);
        }
    }
}

static void audio_cb(frame_data_t *data, void *arg)
{
    (void)data;
    (void)arg;
}

/* 播放任务（Core1） */
static void avi_play_task(void *arg)
{
    ESP_LOGI(TAG, "avi_play_task start (Core=%d)", xPortGetCoreID());

    avi_player_handle_t handle = NULL;
    avi_player_config_t cfg = {
        .buffer_size = 256 * 1024,
        .video_cb    = video_cb,
        .audio_cb    = audio_cb,
        .priority    = 5,
        .coreID      = 1,
        .stack_size  = 10 * 1024,
    };

    for (;;) {
        if (g_count <= 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // ★ 关键：播新文件前，确保上一个已释放
        if (handle) {
            avi_player_deinit(handle);
            handle = NULL;
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        const char *path = g_playlist[g_index];
        ESP_LOGI(TAG, "play file: %s (index=%d/%d)", path, g_index+1, g_count);

        if (avi_player_init(cfg, &handle) != ESP_OK) {
            ESP_LOGE(TAG, "avi_player_init failed");
            vTaskDelay(pdMS_TO_TICKS(500));
            // 切下一条，避免卡死某个坏文件
            g_index = (g_index + 1) % g_count;
            continue;
        }

        // 启动播放
        if (avi_player_play_from_file(handle, path) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to play %s", path);
            // 失败也要释放
            avi_player_deinit(handle);
            handle = NULL;
            g_index = (g_index + 1) % g_count;
            continue;
        }

        // 初始化心跳时间（防止刚开头就被判“超时”）
        s_last_frame_us = esp_timer_get_time();

        // 等待：1) BOOT 按键；2) 连续 N 秒无新帧 -> 认为自然结束
        const int idle_ms_to_end = 1500; // 无帧1.5秒视为结束，可按视频帧率调整
        bool next_file = false;

        while (!next_file) {
            // BOOT 切下一条
            uint32_t sig;
            if (g_btn_q && xQueueReceive(g_btn_q, &sig, 0) == pdPASS) {
#ifdef AVI_PLAYER_HAS_STOP
                avi_player_stop(handle);
#else
                avi_player_deinit(handle);
                handle = NULL;
#endif
                next_file = true;
                break;
            }

            // 心跳超时 -> 认为播放结束
            uint64_t now = esp_timer_get_time();
            if ((now - s_last_frame_us) > (uint64_t)idle_ms_to_end * 1000ULL) {
                // 正常结束或异常无帧，关闭并切下一条
                avi_player_deinit(handle);
                handle = NULL;
                next_file = true;
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 切下一条
        g_index = (g_index + 1) % g_count;
        // 可选：黑屏
        // bsp_st7789_fill_screen(s_lcd, BSP_COLOR_BLACK);
    }
}

/* ======= 对外 API ======= */

void decode_pipeline_init(bsp_st7789_t *lcd, int rotation, int backlight_percent, int right_nudge_px)
{
    s_lcd = lcd;
    s_right_nudge = right_nudge_px;

    // 初始化 LCD（如果 main 已经 init 过 LCD，这里只做旋转/背光/清屏也OK）
    bsp_st7789_set_rotation(s_lcd, rotation);
    bsp_st7789_backlight_set(backlight_percent);
    bsp_st7789_fill_screen(s_lcd, BSP_COLOR_BLACK);

    // 分配内部 SRAM 的 DMA 小缓存
    s_dma_chunk = (uint8_t *)heap_caps_malloc(TX_CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_dma_chunk)
    {
        ESP_LOGE(TAG, "alloc DMA chunk failed (%u bytes)", (unsigned)TX_CHUNK_BYTES);
        abort();
    }

    // 创建“就绪帧”队列
    s_q_ready = xQueueCreate(3, sizeof(frame_t));
    if (!s_q_ready)
    {
        ESP_LOGE(TAG, "create queue failed");
        abort();
    }

    // 推屏任务（Core0，优先级略高）
    xTaskCreatePinnedToCore(blit_task, "blit_task", 6 * 1024, NULL, 7, NULL, 0);
}

// 原型改名并改参：传入目录，而不是文件
void decode_play_dir(const char *dirpath)
{
    // 先建播放列表
    g_count = build_playlist(dirpath);
    if (g_count <= 0)
    {
        ESP_LOGE(TAG, "no avi files under %s", dirpath);
        return;
    }

    // 初始化 BOOT 按键（有中断+队列）
    init_boot_button();

    // 启动播放任务（Core1）
    xTaskCreatePinnedToCore(avi_play_task, "avi_play_task", 12 * 1024,
                            NULL, 6, NULL, 1);
}

void decode_set_right_nudge(int px)
{
    s_right_nudge = px;
}