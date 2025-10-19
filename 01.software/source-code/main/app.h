#pragma once

#include "bsp.h"

// 初始化视频流水线（创建任务/队列、申请DMA块等）
// rotation: 调用后内部会对 lcd 做旋转（比如 1=90°）
// backlight_percent: 背光 0~100
// right_nudge_px: 逻辑坐标系下的右移像素（你的面板需要 +20）
void decode_pipeline_init(bsp_st7789_t *lcd,
                          int rotation,
                          int backlight_percent,
                          int right_nudge_px);

// 播放一个 AVI 文件（异步在后台任务播放）
void decode_play_dir(const char *path);

// 可选：运行中动态调整右移像素
void decode_set_right_nudge(int px);


void init_boot_button(void);
int build_playlist(const char *dirpath);