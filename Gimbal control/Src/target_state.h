#ifndef TARGET_STATE_H
#define TARGET_STATE_H

#include "struct_typedef.h"
#include "vision_input.h"

#define TARGET_STATE_TIMEOUT_MS      100U
#define TARGET_STATE_SMOOTH_ALPHA    0.22f

typedef struct
{
    uint8_t valid;// 是否有效
    uint8_t fresh;// 是否为新数据
    uint8_t seq;// 序列号
    uint16_t raw_x;// 原始x坐标
    uint16_t raw_y;// 原始y坐标
    fp32 filtered_x;// 滤波后的x坐标
    fp32 filtered_y;// 滤波后的y坐标
    uint32_t last_update_tick;// 最后更新时间戳
} target_state_t;// 目标状态结构体
// 目标阶段初始化
void TargetState_Init(void);
// 目标阶段更新
void TargetState_Update(void);
// 目标阶段注入
void TargetState_Inject(uint16_t x, uint16_t y);
// 目标阶段设置就绪
void TargetState_SetReady(uint8_t ready);
// 目标阶段获取状态
const target_state_t *TargetState_Get(void);
// 目标阶段获取位置
uint8_t TargetState_FetchPosition(uint16_t *x, uint16_t *y);

#endif
