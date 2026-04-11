#include "target_state.h"

// 全局目标状态：由控制任务周期更新，被云台控制逻辑只读消费。
static target_state_t target_state = {0};

void TargetState_Init(void)
{
    // 冷启动复位所有字段，等待第一帧视觉数据到来。
    target_state.valid = 0U;
    target_state.fresh = 0U;
    target_state.seq = 0U;
    target_state.raw_x = 0U;
    target_state.raw_y = 0U;
    target_state.filtered_x = 0.0f;
    target_state.filtered_y = 0.0f;
    target_state.last_update_tick = 0U;
}

void TargetState_Update(void)
{
    vision_input_frame_t frame;

    // 尽可能取空当前周期积压的视觉帧，最终保留最新状态。
    while (VisionInput_FetchFrame(&frame))
    {
        target_state.raw_x = frame.x;
        target_state.raw_y = frame.y;
        target_state.seq = frame.seq;
        target_state.last_update_tick = frame.tick;
        target_state.fresh = 1U;
        target_state.valid = 1U;

        // 第一帧直接赋值；后续帧使用一阶指数平滑降低抖动。
        if (target_state.filtered_x == 0.0f && target_state.filtered_y == 0.0f)
        {
            target_state.filtered_x = (fp32)frame.x;
            target_state.filtered_y = (fp32)frame.y;
        }
        else
        {
            target_state.filtered_x +=
              ((fp32)frame.x - target_state.filtered_x) * TARGET_STATE_SMOOTH_ALPHA;
            target_state.filtered_y +=
              ((fp32)frame.y - target_state.filtered_y) * TARGET_STATE_SMOOTH_ALPHA;
        }
    }

    // 超时未收到新帧则判定目标失效，避免继续使用陈旧目标。
    if (target_state.valid &&
        (HAL_GetTick() - target_state.last_update_tick) > TARGET_STATE_TIMEOUT_MS)
    {
        target_state.valid = 0U;
        target_state.fresh = 0U;
    }
}

void TargetState_Inject(uint16_t x, uint16_t y)
{
    // 调试/测试注入入口：直接写入目标并标记为新鲜有效。
    target_state.raw_x = x;
    target_state.raw_y = y;
    target_state.filtered_x = (fp32)x;
    target_state.filtered_y = (fp32)y;
    target_state.valid = 1U;
    target_state.fresh = 1U;
    target_state.last_update_tick = HAL_GetTick();
}

void TargetState_SetReady(uint8_t ready)
{
    // fresh 表示“是否有未消费的新目标”。
    target_state.fresh = ready ? 1U : 0U;

    if (ready == 0U)
    {
        target_state.valid = 0U;
    }
}

const target_state_t *TargetState_Get(void)
{
    // 对外提供只读视图，调用者不应修改内部状态。
    return &target_state;
}

uint8_t TargetState_FetchPosition(uint16_t *x, uint16_t *y)
{
    if (x == NULL || y == NULL || target_state.valid == 0U)
    {
        return 0U;
    }

    *x = (uint16_t)target_state.filtered_x;
    *y = (uint16_t)target_state.filtered_y;
    // 成功取走位置后，清除 fresh 标志。
    target_state.fresh = 0U;
    return 1U;
}
