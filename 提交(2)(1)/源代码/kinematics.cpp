/**
 * @file kinematics.cpp
 * @brief 双轮差速移动机器人运动学算法实现文件
 *
 * 实现原地旋转运动学建模、偏移量计算和补偿控制的核心算法
 * 作者：郭雨诺
 * 日期：2025-11-17
 */

#include "kinematics.h"
#include "config.h"
#include <stdio.h>
#include <math.h>

// ============================================================================
// 核心算法函数实现 (Core Algorithm Implementation)
// ============================================================================

/**
 * @brief 计算理想原地旋转的轮速
 *
 * 理想原地旋转：左右轮速度大小相等、方向相反
 * V_L = -V₀ (向后), V_R = +V₀ (向前)
 */
void calc_ideal_speed(float* V_L, float* V_R) {
    // 检查输入参数
    if (V_L == NULL || V_R == NULL) {
        fprintf(stderr, "[ERROR] calc_ideal_speed: 空指针输入\n");
        return;
    }

    // 理想轮速：大小相等、方向相反
    *V_L = -BASE_SPEED_V0;  // 左轮向后（负值）
    *V_R = +BASE_SPEED_V0;  // 右轮向前（正值）

#if ENABLE_VERBOSE_LOG
    printf("[INFO] 理想轮速计算完成: V_L = %.2f cm/s, V_R = %.2f cm/s\n",
           *V_L, *V_R);
#endif
}

/**
 * @brief 计算旋转中心偏移量
 *
 * 公式推导：
 * 1. 左轮到旋转中心距离：r_L = W × |V_L| / (|V_L| + |V_R|)
 * 2. 偏移量：Δd = r_L - W/2
 * 3. 简化：Δd = (W/2) × (|V_L| - |V_R|) / (|V_L| + |V_R|)
 */
float calc_offset(float V_L, float V_R) {
    // 计算速度的绝对值
    float abs_V_L = fabs(V_L);
    float abs_V_R = fabs(V_R);

    // 计算速度之和（分母）
    float sum_speed = abs_V_L + abs_V_R;

    // 特殊情况：两轮速度都为0（静止状态）
    if (sum_speed < 1e-6f) {
#if ENABLE_VERBOSE_LOG
        printf("[WARN] calc_offset: 轮速为0，无法计算偏移量\n");
#endif
        return 0.0f;
    }

    // 特殊情况：两轮同向运动（不是原地旋转）
    // 如果 V_L 和 V_R 同号，说明不是原地旋转
    if ((V_L > 0 && V_R > 0) || (V_L < 0 && V_R < 0)) {
#if ENABLE_VERBOSE_LOG
        printf("[WARN] calc_offset: 两轮同向运动，不是原地旋转\n");
#endif
        return 0.0f;
    }

    // 计算偏移量：Δd = (W/2) × (|V_L| - |V_R|) / (|V_L| + |V_R|)
    float delta_d = (WHEEL_BASE_W / 2.0f) * (abs_V_L - abs_V_R) / sum_speed;

#if ENABLE_VERBOSE_LOG
    printf("[INFO] 偏移量计算: |V_L|=%.2f, |V_R|=%.2f, Δd=%.3f cm",
           abs_V_L, abs_V_R, delta_d);
    if (delta_d > 0) {
        printf(" (偏右)\n");
    } else if (delta_d < 0) {
        printf(" (偏左)\n");
    } else {
        printf(" (无偏移)\n");
    }
#endif

    // 检查偏移量是否超出允许范围
    if (fabs(delta_d) > MAX_OFFSET) {
        fprintf(stderr, "[ERROR] 偏移量超出范围: Δd=%.2f cm (最大允许%.2f cm)\n",
                delta_d, MAX_OFFSET);
    }

    return delta_d;
}

/**
 * @brief 计算补偿后的轮速
 *
 * 补偿策略：
 * 1. 计算补偿速度：ΔV = k × Δd
 * 2. 应用补偿：
 *    V_L = -V₀ + ΔV
 *    V_R = +V₀ - ΔV
 *
 * 补偿效果：
 * - 当 Δd > 0（偏右）：ΔV > 0，V_L增大（绝对值减小），V_R减小，旋转中心左移
 * - 当 Δd < 0（偏左）：ΔV < 0，V_L减小（绝对值增大），V_R增大，旋转中心右移
 */
void compensate_speed(float delta_d, float* V_L, float* V_R) {
    // 检查输入参数
    if (V_L == NULL || V_R == NULL) {
        fprintf(stderr, "[ERROR] compensate_speed: 空指针输入\n");
        return;
    }

    // 如果偏移量在阈值范围内，不需要补偿
    if (fabs(delta_d) < OFFSET_THRESHOLD) {
        // 直接使用理想轮速
        *V_L = -BASE_SPEED_V0;
        *V_R = +BASE_SPEED_V0;

#if ENABLE_VERBOSE_LOG
        printf("[INFO] 偏移量在阈值内(%.3f cm)，无需补偿\n", delta_d);
#endif
        return;
    }

    // 计算补偿速度：ΔV = k × Δd
    float delta_V = COMPENSATION_COEFF_K * delta_d;

    // 应用补偿
    *V_L = -BASE_SPEED_V0 + delta_V;
    *V_R = +BASE_SPEED_V0 - delta_V;

#if ENABLE_VERBOSE_LOG
    printf("[INFO] 补偿计算: Δd=%.3f cm, ΔV=%.3f cm/s\n", delta_d, delta_V);
    printf("[INFO] 补偿后轮速: V_L=%.2f cm/s, V_R=%.2f cm/s\n", *V_L, *V_R);
#endif

    // 验证补偿后的偏移量（理论上应该为0）
#if ENABLE_VERBOSE_LOG
    float new_offset = calc_offset(*V_L, *V_R);
    printf("[INFO] 补偿后偏移量验证: Δd_new=%.3f cm\n", new_offset);
#endif
}

/**
 * @brief 计算角速度
 *
 * 公式：ω = (V_R - V_L) / W
 *
 * 逆时针旋转（V_R > -V_L）：ω > 0
 * 顺时针旋转（V_R < -V_L）：ω < 0
 */
float calc_angular_velocity(float V_L, float V_R) {
    // 角速度公式：ω = (V_R - V_L) / W
    float omega = (V_R - V_L) / WHEEL_BASE_W;

#if ENABLE_VERBOSE_LOG
    printf("[INFO] 角速度计算: ω = %.4f rad/s = %.2f deg/s\n",
           omega, omega * RAD_TO_DEG);
#endif

    return omega;
}

// ============================================================================
// 辅助函数实现 (Helper Functions Implementation)
// ============================================================================

#if ENABLE_VERBOSE_LOG
/**
 * @brief 打印轮速信息
 */
void print_wheel_speeds(const char* label, float V_L, float V_R) {
    printf("[%s] V_L = %+.2f cm/s, V_R = %+.2f cm/s, |V_L+V_R| = %.2f cm/s\n",
           label, V_L, V_R, fabs(V_L + V_R));
}

/**
 * @brief 打印机器人状态
 */
void print_robot_state(const RobotState* state) {
    if (state == NULL) {
        fprintf(stderr, "[ERROR] print_robot_state: 空指针输入\n");
        return;
    }

    printf("\n=== 机器人状态 (t=%.2f s) ===\n", state->timestamp);
    print_wheel_speeds("当前轮速", state->speeds.V_L, state->speeds.V_R);
    printf("[偏移量] Δd = %+.3f cm", state->offset_d);

    if (state->offset_d > OFFSET_THRESHOLD) {
        printf(" (偏右)\n");
    } else if (state->offset_d < -OFFSET_THRESHOLD) {
        printf(" (偏左)\n");
    } else {
        printf(" (无偏移)\n");
    }

    printf("[角速度] ω = %.4f rad/s = %.2f deg/s\n",
           state->angular_vel, state->angular_vel * RAD_TO_DEG);

    // 判断是否满足成功标准
    if (is_offset_acceptable(state->offset_d)) {
        printf("[状态] ✓ 偏移量在允许范围内\n");
    } else {
        printf("[状态] ✗ 偏移量超出允许范围，需要补偿\n");
    }

    printf("=============================\n\n");
}
#endif

/**
 * @brief 检查偏移量是否在允许范围内
 */
int is_offset_acceptable(float delta_d) {
    return fabs(delta_d) <= SUCCESS_OFFSET_TOLERANCE;
}
