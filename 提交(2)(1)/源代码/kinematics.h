/**
 * @file kinematics.h
 * @brief 双轮差速移动机器人运动学算法头文件
 *
 * 提供原地旋转运动学建模、偏移量计算和补偿控制的核心算法
 * 作者：郭雨诺
 * 日期：2025-11-17
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

// ============================================================================
// 数据结构定义 (Data Structures)
// ============================================================================

/**
 * @struct WheelSpeeds
 * @brief 轮速数据结构
 *
 * 存储左右主动轮的线速度
 * 正值表示向前，负值表示向后
 */
typedef struct {
    float V_L;  /**< 左主动轮线速度 (cm/s) */
    float V_R;  /**< 右主动轮线速度 (cm/s) */
} WheelSpeeds;

/**
 * @struct RobotState
 * @brief 机器人状态数据结构
 *
 * 存储机器人的运动状态信息
 */
typedef struct {
    float offset_d;      /**< 旋转中心偏移量 Δd (cm)，正值=偏右，负值=偏左 */
    float angular_vel;   /**< 角速度 ω (rad/s)，逆时针为正 */
    float timestamp;     /**< 时间戳 (秒) */
    WheelSpeeds speeds;  /**< 当前轮速 */
} RobotState;

// ============================================================================
// 核心算法函数 (Core Algorithm Functions)
// ============================================================================

/**
 * @brief 计算理想原地旋转的轮速
 *
 * 理想原地旋转条件：
 * - 左右轮速度大小相等、方向相反
 * - V_L = -V₀，V_R = +V₀
 * - 旋转中心位于几何中心O（后两轮连线中点）
 *
 * @param[out] V_L  输出左轮理想速度 (cm/s)
 * @param[out] V_R  输出右轮理想速度 (cm/s)
 *
 * @note 基准转速V₀在配置文件中定义（默认5.0 cm/s）
 */
void calc_ideal_speed(float* V_L, float* V_R);

/**
 * @brief 计算旋转中心偏移量
 *
 * 根据实际左右轮速度，反推实际旋转中心位置，计算其相对于
 * 几何中心O的偏移量Δd
 *
 * 公式：Δd = (W/2) × (|V_L| - |V_R|) / (|V_L| + |V_R|)
 *
 * 其中：
 * - W：轮距 (cm)
 * - Δd > 0：偏右（实际旋转中心在O点右侧）
 * - Δd < 0：偏左（实际旋转中心在O点左侧）
 * - Δd = 0：无偏移（理想状态）
 *
 * @param[in] V_L  左轮实际速度 (cm/s)
 * @param[in] V_R  右轮实际速度 (cm/s)
 * @return 偏移量 Δd (cm)
 *
 * @note 如果 V_L 和 V_R 同号（同向运动），函数返回0（不是原地旋转）
 */
float calc_offset(float V_L, float V_R);

/**
 * @brief 计算补偿后的轮速
 *
 * 采用比例控制（P控制），根据偏移量Δd计算补偿速度ΔV，
 * 调整左右轮速度，使旋转中心回归几何中心O
 *
 * 补偿策略：
 * - 偏右（Δd > 0）：增加左轮速度，减小右轮速度，使旋转中心左移
 * - 偏左（Δd < 0）：减小左轮速度，增加右轮速度，使旋转中心右移
 *
 * 公式：
 *   ΔV = k × Δd
 *   V_L_compensated = -V₀ + ΔV
 *   V_R_compensated = +V₀ - ΔV
 *
 * @param[in]  delta_d  偏移量 Δd (cm)
 * @param[out] V_L      输出补偿后的左轮速度 (cm/s)
 * @param[out] V_R      输出补偿后的右轮速度 (cm/s)
 *
 * @note 补偿系数k在配置文件中定义（默认0.5 cm/(s·cm)）
 */
void compensate_speed(float delta_d, float* V_L, float* V_R);

/**
 * @brief 计算角速度
 *
 * 根据左右轮速度计算机器人的角速度
 *
 * 公式：ω = (V_R - V_L) / W
 *
 * @param[in] V_L  左轮速度 (cm/s)
 * @param[in] V_R  右轮速度 (cm/s)
 * @return 角速度 ω (rad/s)，逆时针为正
 */
float calc_angular_velocity(float V_L, float V_R);

// ============================================================================
// 辅助函数 (Helper Functions)
// ============================================================================

#if ENABLE_VERBOSE_LOG
/**
 * @brief 打印轮速信息
 *
 * 格式化输出左右轮速度，用于调试
 *
 * @param[in] label  描述标签（如 "理想轮速"、"补偿后轮速"）
 * @param[in] V_L    左轮速度 (cm/s)
 * @param[in] V_R    右轮速度 (cm/s)
 */
void print_wheel_speeds(const char* label, float V_L, float V_R);

/**
 * @brief 打印机器人状态
 *
 * 格式化输出机器人完整状态信息
 *
 * @param[in] state  机器人状态结构体指针
 */
void print_robot_state(const RobotState* state);
#endif

/**
 * @brief 检查偏移量是否在允许范围内
 *
 * @param[in] delta_d  偏移量 (cm)
 * @return 1 = 在允许范围内，0 = 超出范围
 */
int is_offset_acceptable(float delta_d);

#endif // KINEMATICS_H
