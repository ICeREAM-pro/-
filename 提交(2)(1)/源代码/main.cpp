/**
 * @file main.cpp
 * @brief 双轮差速移动机器人控制台测试程序
 *
 * 功能：
 * 1. 测试理想轮速计算
 * 2. 测试偏移量计算
 * 3. 测试补偿算法
 * 4. 模拟"初始偏移→补偿→稳定"过程
 *
 * 作者：郭雨诺
 * 日期：2025-11-17
 */

#include "kinematics.h"
#include "config.h"
#include <stdio.h>
#include <math.h>

// ============================================================================
// 测试函数声明 (Test Functions Declaration)
// ============================================================================

/**
 * @brief 测试1：理想轮速计算测试
 */
void test_ideal_speed();

/**
 * @brief 测试2：偏移量计算测试
 */
void test_offset_calculation();

/**
 * @brief 测试3：补偿算法测试
 */
void test_compensation();

/**
 * @brief 测试4：完整闭环控制仿真（初始偏移→补偿→稳定）
 * @param initial_offset  初始偏移量 (cm)
 * @param test_name       测试用例名称
 */
void test_closed_loop_control(float initial_offset, const char* test_name);

// ============================================================================
// 主函数 (Main Function)
// ============================================================================

int main() {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║   双轮差速移动机器人 - 原地旋转中心调节算法测试程序      ║\n");
    printf("║   Differential Drive Robot - Rotation Center Control     ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("\n");

    printf("=== 系统参数配置 ===\n");
    printf("轮距 W = %.1f cm\n", WHEEL_BASE_W);
    printf("基准转速 V₀ = %.1f cm/s\n", BASE_SPEED_V0);
    printf("补偿系数 k = %.2f cm/(s·cm)\n", COMPENSATION_COEFF_K);
    printf("偏移阈值 = %.2f cm\n", OFFSET_THRESHOLD);
    printf("成功标准：|Δd| ≤ %.2f cm\n\n", SUCCESS_OFFSET_TOLERANCE);

    // 运行所有测试
    test_ideal_speed();
    test_offset_calculation();
    test_compensation();

    // 完整闭环控制仿真
    test_closed_loop_control(0.0f, "测试用例1：初始无偏移");
    test_closed_loop_control(3.0f, "测试用例2：初始偏右3.0cm");
    test_closed_loop_control(-2.0f, "测试用例3：初始偏左2.0cm");

    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║                     所有测试完成                          ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("\n");

    return 0;
}

// ============================================================================
// 测试函数实现 (Test Functions Implementation)
// ============================================================================

/**
 * @brief 测试1：理想轮速计算测试
 */
void test_ideal_speed() {
    printf("\n");
    printf("┌───────────────────────────────────────────────────────────┐\n");
    printf("│ 测试1：理想轮速计算                                       │\n");
    printf("└───────────────────────────────────────────────────────────┘\n");

    float V_L, V_R;
    calc_ideal_speed(&V_L, &V_R);

    print_wheel_speeds("理想轮速", V_L, V_R);

    // 计算角速度
    float omega = calc_angular_velocity(V_L, V_R);
    printf("[理论角速度] ω = 2V₀/W = 2×%.1f/%.1f = %.4f rad/s = %.2f deg/s\n",
           BASE_SPEED_V0, WHEEL_BASE_W,
           2.0f * BASE_SPEED_V0 / WHEEL_BASE_W,
           2.0f * BASE_SPEED_V0 / WHEEL_BASE_W * RAD_TO_DEG);

    printf("\n[验证] V_L = -V_R? %s\n",
           (fabs(V_L + V_R) < 1e-6f) ? "✓ 通过" : "✗ 失败");
    printf("[验证] 角速度计算正确? %s\n",
           (fabs(omega - 2.0f * BASE_SPEED_V0 / WHEEL_BASE_W) < 1e-6f) ? "✓ 通过" : "✗ 失败");
    printf("\n");
}

/**
 * @brief 测试2：偏移量计算测试
 */
void test_offset_calculation() {
    printf("\n");
    printf("┌───────────────────────────────────────────────────────────┐\n");
    printf("│ 测试2：偏移量计算                                         │\n");
    printf("└───────────────────────────────────────────────────────────┘\n");

    // 测试用例数据
    struct TestCase {
        float V_L;
        float V_R;
        float expected_offset;
        const char* description;
    } test_cases[] = {
        {-5.0f, 5.0f, 0.0f, "理想状态（无偏移）"},
        {-3.0f, 7.0f, -6.0f, "左轮慢，偏左"},
        {-7.0f, 3.0f, 6.0f, "右轮慢，偏右"},
        {-4.0f, 6.0f, -3.0f, "偏左3cm"},
        {-6.0f, 4.0f, 3.0f, "偏右3cm"}
    };

    int num_cases = sizeof(test_cases) / sizeof(test_cases[0]);

    for (int i = 0; i < num_cases; i++) {
        printf("\n--- 测试用例 %d: %s ---\n", i + 1, test_cases[i].description);
        float offset = calc_offset(test_cases[i].V_L, test_cases[i].V_R);

        print_wheel_speeds("输入轮速", test_cases[i].V_L, test_cases[i].V_R);
        printf("[计算偏移量] Δd = %.2f cm\n", offset);
        printf("[预期偏移量] Δd_expected = %.2f cm\n", test_cases[i].expected_offset);

        // 验证（允许小误差）
        if (fabs(offset - test_cases[i].expected_offset) < 0.1f) {
            printf("[验证] ✓ 通过\n");
        } else {
            printf("[验证] ✗ 失败（误差：%.2f cm）\n",
                   fabs(offset - test_cases[i].expected_offset));
        }
    }

    printf("\n");
}

/**
 * @brief 测试3：补偿算法测试
 */
void test_compensation() {
    printf("\n");
    printf("┌───────────────────────────────────────────────────────────┐\n");
    printf("│ 测试3：补偿算法                                           │\n");
    printf("└───────────────────────────────────────────────────────────┘\n");

    // 测试用例：不同的偏移量
    float offsets[] = {0.0f, 3.0f, -2.0f, 5.0f, -4.0f};
    int num_cases = sizeof(offsets) / sizeof(offsets[0]);

    for (int i = 0; i < num_cases; i++) {
        float delta_d = offsets[i];
        float V_L, V_R;

        printf("\n--- 测试用例 %d: 偏移量 Δd = %.1f cm ---\n", i + 1, delta_d);

        // 计算补偿后的轮速
        compensate_speed(delta_d, &V_L, &V_R);

        print_wheel_speeds("补偿后轮速", V_L, V_R);

        // 验证：计算补偿后的偏移量（应该接近0）
        float new_offset = calc_offset(V_L, V_R);
        printf("[补偿后偏移] Δd_new = %.3f cm\n", new_offset);

        if (fabs(new_offset) < 0.01f) {
            printf("[验证] ✓ 补偿成功，偏移量消除\n");
        } else {
            printf("[验证] ⚠ 补偿后仍有残余偏移（%.3f cm）\n", new_offset);
        }
    }

    printf("\n");
}

/**
 * @brief 测试4：完整闭环控制仿真（初始偏移→补偿→稳定）
 */
void test_closed_loop_control(float initial_offset, const char* test_name) {
    printf("\n");
    printf("┌───────────────────────────────────────────────────────────┐\n");
    printf("│ 测试4：闭环控制仿真 - %s\n", test_name);
    printf("└───────────────────────────────────────────────────────────┘\n");

    printf("\n初始偏移量：Δd = %.2f cm\n", initial_offset);

    // 模拟参数
    float delta_d = initial_offset;  // 当前偏移量
    float V_L, V_R;                  // 当前轮速
    float time = 0.0f;               // 仿真时间
    int iteration = 0;               // 迭代次数
    int converged = 0;               // 是否收敛

    printf("\n迭代过程：\n");
    printf("%-4s | %-8s | %-10s | %-10s | %-10s | %-6s\n",
           "次数", "时间(s)", "Δd(cm)", "V_L(cm/s)", "V_R(cm/s)", "状态");
    printf("─────┼──────────┼────────────┼────────────┼────────────┼────────\n");

    // 仿真循环：最多模拟10秒或100次迭代
    while (time < SIMULATION_DURATION && iteration < 100) {
        // 计算补偿后的轮速
        compensate_speed(delta_d, &V_L, &V_R);

        // 检查是否收敛
        if (fabs(delta_d) <= SUCCESS_OFFSET_TOLERANCE) {
            if (!converged) {
                printf("%-4d | %-8.2f | %+10.3f | %+10.2f | %+10.2f | %s\n",
                       iteration, time, delta_d, V_L, V_R, "✓ 收敛");
                converged = 1;
                break;  // 收敛后停止仿真
            }
        } else {
            // 打印当前状态（每5次迭代打印一次）
            if (iteration % 5 == 0) {
                printf("%-4d | %-8.2f | %+10.3f | %+10.2f | %+10.2f | %s\n",
                       iteration, time, delta_d, V_L, V_R,
                       (delta_d > 0) ? "偏右" : "偏左");
            }
        }

        // 更新偏移量（模拟补偿效果）
        // 假设每次补偿后，偏移量减小50%（简化模型）
        delta_d = delta_d * 0.5f;

        // 更新时间
        time += TIME_STEP;
        iteration++;
    }

    printf("\n=== 仿真结果 ===\n");
    printf("初始偏移：Δd_init = %.2f cm\n", initial_offset);
    printf("最终偏移：Δd_final = %.3f cm\n", delta_d);
    printf("收敛时间：t = %.2f s (%d 次迭代)\n", time, iteration);
    printf("收敛状态：%s\n", converged ? "✓ 成功收敛" : "✗ 未收敛");

    // 验证是否满足成功标准
    printf("\n=== 成功标准验证 ===\n");
    printf("1. 偏移量精度：|Δd| ≤ %.2f cm? %s (实际：%.3f cm)\n",
           SUCCESS_OFFSET_TOLERANCE,
           (fabs(delta_d) <= SUCCESS_OFFSET_TOLERANCE) ? "✓ 通过" : "✗ 失败",
           fabs(delta_d));

    printf("2. 收敛时间：t ≤ %.1f s? %s (实际：%.2f s)\n",
           SUCCESS_CONVERGE_TIME,
           (time <= SUCCESS_CONVERGE_TIME) ? "✓ 通过" : "✗ 失败",
           time);

    printf("3. 速度精度：|V_L + V_R| ≤ %.1f cm/s? %s (实际：%.2f cm/s)\n",
           SUCCESS_SPEED_TOLERANCE,
           (fabs(V_L + V_R) <= SUCCESS_SPEED_TOLERANCE) ? "✓ 通过" : "✗ 失败",
           fabs(V_L + V_R));

    printf("\n");
}
