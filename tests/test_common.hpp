// =============================================================================
// test_common.hpp — 测试辅助工具
// =============================================================================
//
// 提供测试中常用的浮点数比较工具函数。
// =============================================================================

#ifndef HANFENG_TEST_COMMON_HPP
#define HANFENG_TEST_COMMON_HPP

#include <cmath>

/**
 * @brief 判断两个浮点数是否近似相等。
 *
 * 使用 |lhs - rhs| <= epsilon 的方式判断，
 * 默认 epsilon = 1e-9，比 kCurveTolerance (1e-6) 更严格，
 * 适用于测试中对精度要求较高的场景。
 *
 * @param lhs     左操作数
 * @param rhs     右操作数
 * @param epsilon 容差阈值，默认 1.0e-9
 * @return true 表示两数在容差范围内相等
 */
inline bool nearly_equal(double lhs, double rhs, double epsilon = 1.0e-9) {
	return std::fabs(lhs - rhs) <= epsilon;
}

#endif
