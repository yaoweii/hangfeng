// =============================================================================
// base_geometry_test.cpp — 基础几何类型（SPAposition、SPAvector、SPAbox）的测试
// =============================================================================
//
// 测试覆盖：
//   - 点与向量的算术运算（加减、平移）
//   - 向量长度、单位化
//   - 包围盒的包含判断、并集、交集、相交判断
//
// 使用 assert 宏进行断言，配合 nearly_equal() 做浮点数比较。
// =============================================================================

#include <cassert>

#include "hanfeng/geometry.hpp"
#include "test_common.hpp"

/**
 * @brief 测试 SPAposition 和 SPAvector 的基本算术运算。
 *
 * 验证：
 * - 点 + 向量 = 平移后的点
 * - 点 - 点 = 位移向量
 * - 向量长度平方的正确性
 * - 向量归一化为单位向量后的有效性
 */
void test_spa_position_and_vector_arithmetic() {
	const hanfeng::SPAposition origin(1.0, 2.0, 3.0);
	const hanfeng::SPAvector offset(4.0, -1.0, 0.5);
	const hanfeng::SPAposition moved = origin + offset;  // 平移：(1+4, 2-1, 3+0.5) = (5, 1, 3.5)
	const hanfeng::SPAvector delta = moved - origin;      // 位移向量：(4, -1, 0.5)
	const hanfeng::SPAunit_vector direction(delta);        // 归一化为单位向量

	// 验证平移后的点坐标
	assert(nearly_equal(moved.x(), 5.0));
	assert(nearly_equal(moved.y(), 1.0));
	assert(nearly_equal(moved.z(), 3.5));
	// 验证位移向量的长度平方：4^2 + (-1)^2 + 0.5^2 = 16 + 1 + 0.25 = 17.25
	assert(nearly_equal(delta.len_sq(), 17.25));
	// 验证单位向量归一化成功且长度为 1
	assert(direction.is_valid());
	assert(nearly_equal(direction.vector().len(), 1.0));
}

/**
 * @brief 测试 SPAbox 的各种语义。
 *
 * 验证：
 * - 无效包围盒不包含任何点
 * - 有效包围盒的包含判断（闭区间语义，边界点算包含）
 * - 两个包围盒的相交判断（边界接触也算相交）
 * - 并集运算：invalid | valid = valid
 * - 交集运算：重叠区域的正确性
 * - 不相交包围盒的交集为无效包围盒
 */
void test_spa_box_semantics() {
	// 构造一个无效包围盒和两个有效包围盒
	const hanfeng::SPAbox invalid_box;
	const hanfeng::SPAbox first(hanfeng::SPAposition(0.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 10.0, 10.0));
	const hanfeng::SPAbox second(hanfeng::SPAposition(10.0, 5.0, 5.0),
		hanfeng::SPAposition(20.0, 15.0, 15.0));

	// 无效包围盒不包含任何点
	assert(!invalid_box.is_valid());
	// first 包含其角点（闭区间）
	assert(first.contains(hanfeng::SPAposition(0.0, 0.0, 0.0)));
	assert(first.contains(hanfeng::SPAposition(10.0, 10.0, 10.0)));
	// first 和 second 在 (10, y, z) 面上接触，视为相交
	assert(first && second);

	// 并集：invalid | first = first
	const hanfeng::SPAbox merged = invalid_box | first;
	assert(merged.is_valid());
	assert(merged.contains(hanfeng::SPAposition(5.0, 5.0, 5.0)));

	// 交集：first ∩ second = [10,10] x [5,10] x [5,10]
	const hanfeng::SPAbox overlap = first & second;
	assert(overlap.is_valid());
	assert(overlap.contains(hanfeng::SPAposition(10.0, 7.5, 7.5)));

	// 不相交的包围盒
	const hanfeng::SPAbox disjoint(hanfeng::SPAposition(30.0, 30.0, 30.0),
		hanfeng::SPAposition(40.0, 40.0, 40.0));
	assert(!(first && disjoint));      // 不相交
	assert(!(first & disjoint).is_valid());  // 交集为无效
}
