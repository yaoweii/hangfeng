// =============================================================================
// base.cpp — hanfeng 基础几何类型的实现
// =============================================================================
//
// 本文件实现了 include/hanfeng/base.hpp 中声明的所有类方法和全局运算符，
// 包括：
//   - SPAposition 的构造与比较
//   - SPAvector 的构造、长度计算与比较
//   - SPAunit_vector 的构造与归一化
//   - SPAbox 的构造、包含判断
//   - 点/向量之间的加减乘除运算符
//   - 点乘（%）和叉乘（*）运算符
//   - 包围盒的并集（|）、交集（&）、相交判断（&&）运算符
// =============================================================================

#include "hanfeng/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hanfeng {

	// =============================================================================
	// 匿名命名空间 — 内部辅助函数
	// =============================================================================

	namespace {

		/// 归一化时的长度阈值：若向量长度小于等于此值，视为零向量，归一化失败
		constexpr double kUnitVectorEpsilon = 1.0e-12;

		/**
		 * @brief 叉乘计算实现。
		 *
		 * 使用分量展开公式：
		 *   result.x = ay * bz - az * by
		 *   result.y = az * bx - ax * bz
		 *   result.z = ax * by - ay * bx
		 *
		 * @param ax, ay, az  左操作数的 x, y, z 分量
		 * @param bx, by, bz  右操作数的 x, y, z 分量
		 * @return 叉乘结果向量
		 */
		SPAvector cross_impl(const double ax, const double ay, const double az,
			const double bx, const double by, const double bz) {
			return SPAvector(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);
		}

		/**
		 * @brief 点乘计算实现。
		 *
		 * result = ax * bx + ay * by + az * bz
		 *
		 * @param ax, ay, az  左操作数的 x, y, z 分量
		 * @param bx, by, bz  右操作数的 x, y, z 分量
		 * @return 点乘结果（标量）
		 */
		double dot_impl(const double ax, const double ay, const double az,
			const double bx, const double by, const double bz) {
			return ax * bx + ay * by + az * bz;
		}

	}  // namespace

	// =============================================================================
	// SPAposition — 三维空间位置点
	// =============================================================================

	/**
	 * @brief 使用三个坐标分量构造空间点。
	 */
	SPAposition::SPAposition(double x, double y, double z) {
		comp_[0] = x;
		comp_[1] = y;
		comp_[2] = z;
	}

	/**
	 * @brief 严格相等比较：三个分量完全相同。
	 */
	bool SPAposition::operator==(const SPAposition& other) const {
		return comp_[0] == other.comp_[0] && comp_[1] == other.comp_[1] &&
			comp_[2] == other.comp_[2];
	}

	/**
	 * @brief 不等比较：取反 operator==。
	 */
	bool SPAposition::operator!=(const SPAposition& other) const {
		return !(*this == other);
	}

	// =============================================================================
	// SPAvector — 三维向量
	// =============================================================================

	/**
	 * @brief 使用三个分量构造向量。
	 */
	SPAvector::SPAvector(double x, double y, double z) {
		comp_[0] = x;
		comp_[1] = y;
		comp_[2] = z;
	}

	/**
	 * @brief 计算向量长度的平方。
	 *
	 * 返回 x^2 + y^2 + z^2，避免开方运算，适用于长度比较。
	 */
	double SPAvector::len_sq() const {
		return comp_[0] * comp_[0] + comp_[1] * comp_[1] + comp_[2] * comp_[2];
	}

	/**
	 * @brief 计算向量长度（欧几里得范数）。
	 *
	 * 返回 sqrt(len_sq())。
	 */
	double SPAvector::len() const { return std::sqrt(len_sq()); }

	/**
	 * @brief 严格相等比较：三个分量完全相同。
	 */
	bool SPAvector::operator==(const SPAvector& other) const {
		return comp_[0] == other.comp_[0] && comp_[1] == other.comp_[1] &&
			comp_[2] == other.comp_[2];
	}

	/**
	 * @brief 不等比较：取反 operator==。
	 */
	bool SPAvector::operator!=(const SPAvector& other) const {
		return !(*this == other);
	}

	// =============================================================================
	// SPAunit_vector — 单位向量
	// =============================================================================

	/// 默认构造为 (1, 0, 0)，valid_ = true（类内初始值）
	SPAunit_vector::SPAunit_vector() = default;

	/**
	 * @brief 使用三个分量构造，内部进行归一化。
	 */
	SPAunit_vector::SPAunit_vector(double x, double y, double z) {
		assign_from_vector(SPAvector(x, y, z));
	}

	/**
	 * @brief 从普通向量构造，内部进行归一化。
	 */
	SPAunit_vector::SPAunit_vector(const SPAvector& vector) {
		assign_from_vector(vector);
	}

	/**
	 * @brief 将单位向量转为普通向量。
	 *
	 * 单位向量的分量本身就是有效的 double 值，直接构造即可。
	 */
	SPAvector SPAunit_vector::vector() const {
		return SPAvector(comp_[0], comp_[1], comp_[2]);
	}

	/**
	 * @brief 归一化赋值核心实现。
	 *
	 * 步骤：
	 * 1. 计算输入向量的长度
	 * 2. 若长度 <= kUnitVectorEpsilon，视为零向量，回退到 (1, 0, 0)，valid_ = false
	 * 3. 否则将每个分量除以长度，valid_ = true
	 */
	void SPAunit_vector::assign_from_vector(const SPAvector& vector) {
		const double length = vector.len();
		if (length <= kUnitVectorEpsilon) {
			// 零向量：回退到默认方向 (1, 0, 0)，标记为无效
			comp_[0] = 1.0;
			comp_[1] = 0.0;
			comp_[2] = 0.0;
			valid_ = false;
			return;
		}

		// 正常归一化
		comp_[0] = vector.x() / length;
		comp_[1] = vector.y() / length;
		comp_[2] = vector.z() / length;
		valid_ = true;
	}

	// =============================================================================
	// SPAbox — 轴对齐包围盒
	// =============================================================================

	/**
	 * @brief 使用单个点构造退化包围盒。
	 *
	 * 最小角点和最大角点相同，包围盒退化为一个点，valid_ = true。
	 */
	SPAbox::SPAbox(const SPAposition& point)
		: min_corner_(point), max_corner_(point), valid_(true) {}

	/**
	 * @brief 使用两个点构造包围盒。
	 *
	 * 对每个分量取 min/max，确保 min_corner_ <= max_corner_。
	 */
	SPAbox::SPAbox(const SPAposition& first, const SPAposition& second)
		: min_corner_(std::min(first.x(), second.x()),
			std::min(first.y(), second.y()),
			std::min(first.z(), second.z())),
		max_corner_(std::max(first.x(), second.x()),
			std::max(first.y(), second.y()),
			std::max(first.z(), second.z())),
		valid_(true) {}

	/**
	 * @brief 判断点是否在包围盒内部或边界上。
	 *
	 * 使用闭区间语义（>= 和 <=），边界点算作包含。
	 * 若包围盒无效，对任何点返回 false。
	 */
	bool SPAbox::contains(const SPAposition& point) const {
		if (!valid_) {
			return false;
		}

		return point.x() >= min_corner_.x() && point.x() <= max_corner_.x() &&
			point.y() >= min_corner_.y() && point.y() <= max_corner_.y() &&
			point.z() >= min_corner_.z() && point.z() <= max_corner_.z();
	}

	// =============================================================================
	// 点与向量之间的算术运算符实现
	// =============================================================================

	/**
	 * @brief 点减点 → 位移向量。
	 */
	SPAvector operator-(const SPAposition& left, const SPAposition& right) {
		return SPAvector(left.x() - right.x(), left.y() - right.y(),
			left.z() - right.z());
	}

	/**
	 * @brief 点 + 向量 → 平移后的新点。
	 */
	SPAposition operator+(const SPAposition& point, const SPAvector& vector) {
		return SPAposition(point.x() + vector.x(), point.y() + vector.y(),
			point.z() + vector.z());
	}

	/**
	 * @brief 向量 + 点 → 新点（交换律）。
	 */
	SPAposition operator+(const SPAvector& vector, const SPAposition& point) {
		return point + vector;
	}

	/**
	 * @brief 点 - 向量 → 反向平移后的新点。
	 */
	SPAposition operator-(const SPAposition& point, const SPAvector& vector) {
		return SPAposition(point.x() - vector.x(), point.y() - vector.y(),
			point.z() - vector.z());
	}

	/**
	 * @brief 向量取负。
	 */
	SPAvector operator-(const SPAvector& vector) {
		return SPAvector(-vector.x(), -vector.y(), -vector.z());
	}

	/**
	 * @brief 向量加法。
	 */
	SPAvector operator+(const SPAvector& left, const SPAvector& right) {
		return SPAvector(left.x() + right.x(), left.y() + right.y(),
			left.z() + right.z());
	}

	/**
	 * @brief 向量减法。
	 */
	SPAvector operator-(const SPAvector& left, const SPAvector& right) {
		return SPAvector(left.x() - right.x(), left.y() - right.y(),
			left.z() - right.z());
	}

	/**
	 * @brief 向量 * 标量 → 数乘。
	 */
	SPAvector operator*(const SPAvector& vector, double scalar) {
		return SPAvector(vector.x() * scalar, vector.y() * scalar,
			vector.z() * scalar);
	}

	/**
	 * @brief 标量 * 向量 → 数乘（交换律）。
	 */
	SPAvector operator*(double scalar, const SPAvector& vector) {
		return vector * scalar;
	}

	/**
	 * @brief 向量 / 标量 → 数除。
	 */
	SPAvector operator/(const SPAvector& vector, double scalar) {
		return SPAvector(vector.x() / scalar, vector.y() / scalar,
			vector.z() / scalar);
	}

	// =============================================================================
	// 点乘运算符实现（% 运算符）
	// =============================================================================

	/// 两个普通向量的点乘
	double operator%(const SPAvector& left, const SPAvector& right) {
		return dot_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	/// 普通向量与单位向量的点乘
	double operator%(const SPAvector& left, const SPAunit_vector& right) {
		return dot_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	/// 单位向量与普通向量的点乘
	double operator%(const SPAunit_vector& left, const SPAvector& right) {
		return dot_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	/// 两个单位向量的点乘
	double operator%(const SPAunit_vector& left, const SPAunit_vector& right) {
		return dot_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	// =============================================================================
	// 叉乘运算符实现（* 运算符，两个向量之间）
	// =============================================================================

	/// 两个普通向量的叉乘
	SPAvector operator*(const SPAvector& left, const SPAvector& right) {
		return cross_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	/// 普通向量与单位向量的叉乘
	SPAvector operator*(const SPAvector& left, const SPAunit_vector& right) {
		return cross_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	/// 单位向量与普通向量的叉乘
	SPAvector operator*(const SPAunit_vector& left, const SPAvector& right) {
		return cross_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	/// 两个单位向量的叉乘
	SPAvector operator*(const SPAunit_vector& left, const SPAunit_vector& right) {
		return cross_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	// =============================================================================
	// 归一化函数
	// =============================================================================

	/**
	 * @brief 将普通向量归一化为单位向量。
	 *
	 * 委托给 SPAunit_vector 的构造函数完成归一化。
	 */
	SPAunit_vector normalise(const SPAvector& vector) {
		return SPAunit_vector(vector);
	}

	// =============================================================================
	// 包围盒运算符实现
	// =============================================================================

	/**
	 * @brief 包围盒并集（| 运算符）。
	 *
	 * 规则：
	 * - 若 left 无效，直接返回 right
	 * - 若 right 无效，直接返回 left
	 * - 两者都有效时，对各分量取 min 构造新最小角点，取 max 构造新最大角点
	 */
	SPAbox operator|(const SPAbox& left, const SPAbox& right) {
		if (!left.valid_) {
			return right;
		}
		if (!right.valid_) {
			return left;
		}

		return SPAbox(
			SPAposition(std::min(left.min_corner_.x(), right.min_corner_.x()),
				std::min(left.min_corner_.y(), right.min_corner_.y()),
				std::min(left.min_corner_.z(), right.min_corner_.z())),
			SPAposition(std::max(left.max_corner_.x(), right.max_corner_.x()),
				std::max(left.max_corner_.y(), right.max_corner_.y()),
				std::max(left.max_corner_.z(), right.max_corner_.z())));
	}

	/**
	 * @brief 包围盒交集（& 运算符）。
	 *
	 * 规则：
	 * - 若两者不相交（通过 && 判断），返回无效包围盒
	 * - 否则对各分量取 max 构造新最小角点，取 min 构造新最大角点
	 */
	SPAbox operator&(const SPAbox& left, const SPAbox& right) {
		if (!(left && right)) {
			return SPAbox();  // 不相交，返回无效包围盒
		}

		return SPAbox(
			SPAposition(std::max(left.min_corner_.x(), right.min_corner_.x()),
				std::max(left.min_corner_.y(), right.min_corner_.y()),
				std::max(left.min_corner_.z(), right.min_corner_.z())),
			SPAposition(std::min(left.max_corner_.x(), right.max_corner_.x()),
				std::min(left.max_corner_.y(), right.max_corner_.y()),
				std::min(left.max_corner_.z(), right.max_corner_.z())));
	}

	/**
	 * @brief 判断两个包围盒是否相交（&& 运算符）。
	 *
	 * 使用分离轴定理（SAT）在三个坐标轴上分别检测。
	 * 闭区间语义：边界接触也算相交。
	 * 任一包围盒无效时返回 false。
	 */
	bool operator&&(const SPAbox& left, const SPAbox& right) {
		if (!left.valid_ || !right.valid_) {
			return false;
		}

		// 三个轴上分别检测是否重叠
		return left.min_corner_.x() <= right.max_corner_.x() &&
			left.max_corner_.x() >= right.min_corner_.x() &&
			left.min_corner_.y() <= right.max_corner_.y() &&
			left.max_corner_.y() >= right.min_corner_.y() &&
			left.min_corner_.z() <= right.max_corner_.z() &&
			left.max_corner_.z() >= right.min_corner_.z();
	}

}  // namespace hanfeng
