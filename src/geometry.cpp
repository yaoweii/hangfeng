// =============================================================================
// geometry.cpp — 三维几何基础类型的实现
// =============================================================================
//
// 实现 geometry.hpp 中声明的类方法与全局运算符：
//   - SPAposition / SPAvector / SPAunit_vector 构造与比较
//   - SPAbox 构造、包含判断
//   - 点/向量加减乘除运算符
//   - 点乘（%）、叉乘（*）、包围盒并交运算符
// =============================================================================

#include "hanfeng/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hanfeng {

	namespace {

		/// 归一化长度阈值：向量长度 <= 此值视为零向量
		constexpr double kUnitVectorEpsilon = 1.0e-12;

		SPAvector cross_impl(double ax, double ay, double az,
			double bx, double by, double bz) {
			return SPAvector(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);
		}

		double dot_impl(double ax, double ay, double az,
			double bx, double by, double bz) {
			return ax * bx + ay * by + az * bz;
		}

	}  // namespace

	// =============================================================================
	// SPAposition
	// =============================================================================

	SPAposition::SPAposition(double x, double y, double z) {
		comp_[0] = x;
		comp_[1] = y;
		comp_[2] = z;
	}

	bool SPAposition::operator==(const SPAposition& other) const {
		return comp_[0] == other.comp_[0] && comp_[1] == other.comp_[1] &&
			comp_[2] == other.comp_[2];
	}

	bool SPAposition::operator!=(const SPAposition& other) const {
		return !(*this == other);
	}

	// =============================================================================
	// SPAvector
	// =============================================================================

	SPAvector::SPAvector(double x, double y, double z) {
		comp_[0] = x;
		comp_[1] = y;
		comp_[2] = z;
	}

	double SPAvector::len_sq() const {
		return comp_[0] * comp_[0] + comp_[1] * comp_[1] + comp_[2] * comp_[2];
	}

	double SPAvector::len() const { return std::sqrt(len_sq()); }

	bool SPAvector::operator==(const SPAvector& other) const {
		return comp_[0] == other.comp_[0] && comp_[1] == other.comp_[1] &&
			comp_[2] == other.comp_[2];
	}

	bool SPAvector::operator!=(const SPAvector& other) const {
		return !(*this == other);
	}

	// =============================================================================
	// SPAunit_vector
	// =============================================================================

	SPAunit_vector::SPAunit_vector() = default;

	SPAunit_vector::SPAunit_vector(double x, double y, double z) {
		assign_from_vector(SPAvector(x, y, z));
	}

	SPAunit_vector::SPAunit_vector(const SPAvector& vector) {
		assign_from_vector(vector);
	}

	SPAvector SPAunit_vector::vector() const {
		return SPAvector(comp_[0], comp_[1], comp_[2]);
	}

	void SPAunit_vector::assign_from_vector(const SPAvector& vector) {
		const double length = vector.len();
		if (length <= kUnitVectorEpsilon) {
			// 零向量回退到 (1,0,0)
			comp_[0] = 1.0;
			comp_[1] = 0.0;
			comp_[2] = 0.0;
			valid_ = false;
			return;
		}

		comp_[0] = vector.x() / length;
		comp_[1] = vector.y() / length;
		comp_[2] = vector.z() / length;
		valid_ = true;
	}

	// =============================================================================
	// SPAbox
	// =============================================================================

	SPAbox::SPAbox(const SPAposition& point)
		: min_corner_(point), max_corner_(point), valid_(true) {}

	SPAbox::SPAbox(const SPAposition& first, const SPAposition& second)
		: min_corner_(std::min(first.x(), second.x()),
			std::min(first.y(), second.y()),
			std::min(first.z(), second.z())),
		max_corner_(std::max(first.x(), second.x()),
			std::max(first.y(), second.y()),
			std::max(first.z(), second.z())),
		valid_(true) {}

	bool SPAbox::contains(const SPAposition& point) const {
		if (!valid_) {
			return false;
		}

		return point.x() >= min_corner_.x() && point.x() <= max_corner_.x() &&
			point.y() >= min_corner_.y() && point.y() <= max_corner_.y() &&
			point.z() >= min_corner_.z() && point.z() <= max_corner_.z();
	}

	// =============================================================================
	// 点与向量之间的算术运算符
	// =============================================================================

	SPAvector operator-(const SPAposition& left, const SPAposition& right) {
		return SPAvector(left.x() - right.x(), left.y() - right.y(),
			left.z() - right.z());
	}

	SPAposition operator+(const SPAposition& point, const SPAvector& vector) {
		return SPAposition(point.x() + vector.x(), point.y() + vector.y(),
			point.z() + vector.z());
	}

	SPAposition operator+(const SPAvector& vector, const SPAposition& point) {
		return point + vector;
	}

	SPAposition operator-(const SPAposition& point, const SPAvector& vector) {
		return SPAposition(point.x() - vector.x(), point.y() - vector.y(),
			point.z() - vector.z());
	}

	SPAvector operator-(const SPAvector& vector) {
		return SPAvector(-vector.x(), -vector.y(), -vector.z());
	}

	SPAvector operator+(const SPAvector& left, const SPAvector& right) {
		return SPAvector(left.x() + right.x(), left.y() + right.y(),
			left.z() + right.z());
	}

	SPAvector operator-(const SPAvector& left, const SPAvector& right) {
		return SPAvector(left.x() - right.x(), left.y() - right.y(),
			left.z() - right.z());
	}

	SPAvector operator*(const SPAvector& vector, double scalar) {
		return SPAvector(vector.x() * scalar, vector.y() * scalar,
			vector.z() * scalar);
	}

	SPAvector operator*(double scalar, const SPAvector& vector) {
		return vector * scalar;
	}

	SPAvector operator/(const SPAvector& vector, double scalar) {
		return SPAvector(vector.x() / scalar, vector.y() / scalar,
			vector.z() / scalar);
	}

	// =============================================================================
	// 点乘运算符（%）
	// =============================================================================

	double operator%(const SPAvector& left, const SPAvector& right) {
		return dot_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	double operator%(const SPAvector& left, const SPAunit_vector& right) {
		return dot_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	double operator%(const SPAunit_vector& left, const SPAvector& right) {
		return dot_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	double operator%(const SPAunit_vector& left, const SPAunit_vector& right) {
		return dot_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	// =============================================================================
	// 叉乘运算符（*，两个向量之间）
	// =============================================================================

	SPAvector operator*(const SPAvector& left, const SPAvector& right) {
		return cross_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	SPAvector operator*(const SPAvector& left, const SPAunit_vector& right) {
		return cross_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	SPAvector operator*(const SPAunit_vector& left, const SPAvector& right) {
		return cross_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	SPAvector operator*(const SPAunit_vector& left, const SPAunit_vector& right) {
		return cross_impl(left.x(), left.y(), left.z(), right.x(), right.y(),
			right.z());
	}

	// =============================================================================
	// 归一化
	// =============================================================================

	SPAunit_vector normalise(const SPAvector& vector) {
		return SPAunit_vector(vector);
	}

	// =============================================================================
	// 包围盒运算符
	// =============================================================================

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

	SPAbox operator&(const SPAbox& left, const SPAbox& right) {
		if (!(left && right)) {
			return SPAbox();
		}

		return SPAbox(
			SPAposition(std::max(left.min_corner_.x(), right.min_corner_.x()),
				std::max(left.min_corner_.y(), right.min_corner_.y()),
				std::max(left.min_corner_.z(), right.min_corner_.z())),
			SPAposition(std::min(left.max_corner_.x(), right.max_corner_.x()),
				std::min(left.max_corner_.y(), right.max_corner_.y()),
				std::min(left.max_corner_.z(), right.max_corner_.z())));
	}

	/// 相交判断：SAT 三轴检测，闭区间语义（边界接触算相交）
	bool operator&&(const SPAbox& left, const SPAbox& right) {
		if (!left.valid_ || !right.valid_) {
			return false;
		}

		return left.min_corner_.x() <= right.max_corner_.x() &&
			left.max_corner_.x() >= right.min_corner_.x() &&
			left.min_corner_.y() <= right.max_corner_.y() &&
			left.max_corner_.y() >= right.min_corner_.y() &&
			left.min_corner_.z() <= right.max_corner_.z() &&
			left.max_corner_.z() >= right.min_corner_.z();
	}

}  // namespace hanfeng
