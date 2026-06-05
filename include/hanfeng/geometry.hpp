// =============================================================================
// geometry.hpp — 三维几何基础类型定义
// =============================================================================
//
// 定义 hanfeng 项目最底层的几何类型及运算符：
//   - SPAposition  : 三维空间位置点
//   - SPAvector    : 三维向量
//   - SPAunit_vector : 单位向量（归一化方向）
//   - SPAbox       : 轴对齐包围盒（AABB）
//
// 运算符约定（沿用 ACIS/GME 风格）：
//   - %  : 点乘（dot product）
//   - *  : 两个向量之间为叉乘（cross product）；向量与标量之间为数乘
//   - |  : 包围盒并集
//   - &  : 包围盒交集
//   - && : 包围盒相交判断
// =============================================================================

#ifndef HANFENG_GEOMETRY_HPP
#define HANFENG_GEOMETRY_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace hanfeng {

	// 前向声明
	class SPAvector;
	class SPAunit_vector;
	class SPAbox;

	// =============================================================================
	// SPAposition — 三维空间位置点
	// =============================================================================

	/// 三维空间位置点。点与向量是不同概念：点-点=向量，点+/-向量=点。
	/// 内部存储为 comp_[3]，对应 x/y/z 坐标。
	class SPAposition {
	public:
		SPAposition() = default;
		SPAposition(double x, double y, double z);

		double x() const { return comp_[0]; }
		double y() const { return comp_[1]; }
		double z() const { return comp_[2]; }

		double operator[](std::size_t index) const { return comp_[index]; }
		double& operator[](std::size_t index) { return comp_[index]; }

		/// 严格相等比较（逐分量 ==，不考虑浮点误差）
		bool operator==(const SPAposition& other) const;
		bool operator!=(const SPAposition& other) const;

	private:
		double comp_[3] = { 0.0, 0.0, 0.0 };
	};

	// =============================================================================
	// SPAvector — 三维向量
	// =============================================================================

	/// 三维向量，不保证长度为 1。
	class SPAvector {
	public:
		SPAvector() = default;
		SPAvector(double x, double y, double z);

		double x() const { return comp_[0]; }
		double y() const { return comp_[1]; }
		double z() const { return comp_[2]; }

		double operator[](std::size_t index) const { return comp_[index]; }
		double& operator[](std::size_t index) { return comp_[index]; }

		/// 向量长度平方（省去开方，适合比较场景）
		double len_sq() const;

		/// 向量长度（欧几里得范数）
		double len() const;

		/// 严格相等比较（逐分量 ==）
		bool operator==(const SPAvector& other) const;
		bool operator!=(const SPAvector& other) const;

	private:
		double comp_[3] = { 0.0, 0.0, 0.0 };
	};

	// =============================================================================
	// SPAunit_vector — 单位向量
	// =============================================================================

	/// 单位向量。构造时自动归一化：
	/// - 输入长度 > 阈值：保存归一化结果，is_valid() = true
	/// - 输入接近零向量：回退到 (1,0,0)，is_valid() = false
	class SPAunit_vector {
	public:
		SPAunit_vector();
		SPAunit_vector(double x, double y, double z);
		explicit SPAunit_vector(const SPAvector& vector);

		double x() const { return comp_[0]; }
		double y() const { return comp_[1]; }
		double z() const { return comp_[2]; }

		double operator[](std::size_t index) const { return comp_[index]; }

		bool is_valid() const { return valid_; }

		/// 转为普通向量
		SPAvector vector() const;

	private:
		/// 归一化赋值：长度 > 阈值则归一化，否则回退到 (1,0,0)
		void assign_from_vector(const SPAvector& vector);

		double comp_[3] = { 1.0, 0.0, 0.0 };
		bool valid_ = true;
	};

	// =============================================================================
	// SPAbox — 轴对齐包围盒（AABB）
	// =============================================================================

	/// 轴对齐包围盒。无效包围盒不包含任何点，参与并集运算时被忽略。
	class SPAbox {
	public:
		SPAbox() = default;
		explicit SPAbox(const SPAposition& point);
		SPAbox(const SPAposition& first, const SPAposition& second);

		bool is_valid() const { return valid_; }
		const SPAposition& min_corner() const { return min_corner_; }
		const SPAposition& max_corner() const { return max_corner_; }

		/// 判断点是否在包围盒内或边界上（闭区间语义）。无效盒对任何点返回 false。
		bool contains(const SPAposition& point) const;

	private:
		SPAposition min_corner_{};
		SPAposition max_corner_{};
		bool valid_ = false;

		friend SPAbox operator|(const SPAbox& left, const SPAbox& right);
		friend SPAbox operator&(const SPAbox& left, const SPAbox& right);
		friend bool operator&&(const SPAbox& left, const SPAbox& right);
	};

	// =============================================================================
	// 点与向量的算术运算符
	// =============================================================================

	SPAvector operator-(const SPAposition& left, const SPAposition& right);
	SPAposition operator+(const SPAposition& point, const SPAvector& vector);
	SPAposition operator+(const SPAvector& vector, const SPAposition& point);
	SPAposition operator-(const SPAposition& point, const SPAvector& vector);
	SPAvector operator-(const SPAvector& vector);
	SPAvector operator+(const SPAvector& left, const SPAvector& right);
	SPAvector operator-(const SPAvector& left, const SPAvector& right);
	SPAvector operator*(const SPAvector& vector, double scalar);
	SPAvector operator*(double scalar, const SPAvector& vector);
	SPAvector operator/(const SPAvector& vector, double scalar);

	// =============================================================================
	// 点乘（% 运算符）
	// =============================================================================

	double operator%(const SPAvector& left, const SPAvector& right);
	double operator%(const SPAvector& left, const SPAunit_vector& right);
	double operator%(const SPAunit_vector& left, const SPAvector& right);
	double operator%(const SPAunit_vector& left, const SPAunit_vector& right);

	// =============================================================================
	// 叉乘（* 运算符，两个向量之间）
	// =============================================================================

	SPAvector operator*(const SPAvector& left, const SPAvector& right);
	SPAvector operator*(const SPAvector& left, const SPAunit_vector& right);
	SPAvector operator*(const SPAunit_vector& left, const SPAvector& right);
	SPAvector operator*(const SPAunit_vector& left, const SPAunit_vector& right);

	// =============================================================================
	// 自由函数
	// =============================================================================

	/// 将普通向量归一化为单位向量。零向量输入回退到 (1,0,0) 并标记无效。
	SPAunit_vector normalise(const SPAvector& vector);

	// =============================================================================
	// 包围盒运算符
	// =============================================================================

	/// 并集：invalid | valid = valid；两者有效时取包围两者的最小盒
	SPAbox operator|(const SPAbox& left, const SPAbox& right);

	/// 交集：不相交时返回无效盒
	SPAbox operator&(const SPAbox& left, const SPAbox& right);

	/// 相交判断：闭区间语义，边界接触算相交。任一无效返回 false。
	bool operator&&(const SPAbox& left, const SPAbox& right);

}  // namespace hanfeng

#endif
