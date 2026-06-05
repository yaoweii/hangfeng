// =============================================================================
// base.hpp — hanfeng 项目的三维几何基础类型定义
// =============================================================================
//
// 本文件定义了 hanfeng 项目中最底层的几何类型，包括：
//   - SPAposition : 三维空间中的位置点（对应 ACIS 中的 SPAposition）
//   - SPAvector   : 三维向量（方向 + 位移）
//   - SPAunit_vector : 单位向量（归一化方向）
//   - SPAbox      : 轴对齐包围盒（AABB）
//
// 同时声明了这些类型之间的算术运算符，遵循 ACIS/GME 的命名风格。
// 点与向量是两个不同概念：
//   - 点 - 点 = 向量
//   - 点 +/- 向量 = 点
//   - 向量 +/- 向量 = 向量
//
// 运算符约定：
//   - `%` 运算符表示点乘（dot product）
//   - `*` 运算符在两个向量之间表示叉乘（cross product）
//   - `*` 运算符在向量与标量之间表示数乘
//   - `|` 运算符在两个包围盒之间表示并集
//   - `&` 运算符在两个包围盒之间表示交集
//   - `&&` 运算符在两个包围盒之间判断是否相交
// =============================================================================

#ifndef HANFENG_GEOMETRY_HPP
#define HANFENG_GEOMETRY_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace hanfeng {

	// 前向声明，用于在运算符签名中引用尚未完整定义的类型
	class SPAvector;
	class SPAunit_vector;
	class SPAbox;

	// =============================================================================
	// SPAposition — 三维空间位置点
	// =============================================================================

	/**
	 * @brief 表示三维空间中的一个位置点。
	 *
	 * 这个类对齐 ACIS/GME 中 `SPAposition` 的命名风格，用于表达几何意义上的"点"。
	 * 点和向量是两个不同概念：
	 * - 点减点得到向量
	 * - 点加/减向量得到点
	 *
	 * 内部存储为三个 double 分量 comp_[3]，分别对应 x、y、z 坐标。
	 */
	class SPAposition {
	public:
		/**
		 * @brief 默认构造为原点 `(0, 0, 0)`。
		 *
		 * 成员数组 comp_ 使用类内初始化 = {0.0, 0.0, 0.0}，
		 * 因此默认构造即为坐标原点。
		 */
		SPAposition() = default;

		/**
		 * @brief 使用三个坐标分量构造空间点。
		 *
		 * @param x  x 坐标值
		 * @param y  y 坐标值
		 * @param z  z 坐标值
		 */
		SPAposition(double x, double y, double z);

		/**
		 * @brief 返回 x 坐标。
		 */
		double x() const { return comp_[0]; }

		/**
		 * @brief 返回 y 坐标。
		 */
		double y() const { return comp_[1]; }

		/**
		 * @brief 返回 z 坐标。
		 */
		double z() const { return comp_[2]; }

		/**
		 * @brief 访问指定坐标分量（只读）。
		 *
		 * 下标约定：
		 * - `0` 对应 x
		 * - `1` 对应 y
		 * - `2` 对应 z
		 *
		 * @param index 分量下标，取值 0/1/2
		 * @return 对应坐标值
		 */
		double operator[](std::size_t index) const { return comp_[index]; }

		/**
		 * @brief 访问并修改指定坐标分量（可写）。
		 *
		 * @param index 分量下标，取值 0/1/2
		 * @return 对应坐标值的可写引用
		 */
		double& operator[](std::size_t index) { return comp_[index]; }

		/**
		 * @brief 严格比较两个点的坐标是否完全相等。
		 *
		 * 使用 `==` 逐分量比较，不考虑浮点误差。
		 *
		 * @param other 另一个点
		 * @return 三个分量完全相同时返回 true
		 */
		bool operator==(const SPAposition& other) const;

		/**
		 * @brief 严格比较两个点的坐标是否不相等。
		 *
		 * @param other 另一个点
		 * @return 任意分量不同时返回 true
		 */
		bool operator!=(const SPAposition& other) const;

	private:
		double comp_[3] = { 0.0, 0.0, 0.0 };  ///< 内部存储：x, y, z 三个坐标分量
	};

	// =============================================================================
	// SPAvector — 三维向量
	// =============================================================================

	/**
	 * @brief 表示三维空间中的普通向量。
	 *
	 * 这个类用于表达方向和位移，不保证长度为 1。
	 * 内部存储与 SPAposition 相同，为三个 double 分量。
	 */
	class SPAvector {
	public:
		/**
		 * @brief 默认构造为零向量 `(0, 0, 0)`。
		 */
		SPAvector() = default;

		/**
		 * @brief 使用三个分量构造向量。
		 *
		 * @param x  x 分量
		 * @param y  y 分量
		 * @param z  z 分量
		 */
		SPAvector(double x, double y, double z);

		/**
		 * @brief 返回 x 分量。
		 */
		double x() const { return comp_[0]; }

		/**
		 * @brief 返回 y 分量。
		 */
		double y() const { return comp_[1]; }

		/**
		 * @brief 返回 z 分量。
		 */
		double z() const { return comp_[2]; }

		/**
		 * @brief 访问指定向量分量（只读）。
		 *
		 * @param index 分量下标，取值 0/1/2
		 * @return 对应分量值
		 */
		double operator[](std::size_t index) const { return comp_[index]; }

		/**
		 * @brief 访问并修改指定向量分量（可写）。
		 *
		 * @param index 分量下标，取值 0/1/2
		 * @return 对应分量值的可写引用
		 */
		double& operator[](std::size_t index) { return comp_[index]; }

		/**
		 * @brief 返回向量长度的平方。
		 *
		 * 计算 x^2 + y^2 + z^2，比 len() 更高效（省去开方运算），
		 * 常用于长度比较场景。
		 *
		 * @return 向量长度平方值
		 */
		double len_sq() const;

		/**
		 * @brief 返回向量长度（欧几里得范数）。
		 *
		 * 计算 sqrt(x^2 + y^2 + z^2)。
		 *
		 * @return 向量长度
		 */
		double len() const;

		/**
		 * @brief 严格比较两个向量的分量是否完全相等。
		 *
		 * @param other 另一个向量
		 * @return 三个分量完全相同时返回 true
		 */
		bool operator==(const SPAvector& other) const;

		/**
		 * @brief 严格比较两个向量的分量是否不相等。
		 *
		 * @param other 另一个向量
		 * @return 任意分量不同时返回 true
		 */
		bool operator!=(const SPAvector& other) const;

	private:
		double comp_[3] = { 0.0, 0.0, 0.0 };  ///< 内部存储：x, y, z 三个分量
	};

	// =============================================================================
	// SPAunit_vector — 单位向量
	// =============================================================================

	/**
	 * @brief 表示单位向量。
	 *
	 * 该类在构造时会自动尝试归一化输入方向：
	 * - 若输入长度足够大（大于 kUnitVectorEpsilon），则保存归一化结果，并标记为有效
	 * - 若输入接近零向量，则回退到默认方向 `(1, 0, 0)`，并标记为无效
	 *
	 * 通过 is_valid() 可以检查归一化是否成功。
	 */
	class SPAunit_vector {
	public:
		/**
		 * @brief 默认构造为 `(1, 0, 0)`，且视为有效单位向量。
		 */
		SPAunit_vector();

		/**
		 * @brief 使用三个分量构造单位向量。
		 *
		 * 内部会先将 (x, y, z) 视为普通向量，再归一化。
		 * 若输入为零向量，则回退到 `(1, 0, 0)` 并标记为无效。
		 *
		 * @param x  x 分量
		 * @param y  y 分量
		 * @param z  z 分量
		 */
		SPAunit_vector(double x, double y, double z);

		/**
		 * @brief 使用普通向量构造单位向量。
		 *
		 * 对输入向量进行归一化。若输入为零向量，则回退到 `(1, 0, 0)` 并标记为无效。
		 *
		 * @param vector 输入的普通向量
		 */
		explicit SPAunit_vector(const SPAvector& vector);

		/**
		 * @brief 返回 x 分量。
		 */
		double x() const { return comp_[0]; }

		/**
		 * @brief 返回 y 分量。
		 */
		double y() const { return comp_[1]; }

		/**
		 * @brief 返回 z 分量。
		 */
		double z() const { return comp_[2]; }

		/**
		 * @brief 访问指定分量（只读）。
		 *
		 * @param index 分量下标，取值 0/1/2
		 * @return 对应分量值
		 */
		double operator[](std::size_t index) const { return comp_[index]; }

		/**
		 * @brief 返回该单位向量是否由非零输入成功归一化得到。
		 *
		 * 如果输入向量的长度小于等于内部阈值 kUnitVectorEpsilon，
		 * 则归一化失败，is_valid() 返回 false，此时内部回退到 (1, 0, 0)。
		 *
		 * @return true 表示归一化成功，false 表示输入接近零向量
		 */
		bool is_valid() const { return valid_; }

		/**
		 * @brief 将单位向量转换成普通向量。
		 *
		 * 因为单位向量的分量本身就是合法的 double 值，
		 * 所以直接用分量构造一个 SPAvector 即可。
		 *
		 * @return 等价方向的普通向量
		 */
		SPAvector vector() const;

	private:
		/**
		 * @brief 内部归一化赋值实现。
		 *
		 * 计算输入向量的长度：
		 * - 若长度 > kUnitVectorEpsilon，将每个分量除以长度，标记 valid_ = true
		 * - 否则回退到 (1, 0, 0)，标记 valid_ = false
		 *
		 * @param vector 待归一化的输入向量
		 */
		void assign_from_vector(const SPAvector& vector);

		double comp_[3] = { 1.0, 0.0, 0.0 };  ///< 内部存储：归一化后的 x, y, z 分量，默认 (1, 0, 0)
		bool valid_ = true;                   ///< 归一化是否成功的标记
	};

	// =============================================================================
	// SPAbox — 轴对齐包围盒（Axis-Aligned Bounding Box）
	// =============================================================================

	/**
	 * @brief 表示三维轴对齐包围盒（AABB）。
	 *
	 * 当前版本内部直接保存最小角点、最大角点和有效标记，
	 * 对外提供 ACIS 风格的基础查询与操作符语义。
	 *
	 * 无效包围盒（is_valid() == false）不包含任何点，
	 * 参与并集运算时会被忽略（invalid | valid = valid）。
	 */
	class SPAbox {
	public:
		/**
		 * @brief 默认构造为无效包围盒。
		 *
		 * valid_ 默认为 false，min_corner_ 和 max_corner_ 均为原点。
		 * 无效包围盒不包含任何点。
		 */
		SPAbox() = default;

		/**
		 * @brief 使用单个点构造退化但有效的包围盒。
		 *
		 * 最小角点和最大角点相同，包围盒退化为一个点。
		 *
		 * @param point 包围盒的唯一顶点
		 */
		explicit SPAbox(const SPAposition& point);

		/**
		 * @brief 使用两个点构造包围盒，内部会自动整理最小/最大角点。
		 *
		 * 对两个点的每个分量取 min/max，确保 min_corner_ <= max_corner_。
		 *
		 * @param first  任意角点
		 * @param second 任意角点
		 */
		SPAbox(const SPAposition& first, const SPAposition& second);

		/**
		 * @brief 返回包围盒是否有效。
		 *
		 * 只有通过构造函数或并集/交集运算产生的有效包围盒才返回 true。
		 * 默认构造的包围盒为无效。
		 */
		bool is_valid() const { return valid_; }

		/**
		 * @brief 返回最小角点。
		 *
		 * 仅在 `is_valid() == true` 时才有几何意义。
		 * 最小角点的每个分量都 <= 最大角点的对应分量。
		 */
		const SPAposition& min_corner() const { return min_corner_; }

		/**
		 * @brief 返回最大角点。
		 *
		 * 仅在 `is_valid() == true` 时才有几何意义。
		 */
		const SPAposition& max_corner() const { return max_corner_; }

		/**
		 * @brief 判断点是否位于包围盒内部或边界上。
		 *
		 * 当前采用闭区间语义，边界点也算包含。
		 * 若包围盒无效，则对任何点都返回 false。
		 *
		 * @param point 待检测的空间点
		 * @return true 表示点在包围盒内或边界上
		 */
		bool contains(const SPAposition& point) const;

	private:
		SPAposition min_corner_{};   ///< 最小角点（各分量取较小值）
		SPAposition max_corner_{};   ///< 最大角点（各分量取较大值）
		bool valid_ = false;          ///< 包围盒是否有效的标记

		// 友元声明：允许全局运算符函数访问私有成员
		friend SPAbox operator|(const SPAbox& left, const SPAbox& right);
		friend SPAbox operator&(const SPAbox& left, const SPAbox& right);
		friend bool operator&&(const SPAbox& left, const SPAbox& right);
	};

	// =============================================================================
	// 点与向量之间的算术运算符
	// =============================================================================

	/**
	 * @brief 点减点得到位移向量。
	 *
	 * (left.x - right.x, left.y - right.y, left.z - right.z)
	 */
	SPAvector operator-(const SPAposition& left, const SPAposition& right);

	/**
	 * @brief 点沿向量正方向平移，得到新点。
	 *
	 * (point.x + vector.x, point.y + vector.y, point.z + vector.z)
	 */
	SPAposition operator+(const SPAposition& point, const SPAvector& vector);

	/**
	 * @brief 向量加到点上（交换律形式），得到新点。
	 *
	 * 等价于 point + vector。
	 */
	SPAposition operator+(const SPAvector& vector, const SPAposition& point);

	/**
	 * @brief 点沿向量反方向平移，得到新点。
	 *
	 * (point.x - vector.x, point.y - vector.y, point.z - vector.z)
	 */
	SPAposition operator-(const SPAposition& point, const SPAvector& vector);

	/**
	 * @brief 向量取负（方向取反）。
	 *
	 * (-vector.x, -vector.y, -vector.z)
	 */
	SPAvector operator-(const SPAvector& vector);

	/**
	 * @brief 向量加法。
	 *
	 * (left.x + right.x, left.y + right.y, left.z + right.z)
	 */
	SPAvector operator+(const SPAvector& left, const SPAvector& right);

	/**
	 * @brief 向量减法。
	 *
	 * (left.x - right.x, left.y - right.y, left.z - right.z)
	 */
	SPAvector operator-(const SPAvector& left, const SPAvector& right);

	/**
	 * @brief 向量与标量相乘（向量在前）。
	 *
	 * (vector.x * scalar, vector.y * scalar, vector.z * scalar)
	 */
	SPAvector operator*(const SPAvector& vector, double scalar);

	/**
	 * @brief 标量与向量相乘（标量在前）。
	 *
	 * 等价于 vector * scalar。
	 */
	SPAvector operator*(double scalar, const SPAvector& vector);

	/**
	 * @brief 向量与标量相除。
	 *
	 * (vector.x / scalar, vector.y / scalar, vector.z / scalar)
	 */
	SPAvector operator/(const SPAvector& vector, double scalar);

	// =============================================================================
	// 点乘运算符（% 运算符）
	// =============================================================================

	/**
	 * @brief 计算两个普通向量的点乘。
	 *
	 * result = left.x * right.x + left.y * right.y + left.z * right.z
	 */
	double operator%(const SPAvector& left, const SPAvector& right);

	/**
	 * @brief 计算普通向量与单位向量的点乘。
	 */
	double operator%(const SPAvector& left, const SPAunit_vector& right);

	/**
	 * @brief 计算单位向量与普通向量的点乘。
	 */
	double operator%(const SPAunit_vector& left, const SPAvector& right);

	/**
	 * @brief 计算两个单位向量的点乘。
	 */
	double operator%(const SPAunit_vector& left, const SPAunit_vector& right);

	// =============================================================================
	// 叉乘运算符（* 运算符，用于两个向量之间）
	// =============================================================================

	/**
	 * @brief 计算两个普通向量的叉乘。
	 *
	 * result.x = left.y * right.z - left.z * right.y
	 * result.y = left.z * right.x - left.x * right.z
	 * result.z = left.x * right.y - left.y * right.x
	 */
	SPAvector operator*(const SPAvector& left, const SPAvector& right);

	/**
	 * @brief 计算普通向量与单位向量的叉乘。
	 */
	SPAvector operator*(const SPAvector& left, const SPAunit_vector& right);

	/**
	 * @brief 计算单位向量与普通向量的叉乘。
	 */
	SPAvector operator*(const SPAunit_vector& left, const SPAvector& right);

	/**
	 * @brief 计算两个单位向量的叉乘。
	 */
	SPAvector operator*(const SPAunit_vector& left, const SPAunit_vector& right);

	// =============================================================================
	// 自由函数
	// =============================================================================

	/**
	 * @brief 将普通向量归一化为单位向量。
	 *
	 * 等价于 SPAunit_vector(vector)，如果输入为零向量，
	 * 则回退到 (1, 0, 0) 并标记为无效。
	 *
	 * @param vector 待归一化的普通向量
	 * @return 归一化后的单位向量
	 */
	SPAunit_vector normalise(const SPAvector& vector);

	// =============================================================================
	// 包围盒运算符
	// =============================================================================

	/**
	 * @brief 对包围盒求并集（`|` 运算符）。
	 *
	 * 约定：
	 * - `invalid | valid = valid`  （无效盒被忽略）
	 * - `invalid | invalid = invalid`
	 * - `valid | valid` = 包含两者的最小包围盒
	 *
	 * @param left  左操作数
	 * @param right 右操作数
	 * @return 并集包围盒
	 */
	SPAbox operator|(const SPAbox& left, const SPAbox& right);

	/**
	 * @brief 对包围盒求交集（`&` 运算符）。
	 *
	 * 若两者不相交，则返回无效包围盒。
	 *
	 * @param left  左操作数
	 * @param right 右操作数
	 * @return 交集包围盒（不相交时为无效）
	 */
	SPAbox operator&(const SPAbox& left, const SPAbox& right);

	/**
	 * @brief 判断两个包围盒是否相交（`&&` 运算符）。
	 *
	 * 当前采用闭区间语义，边界接触也视为相交。
	 * 任一包围盒无效时返回 false。
	 *
	 * @param left  左操作数
	 * @param right 右操作数
	 * @return true 表示两个包围盒有交集
	 */
	bool operator&&(const SPAbox& left, const SPAbox& right);

}  // namespace hanfeng

#endif
