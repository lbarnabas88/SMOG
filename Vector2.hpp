/*
 * Vector2.hpp
 *
 *  Created on: 2013.07.28.
 *      Author: baarnus
 */

#ifndef VECTOR2_HPP_
#define VECTOR2_HPP_

#include <iostream>
#include <math.h>

namespace math
{

/*********************************
 * Definition
 *********************************/

template<typename T>
class Vector2
{
public:
	/**
	 * The coordinates
	 */
	T x, y;

	/**
	 * Default constructor
	 */
	Vector2();

	/**
	 * Constructor with coordinates
	 * @param x the first coordinate of the vector
	 * @param y the second coordinate of the vector
	 */
	Vector2(T x, T y);

	/**
	 * Contructor with other type vector
	 * @param v other vector
	 */
	template<typename U>
	Vector2(const Vector2<U>& v);

	/**
	 * Assignment operator
	 * @param other vector
	 */
	template<typename U>
	inline Vector2& operator =(const Vector2<U>& v);

    /**
     * Equal operator
     * @return true if all coordinates are equal
     */
    template<typename U>
    inline bool operator ==(const Vector2<U>& v) const;

	/**
	 * Get size of the vector
	 * @return unsigned long size of this vector
	 */
	static const size_t size = 2;

	/**
	 * Set coordinates
	 * @param x the first coordinate of the vector
	 * @param y the second coordinate of the vector
	 */
	inline void set(T x, T y);

	/**
	 * Get as T array
	 * @return pointer to the "coordinates array"
	 */
	inline const T* array() const;

	/**
	 * Access operator
	 * @param i index of the coordinate (0 -> x, 1 -> y)
	 */
	T& operator[](const size_t& i);

	/**
	 * Constant access operator
	 * @param i index of the coordinate (0 -> x, 1 -> y)
	 */
	const T& operator[](const size_t& i) const;

	/**
	 * Add two vector
	 * @param v vector to add
	 * @return (self + v) vector
	 */
	inline const Vector2 add(const Vector2& v) const;

	/**
	 * Subtract two vector
	 * @param v vector to subtract
	 * @return (self - v) vector
	 */
	inline const Vector2 sub(const Vector2& v) const;

	/**
	 * Multply with scalar
	 * @param l scalar multiplier
	 * @return (l*self) vector
	 */
	template<typename U>
	inline const Vector2 mul(const U& l) const;

	/**
	 * Divide by scalar
	 * @param l scalar divider
	 * @return (self/l) vector
	 */
	template<typename U>
	inline const Vector2 div(const U& l) const;

	/**
	 * Get length of the vector
	 * @return length of the vector
     */
    inline const T length() const;

	/**
	 * Get length square of the vector
	 * @return the square of the vector's length
     */
    inline const T length2() const;

	/**
	 * Normalize the vector
	 * @return normalized vector
	 */
	inline const Vector2 normalized() const;

	/**
	 * Dot product calculation
	 * @param v second vector in the product
	 * @return dot product of the vectors
	 */
	inline const T dot(const Vector2& v) const;

	/**
	 * Cross product of the 2 template<typename T>vector
	 * In 2D it's only a number (the third coordinate, becouse the first 2 is 0)!
	 * @param v second vector in the product
	 * @return cross product of the vectors
	 */
	inline const T cross(const Vector2& v) const;

	/**
	 * Consine of the inner angle of the 2 vector
	 * @param v second vector
	 * @return cosine of the inner angle
     */
    const T cos(const Vector2& v) const;

	/**
	 * Sine of the inner angle of the 2 vector
	 * @param v second vector
	 * @return sine of the inner angle
     */
    const T sin(const Vector2& v) const;

	/**
	 * Angle between this vector and v
	 * @param v the other vector
	 * @return the inner angle of the 2 vector (the lesser)
     */
    const T angle(const Vector2& v) const;

	/**
	 * Project vector to an other
	 * @param axis vector to project
	 * @param projected vector to the axis
	 */
	const Vector2 project(const Vector2& axis) const;

	/**
	 * Mirror to a vector
	 * @param axis vector to mirror
	 * @return projected vector to the axis
	 */
	const Vector2 mirror(const Vector2& axis) const;

	/**
	 * Constant unit vectors
	 */
	const static Vector2 UnitI;
	const static Vector2 UnitJ;
};

/**
 * Typdefs
 */
typedef Vector2<float> Vector2f;
typedef Vector2<double> Vector2d;

/***********************************
 * Implementation
 ***********************************/

/**
 * Default constructor
 */
template<typename T>
Vector2<T>::Vector2() :
		x(0), y(0)
{
}

/**
 * Constructor with coordinates
 * @param x the first coordinate of the vector
 * @param y the second coordinate of the vector
 */
template<typename T>
Vector2<T>::Vector2(T x, T y) :
		x(x), y(y)
{
}

/**
 * Contructor with other type vector
 * @param v other vector
 */
template<typename T>
template<typename U>
Vector2<T>::Vector2(const Vector2<U>& v) :
		x(v.x), y(v.y)
{
}

/**
 * Assignment operator
 * @param other vector
 */
template<typename T>
template<typename U>
inline Vector2<T>& Vector2<T>::operator =(const Vector2<U>& v)
{
	set(v.x, v.y);
}

/**
 * Equal operator
 * @return true if all coordinates are equal
 */
template<typename T>
template<typename U>
inline bool Vector2<T>::operator ==(const Vector2<U>& v) const
{
    return x == v.x && y == v.y;
}

/**
 * Set coordinates
 * @param x the first coordinate of the vector
 * @param y the second coordinate of the vector
 */
template<typename T>
inline void Vector2<T>::set(T x, T y)
{
	this->x = x;
	this->y = y;
}

/**
 * Get as T array
 * @return pointer to the "coordinates array"
 */
template<typename T>
inline const T* Vector2<T>::array() const
{
	return (T*) &this->x;
}

/**
 * Access operator
 * @param i index of the coordinate (0 -> x, 1 -> y)
 */
template<typename T>
T& Vector2<T>::operator[](const size_t& i)
{
	switch (i)
	{
	case 0:
		return x;
	case 1:
		return y;
	default:
		static T nullref;
		return nullref;
	}
}

/**
 * Constant access operator
 * @param i index of the coordinate (0 -> x, 1 -> y)
 */
template<typename T>
const T& Vector2<T>::operator[](const size_t& i) const
{
	switch (i)
	{
	case 0:
		return x;
	case 1:
		return y;
	default:
		static T nullref;
		return nullref;
	}
}

/**
 * Add two vector
 * @param v vector to add
 * @return (self + v) vector
 */
template<typename T>
inline const Vector2<T> Vector2<T>::add(const Vector2<T>& v) const
{
	return Vector2<T>(x + v.x, y + v.y);
}

/**
 * Subtract two vector
 * @param v vector to subtract
 * @return (self - v) vector
 */
template<typename T>
inline const Vector2<T> Vector2<T>::sub(const Vector2<T>& v) const
{
	return Vector2<T>(x - v.x, y - v.y);
}

/**
 * Multply with scalar
 * @param l scalar multiplier
 * @return (l*self) vector
 */
template<typename T>
template<typename U>
inline const Vector2<T> Vector2<T>::mul(const U& l) const
{
	return Vector2<T>(l * x, l * y);
}

/**
 * Divide by scalar
 * @param l scalar divider
 * @return (self/l) vector
 */
template<typename T>
template<typename U>
inline const Vector2<T> Vector2<T>::div(const U& l) const
{
	return Vector2<T>(x / l, y / l);
}

/**
 * Get length of the vector
 * @return length of the vector
 */
template<typename T>
inline const T Vector2<T>::length() const
{
	return sqrtl(length2());
}

/**
 * Get length square of the vector
 * @return the square of the vector's length
 */
template<typename T>
inline const T Vector2<T>::length2() const
{
	return x * x + y * y;
}

/**
 * Normalize the vector
 * @return normalized vector
 */
template<typename T>
inline const Vector2<T> Vector2<T>::normalized() const
{
    return div(length());
}

/**
 * Dot product calculation
 * @param v second vector in the product
 * @return dot product of the vectors
 */
template<typename T>
inline const T Vector2<T>::dot(const Vector2<T>& v) const
{
	return x * v.x + y * v.y;
}

/**
 * Cross product of the 2 vector
 * In 2D it's only a number (the third coordinate, becouse the first 2 is 0)!
 * @param v second vector in the product
 * @return cross product of the vectors
 */
template<typename T>
inline const T Vector2<T>::cross(const Vector2<T>& v) const
{
	return x * v.y - y * v.x;
}

/**
 * Consine of the inner angle of the 2 vector
 * @param v second vector
 * @return cosine of the inner angle
 */
template<typename T>
const T Vector2<T>::cos(const Vector2<T>& v) const
{
    return dot(v) / (length() * v.length());
}

/**
 * Sine of the inner angle of the 2 vector
 * @param v second vector
 * @return sine of the inner angle
 */
template<typename T>
const T Vector2<T>::sin(const Vector2<T>& v) const
{
    return cross(v) / (length() * v.length());
}

/**
 * Angle between this vector and v
 * @param v the other vector
 * @return the inner angle of the 2 vector (the lesser)
 */
template<typename T>
const T Vector2<T>::angle(const Vector2<T>& v) const
{
    return acosl(cos(v));
}

/**
 * Project vector to an other
 * @param axis vector to project
 * @param projected vector to the axis
 */
template<typename T>
const Vector2<T> Vector2<T>::project(const Vector2<T>& axis) const
{
	return axis.mul(dot(axis.normalized()));
}

/**
 * Mirror to a vector
 * @param axis vector to mirror
 * @return projected vector to the axis
 */
template<typename T>
const Vector2<T> Vector2<T>::mirror(const Vector2<T>& axis) const
{
	return add(project(axis).sub(*this).mul(2));
}

/***********************************
 * Static constants
 ***********************************/

/**
 * Unit in 'x' direction
 */
template<typename T>
const Vector2<T> Vector2<T>::UnitI(1, 0);

/**
 * Unit in 'y' direction
 */
template<typename T>
const Vector2<T> Vector2<T>::UnitJ(0, 1);

/***********************************
 * Operators
 ***********************************/

/**
 * Output operator to ostream
 * @param out ostream ref
 * @param v vector to write
 */
template<typename T>
inline std::ostream& operator <<(std::ostream& out, const Vector2<T>& v)
{
	out << '(' << v.x << ';' << v.y << ')';
	return out;
}

/**
 * + operator
 * @param v first vector
 * @param w second vector
 * @return v + w vector
 */
template<typename T>
inline const Vector2<T> operator +(const Vector2<T>& v, const Vector2<T>& w)
{
	return v.add(w);
}

/**
 * += operator
 * @param v first vector + modified
 * @param w second vector
 * @return v reference
 */
template<typename T>
inline Vector2<T>& operator +=(Vector2<T>& v, const Vector2<T>& w)
{
	v = v.add(w);
	return v;
}

/**
 * - operator
 * @param v first vector
 * @param w second vector
 * @return v - w vector
 */
template<typename T>
inline const Vector2<T> operator -(const Vector2<T>& v, const Vector2<T>& w)
{
	return v.sub(w);
}

/**
 * -= operator
 * @param v first vector + modified
 * @param w second vector
 * @return v reference
 */
template<typename T>
inline Vector2<T>& operator -=(Vector2<T>& v, const Vector2<T>& w)
{
	v = v.sub(w);
	return v;
}

/**
 * * operator with left scalar
 * @param l scalar
 * @param v vector
 * @return l*v vector
 */
template<typename T, typename U>
inline const Vector2<T> operator *(const U& l, const Vector2<T>& v)
{
	return v.mul(l);
}

/**
 * * operator with right scalar
 * @param v vector
 * @param l scalar
 * @return v*l vector
 */
template<typename T, typename U>
inline const Vector2<T> operator *(const Vector2<T>& v, const U& l)
{
	return v.mul(l);
}

/**
 * *= operator with scalar
 * @param v vector
 * @param l scalar
 * @return reference to v
 */
template<typename T, typename U>
inline Vector2<T>& operator *=(Vector2<T>& v, const U& l)
{
	v = v.mul(l);
	return v;
}

/**
 * / operator with scalar
 * @param v vector
 * @param l scalar
 * @return v/l vector
 */
template<typename T, typename U>
inline const Vector2<T> operator /(const Vector2<T>& v, const U& l)
{
	return v.div(l);
}

/**
 * /= operator with scalar
 * @param v vector
 * @param l scalar
 * @return v reference
 */
template<typename T, typename U>
inline Vector2<T>& operator /=(Vector2<T>& v, const U& l)
{
	v = v.div(l);
	return v;
}

/**
 * * operator with vectors (dot product)
 * @param v first vector
 * @param w second vector
 * @return dot product of the two vectors
 */
template<typename T>
inline const T operator *(const Vector2<T>& v, const Vector2<T>& w)
{
	return v.dot(w);
}

} /* namespace math */
#endif /* VECTOR2_HPP_ */
