#ifndef VECTOR3_HPP
#define VECTOR3_HPP

#include "Vector2.hpp"

namespace math
{

/*********************************
 * Definition
 *********************************/

template<typename T>
class Vector3
{
public:
    /**
     * The coordinates
     */
    T x, y, z;

    /**
     * Default constructor
     */
    Vector3();

    /**
     * Constructor with coordinates
     * @param x the first coordinate of the vector
     * @param y the second coordinate of the vector
     * @param z the third coordinate of the vector
     */
    Vector3(T x, T y, T z);

    /**
     * Contructor with other type vector
     * @param v other vector
     */
    template<typename U>
    Vector3(const Vector3<U>& v);

    /**
     * Contructor with other type vector
     * @param v other vector
     */
    template<typename U>
    Vector3(const Vector2<U>& v);

    /**
     * Assignment operator
     * @param other vector
     */
    template<typename U>
    inline Vector3& operator =(const Vector3<U>& v);

    /**
     * Equal operator
     * @return true if all coordinates are equal
     */
    template<typename U>
    inline bool operator ==(const Vector3<U>& v) const;

    /**
     * Get size of the vector
     * @return unsigned long size of this vector
     */
    static const size_t size = 3;

    /**
     * Set coordinates
     * @param x the first coordinate of the vector
     * @param y the second coordinate of the vector
     * @param y the third coordinate of the vector
     */
    inline void set(T x, T y, T z);

    /**
     * Get as T array
     * @return pointer to the "coordinates array"
     */
    inline const T* array() const;

    /**
     * Access operator
     * @param i index of the coordinate (0 -> x, 1 -> y, 2 -> z)
     */
    T& operator[](const size_t& i);

    /**
     * Constant access operator
     * @param i index of the coordinate (0 -> x, 1 -> y, 2 -> z)
     */
    const T& operator[](const size_t& i) const;

    /**
     * Add two vector
     * @param v vector to add
     * @return (self + v) vector
     */
    inline const Vector3 add(const Vector3& v) const;

    /**
     * Subtract two vector
     * @param v vector to subtract
     * @return (self - v) vector
     */
    inline const Vector3 sub(const Vector3& v) const;

    /**
     * Multply with scalar
     * @param l scalar multiplier
     * @return (l*self) vector
     */
    template<typename U>
    inline const Vector3 mul(const U& l) const;

    /**
     * Divide by scalar
     * @param l scalar divider
     * @return (self/l) vector
     */
    template<typename U>
    inline const Vector3 div(const U& l) const;

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
    inline const Vector3 normalized() const;

    /**
     * Normalize the vector
     * @return this vector normalized
     */
    inline Vector3& normalize();

    /**
     * Dot product calculation
     * @param v second vector in the product
     * @return dot product of the vectors
     */
    inline const T dot(const Vector3& v) const;

    /**
     * Cross product of the 2 template<typename T>vector
     * In 3D it's a vector
     * @param v second vector in the product
     * @return cross product of the vectors
     */
    inline const Vector3 cross(const Vector3& v) const;

    /**
     * Consine of the inner angle of the 2 vector
     * @param v second vector
     * @return cosine of the inner angle
     */
    const T cos(const Vector3& v) const;

    /**
     * Sine of the inner angle of the 2 vector
     * @param v second vector
     * @return sine of the inner angle
     */
    const T sin(const Vector3& v) const;

    /**
     * Angle between this vector and v
     * @param v the other vector
     * @return the inner angle of the 2 vector (the lesser)
     */
    const T angle(const Vector3& v) const;

    /**
     * Project vector to an other
     * @param axis vector to project
     * @param projected vector to the axis
     */
    const Vector3 project(const Vector3& axis) const;

    /**
     * Mirror to a vector
     * @param axis vector to mirror
     * @return projected vector to the axis
     */
    const Vector3 mirror(const Vector3& axis) const;

    /**
     * Convert to 2D Vector2
     * @return 2d vector from this 3d vector
     */
    template<typename U>
    const Vector2<U> toVector2() const;

    /**
     * Convert to 3D Vector3
     * @return 3d vector from this 3d vector with type U
     */
    template<typename U>
    const Vector3<U> toVector3() const;

    /**
     * Constant unit vectors
     */
    const static Vector3 UnitI;
    const static Vector3 UnitJ;
    const static Vector3 UnitK;
};

/**
 * Typdefs
 */
typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

/***********************************
 * Implementation
 ***********************************/

/**
 * Default constructor
 */
template<typename T>
Vector3<T>::Vector3() :
    x(0), y(0), z(0)
{
}

/**
 * Constructor with coordinates
 * @param x the first coordinate of the vector
 * @param y the second coordinate of the vector
 * @param z the third coordinate of the vector
 */
template<typename T>
Vector3<T>::Vector3(T x, T y, T z) :
    x(x), y(y), z(z)
{
}

/**
 * Contructor with other type vector
 * @param v other vector
 */
template<typename T>
template<typename U>
Vector3<T>::Vector3(const Vector3<U>& v) :
    x(v.x), y(v.y), z(v.z)
{
}

/**
 * Contructor with other type 2d vector
 * @param v other vector
 */
template<typename T>
template<typename U>
Vector3<T>::Vector3(const Vector2<U>& v) :
    x(v.x), y(v.y), z(0)
{
}

/**
 * Assignment operator
 * @param other vector
 */
template<typename T>
template<typename U>
inline Vector3<T>& Vector3<T>::operator =(const Vector3<U>& v)
{
    set(v.x, v.y, v.z);
}

/**
 * Equal operator
 * @return true if all coordinates are equal
 */
template<typename T>
template<typename U>
inline bool Vector3<T>::operator ==(const Vector3<U>& v) const
{
    return x == v.x && y == v.y && z == v.z;
}

/**
 * Set coordinates
 * @param x the first coordinate of the vector
 * @param y the second coordinate of the vector
 * @param z the third coordinate of the vector
 */
template<typename T>
inline void Vector3<T>::set(T x, T y, T z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

/**
 * Get as T array
 * @return pointer to the "coordinates array"
 */
template<typename T>
inline const T* Vector3<T>::array() const
{
    return (T*) &this->x;
}

/**
 * Access operator
 * @param i index of the coordinate (0 -> x, 1 -> y, 2 -> z)
 */
template<typename T>
T& Vector3<T>::operator[](const size_t& i)
{
    switch (i)
    {
    case 0:
        return x;
    case 1:
        return y;
    case 2:
        return z;
    default:
        static T nullref;
        return nullref;
    }
}

/**
 * Constant access operator
 * @param i index of the coordinate (0 -> x, 1 -> y, 2 -> z)
 */
template<typename T>
const T& Vector3<T>::operator[](const size_t& i) const
{
    switch (i)
    {
    case 0:
        return x;
    case 1:
        return y;
    case 2:
        return z;
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
inline const Vector3<T> Vector3<T>::add(const Vector3<T>& v) const
{
    return Vector3<T>(x + v.x, y + v.y, z + v.z);
}

/**
 * Subtract two vector
 * @param v vector to subtract
 * @return (self - v) vector
 */
template<typename T>
inline const Vector3<T> Vector3<T>::sub(const Vector3<T>& v) const
{
    return Vector3<T>(x - v.x, y - v.y, z - v.z);
}

/**
 * Multply with scalar
 * @param l scalar multiplier
 * @return (l*self) vector
 */
template<typename T>
template<typename U>
inline const Vector3<T> Vector3<T>::mul(const U& l) const
{
    return Vector3<T>(l * x, l * y, l * z);
}

/**
 * Divide by scalar
 * @param l scalar divider
 * @return (self/l) vector
 */
template<typename T>
template<typename U>
inline const Vector3<T> Vector3<T>::div(const U& l) const
{
    return Vector3<T>(x / l, y / l, z / l);
}

/**
 * Get length of the vector
 * @return length of the vector
 */
template<typename T>
inline const T Vector3<T>::length() const
{
    return sqrtl(length2());
}

/**
 * Get length square of the vector
 * @return the square of the vector's length
 */
template<typename T>
inline const T Vector3<T>::length2() const
{
    return x * x + y * y + z * z;
}

/**
 * Normalize the vector
 * @return normalized vector
 */
template<typename T>
inline const Vector3<T> Vector3<T>::normalized() const
{
    return div(length());
}

/**
 * Normalize the vector
 * @return this vector normalized
 */
template<typename T>
inline Vector3<T>& Vector3<T>::normalize()
{
    (*this) = normalized();
    return *this;
}

/**
 * Dot product calculation
 * @param v second vector in the product
 * @return dot product of the vectors
 */
template<typename T>
inline const T Vector3<T>::dot(const Vector3<T>& v) const
{
    return x * v.x + y * v.y + z * v.z;
}

/**
 * Cross product of the 2 vector
 * In 2D it's only a number (the third coordinate, becouse the first 2 is 0)!
 * @param v second vector in the product
 * @return cross product of the vectors
 */
template<typename T>
inline const Vector3<T> Vector3<T>::cross(const Vector3<T>& v) const
{
    return Vector3(y * v.z - z * v.y,z * v.x - x * v.z, x * v.y - y * v.x);
}

/**
 * Consine of the inner angle of the 2 vector
 * @param v second vector
 * @return cosine of the inner angle
 */
template<typename T>
const T Vector3<T>::cos(const Vector3<T>& v) const
{
    return dot(v) / (length() * v.length());
}

/**
 * Sine of the inner angle of the 2 vector
 * @param v second vector
 * @return sine of the inner angle
 */
template<typename T>
const T Vector3<T>::sin(const Vector3<T>& v) const
{
    return cross(v).length() / (length() * v.length());
}

/**
 * Angle between this vector and v
 * @param v the other vector
 * @return the inner angle of the 2 vector (the lesser)
 */
template<typename T>
const T Vector3<T>::angle(const Vector3<T>& v) const
{
    return acosl(cos(v));
}

/**
 * Project vector to an other
 * @param axis vector to project
 * @param projected vector to the axis
 */
template<typename T>
const Vector3<T> Vector3<T>::project(const Vector3<T>& axis) const
{
    return axis.mul(dot(axis.normalized()));
}

/**
 * Mirror to a vector
 * @param axis vector to mirror
 * @return projected vector to the axis
 */
template<typename T>
const Vector3<T> Vector3<T>::mirror(const Vector3<T>& axis) const
{
    return add(project(axis).sub(*this).mul(2));
}

/**
 * Convert to 2D Vector2
 * @return 2d vector from this 3d vector
 */
template<typename T>
template<typename U>
const Vector2<U> Vector3<T>::toVector2() const
{
    return Vector2<U>(x, y);
}

/**
 * Convert to 3D Vector3
 * @return 3d vector from this 3d vector with type U
 */
template<typename T>
template<typename U>
const Vector3<U> Vector3<T>::toVector3() const
{
    return Vector3<U>(x, y, z);
}

/***********************************
 * Static constants
 ***********************************/

/**
 * Unit in 'x' direction
 */
template<typename T>
const Vector3<T> Vector3<T>::UnitI(1, 0, 0);

/**
 * Unit in 'y' direction
 */
template<typename T>
const Vector3<T> Vector3<T>::UnitJ(0, 1, 0);

/**
 * Unit in 'z' direction
 */
template<typename T>
const Vector3<T> Vector3<T>::UnitK(0, 0, 1);

/***********************************
 * Operators
 ***********************************/

/**
 * Output operator to ostream
 * @param out ostream ref
 * @param v vector to write
 */
template<typename T>
inline std::ostream& operator <<(std::ostream& out, const Vector3<T>& v)
{
    out << '(' << v.x << ';' << v.y << ';' << v.z << ')';
    return out;
}

/**
 * + operator
 * @param v first vector
 * @param w second vector
 * @return v + w vector
 */
template<typename T>
inline const Vector3<T> operator +(const Vector3<T>& v, const Vector3<T>& w)
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
inline Vector3<T>& operator +=(Vector3<T>& v, const Vector3<T>& w)
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
inline const Vector3<T> operator -(const Vector3<T>& v, const Vector3<T>& w)
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
inline Vector3<T>& operator -=(Vector3<T>& v, const Vector3<T>& w)
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
inline const Vector3<T> operator *(const U& l, const Vector3<T>& v)
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
inline const Vector3<T> operator *(const Vector3<T>& v, const U& l)
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
inline Vector3<T>& operator *=(Vector3<T>& v, const U& l)
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
inline const Vector3<T> operator /(const Vector3<T>& v, const U& l)
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
inline Vector3<T>& operator /=(Vector3<T>& v, const U& l)
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
inline const T operator *(const Vector3<T>& v, const Vector3<T>& w)
{
    return v.dot(w);
}

/**
 * < operator with vectors
 * @param v left vector
 * @param w right Vector
 * @return true iff all coordinates of v < the corresponding coordinate of w
 */
template<typename T>
inline bool operator <(const Vector3<T>& v, const Vector3<T>& w)
{
    return v.x < w.x && v.y < w.y && v.z < w.z;
}

/**
 * > operator with vectors
 * @param v left vector
 * @param w right Vector
 * @return true iff all coordinates of v > the corresponding coordinate of w
 */
template<typename T>
inline bool operator >(const Vector3<T>& v, const Vector3<T>& w)
{
    return v.x > w.x && v.y > w.y && v.z > w.z;
}

/**
 * <= operator with vectors
 * @param v left vector
 * @param w right Vector
 * @return true iff all coordinates of v <= the corresponding coordinate of w
 */
template<typename T>
inline bool operator <=(const Vector3<T>& v, const Vector3<T>& w)
{
    return v.x <= w.x && v.y <= w.y && v.z <= w.z;
}

/**
 * >= operator with vectors
 * @param v left vector
 * @param w right Vector
 * @return true iff all coordinates of v >= the corresponding coordinate of w
 */
template<typename T>
inline bool operator >=(const Vector3<T>& v, const Vector3<T>& w)
{
    return v.x >= w.x && v.y >= w.y && v.z >= w.z;
}

} /* namespace math */

#endif // VECTOR3_HPP
