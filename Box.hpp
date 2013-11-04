#ifndef BOX_HPP
#define BOX_HPP

// Std
#include <iostream>
// Math
#include <Vector2.hpp>
#include <Vector3.hpp>

namespace math {

/*********************************
 * Definition
 *********************************/

template<typename V>
class Box
{
public:
    /**
     * Default constuctor
     */
    Box();

    /**
     * @brief Box contructor with start vectors
     * @param min the min coordinates
     * @param max the max coordinates
     */
    Box(const V& min, const V& max);

    /**
     * @brief Get minimum
     * @return the min vector
     */
    const V min() const;

    /**
     * @brief Get maximum
     * @return the max vector
     */
    const V max() const;

    /**
     * @brief Get position
     * @return the min value
     */
    const V pos() const;

    /**
     * @brief Get size
     * @return max - min
     */
    const V size() const;

    /**
     * @brief Get center
     * @return (max + min) / 2
     */
    const V center() const;

    /**
     * @brief Set minimum
     * @param min the new min value
     */
    void setMin(const V& min);

    /**
     * @brief Set maximum
     * @param max the new max value
     */
    void setMax(const V& max);

    /**
     * @brief Set position. It keeps size!
     * @param pos the new pos
     */
    void setPos(const V& pos);

    /**
     * @brief Set size.
     * @param size
     */
    void setSize(const V& size);

    /**
     * @brief Set center. It keeps size!
     * @param center the new center (center = (max + min) / 2)
     */
    void setCenter(const V& center);

    /**
     * @brief Move the Box by (x,y)
     * @param x move by x
     * @param y move by y
     */
    void move(V x, V y);

    /**
     * @brief Move the Box by diff
     * @param diff move by
     */
    void move(const V& diff);

    /**
     * @brief Check if contains the point 'point'
     * @param point the point to check
     * @return true of it's in, false if not
     * @note: It's spacialized for: float, double, Vector2 and Vector3
     * @note: for any other type it's must be specialized!!!
     */
    bool contains(const V& point);
private:
    V mMin, mMax;
};

/**
 * Typdefs
 */
typedef Box<float> Intervalf;
typedef Box<double> Intervald;
typedef Box<Vector2f> Areaf;
typedef Box<Vector2d> Aread;
typedef Box<Vector3f> Volumef;
typedef Box<Vector3d> Volumed;

/***********************************
 * Implementation
 ***********************************/

/**
 * Default constuctor
 */
template<typename V>
Box<V>::Box()
{
}

/**
 * @brief Box contructor with start vectors
 * @param min the min coordinates
 * @param max the max coordinates
 */
template<typename V>
Box<V>::Box(const V &min, const V &max)
    : mMin(min), mMax(max)
{
}

/**
 * @brief Get minimum
 * @return the min vector
 */
template<typename V>
const V Box<V>::min() const
{
    return mMin;
}

/**
 * @brief Get maximum
 * @return the max vector
 */
template<typename V>
const V Box<V>::max() const
{
    return mMax;
}

/**
 * @brief Get position
 * @return the min value
 */
template<typename V>
const V Box<V>::pos() const
{
    return mMin;
}

/**
 * @brief Get size
 * @return max - min
 */
template<typename V>
const V Box<V>::size() const
{
    return max() - min();
}

/**
 * @brief Get center
 * @return (max + min) / 2
 */
template<typename V>
const V Box<V>::center() const
{
    return (min() + max()) / 2;
}

/**
 * @brief Set minimum
 * @param min the new min value
 */
template<typename V>
void Box<V>::setMin(const V &min)
{
    mMin = min;
}

/**
 * @brief Set maximum
 * @param max the new max value
 */
template<typename V>
void Box<V>::setMax(const V& max)
{
    mMax = max;
}

/**
 * @brief Set position. It keeps size!
 * @param pos the new pos
 */
template<typename V>
void Box<V>::setPos(const V& pos)
{
    V size = size();
    setMin(pos);
    setSize(size);
}

/**
 * @brief Set size.
 * @param size
 */
template<typename V>
void Box<V>::setSize(const V& size)
{
    setMax(mMin + size);
}

/**
 * @brief Set center. It keeps size!
 * @param center the new center (center = (max + min) / 2)
 */
template<typename V>
void Box<V>::setCenter(const V& center)
{
    V halfSize = size() / 2;
    setMin(center - halfSize);
    setMax(center + halfSize);
}

/**
 * @brief Move the Box by (x,y)
 * @param x move by x
 * @param y move by y
 */
template<typename V>
void Box<V>::move(V x, V y)
{
    move(V(x, y));
}

/**
 * @brief Move the Box by diff
 * @param diff move by
 */
template<typename V>
void Box<V>::move(const V& diff)
{
    setMin(min() + diff);
    setMax(max() + diff);
}

/**
 * @brief Check if contains the point 'point'
 * @param point the point to check
 * @return true of it's in, false if not
 * @note: for any other type it's must be specialized!!!
 */
template<typename V>
bool Box<V>::contains(const V& point)
{
    return point >= min() && point <= max();
}

/***********************************
 * Operators
 ***********************************/

/**
 * Output operator to ostream
 * @param out ostream ref
 * @param box the Box to write
 */
template<typename V>
std::ostream& operator <<(std::ostream& out, const Box<V>& box)
{
    out << "[" << box.min() << ";" << box.max() << "]";
    return out;
}

} // Namespace math

#endif // BOX_HPP
