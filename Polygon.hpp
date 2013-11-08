#ifndef POLYGON_HPP
#define POLYGON_HPP

// Std
#include <vector>
// Math
#include "Vector2.hpp"

namespace math {

/**
 * Typdef for polygon
 */
template<typename T>
using Polygon = std::vector<Vector2<T> >;

/**
 * Typedef for spec polygons
 */
typedef Polygon<float> Polygonf;
typedef Polygon<double> Polygond;

/**
 * Enum of polygon direction
 */
enum PolygonDirection
{
    Unknown, CCW, CW
};

/**
 * Invert a polygon
 * @param polygon the polygon to invert
 */
template<typename T>
inline void invertPolygon(Polygon<T>& polygon)
{
    // Reverse the list
    std::reverse(polygon.begin(), polygon.end());
}

/**
 * Get inverted polygon of
 * @param polygon the polygon to get the inverted
 * @return a new polygon with inverted points
 */
template<typename T>
Polygon<T> getInvertedPolygon(const Polygon<T>& polygon)
{
    // Copy
    Polygon<T> inverted = polygon;
    // Invert
    invertPolygon(inverted);
    // Return inverted
    return inverted;
}

/**
 * Get a polygon signed area
 * @param polygon the polygon to calculate area
 * @return the signed area of the polygon
 */
template<typename T>
const T getPolygonSignedArea(const Polygon<T>& polygon)
{
    if (polygon.size() < 3)
        return 0;
    T sum = 0;
    for (auto it = polygon.begin(); it != polygon.end();)
    {
        const Vector2<T>& p = *it++;
        const Vector2<T>& q = (it == polygon.end() ? polygon.front() : *it);
        sum += (q.x - p.x) * (q.y + p.y);
    }
    return sum / 2;
}

/**
 * Get a polygon area
 * @param polygon the polygon to calculate area
 * @return the area of the polygon
 */
template<typename T>
const T getPolygonArea(const Polygon<T>& polygon)
{
    return fabs(getPolygonSignedArea(polygon));
}

/**
 * Get a polygon direction
 * @param polygon the polygon to check
 * @return the CCW or CW direction of a polygon
 */
template<typename T>
PolygonDirection getPolygonDirection(const Polygon<T>& polygon)
{
    return getPolygonSignedArea(polygon) < 0 ? CCW : CW;
}

/**
 * Set a polygon direction
 * @param polygon to set
 * @param dir the new direction to set
 */
template<typename T>
void setPolygonDirection(Polygon<T>& polygon, const PolygonDirection& dir)
{
    if (getPolygonDirection(polygon) != dir)
        invertPolygon(polygon);
}

/**
 * Edge type
 */
template<typename T>
using Edge = std::pair<Vector2<T>, Vector2<T> >;

/**
 * Check whether 2 section intersec each other
 * @param edge1 the first section
 * @param edge2 the second section
 * @return true if there is an intersection, false if not
 */
template<typename T>
inline bool isEdgesIntersect(const Edge<T>& edge1, const Edge<T>& edge2)
{
    // Bounding box check
    for (size_t i = 0; i < Vector2<T>::size; ++i)
    {
        if (std::min(edge1.first[i], edge1.second[i]) > std::max(edge2.first[i], edge2.second[i]))
            return false;
        if (std::min(edge2.first[i], edge2.second[i]) > std::max(edge1.first[i], edge1.second[i]))
            return false;
    }
    // Details check
    auto& p = edge1.first;
    auto& q = edge2.first;
    auto r = edge1.second - edge1.first;
    auto s = edge2.second - edge2.first;
    auto q_p = q - p;
    auto rxs = r.cross(s);
    auto q_pxr = q_p.cross(r);
    if (rxs == 0)
        return q_pxr == 0;
    auto q_pxs = q_p.cross(s);
    auto t = q_pxs / rxs;
    auto u = q_pxr / rxs;
    return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

/**
 * Check if a polygon is simple
 * @param polygon the polygon to check
 * @return true if it's a single polygon, false otherwise
 */
template<typename T>
bool isPolygonSimple(const Polygon<T>& polygon)
{
    // If too small it's simple
    if (polygon.size() < 3)
        return true;
    // For all edge
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        // Get edge1
        Edge<T> e1 = std::make_pair(polygon[i], polygon[i == polygon.size() - 1 ? 0 : i + 1]);
        // To all edge
        for (size_t j = i + 2; j < polygon.size() && !(i == 0 && j == polygon.size() - 1); ++j)
        {
            // Get edge2
            Edge<T> e2 = std::make_pair(polygon[j], polygon[j == polygon.size() - 1 ? 0 : j + 1]);
            // Check the edge
            if (isEdgesIntersect(e1, e2))
                return false;
        }
    }
    // There is no intersection, it's simple
    return true;
}

/**
 * Check if a point is inside a polygon
 * @param polygon the polygon
 * @param point the point to check
 * @return true if the 'point' is inside the 'polygon'
 */
template<typename T>
bool isPointInsidePolygon(const Polygon<T>& polygon, const Vector2<T>& point)
{
    // If polygon is too small
    if (polygon.size() < 3)
        return false;
    // Count intersections
    bool inside = false;
    // For all edge
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
    {
        // If edges outside in y direction
        if((point.y > polygon[i].y) != (point.y > polygon[j].y))
            if(point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)
                inside = !inside;
    }
    // If intersections is odd, it's inside
    return inside;
}

/**
 * Bezier calculator
 */
template<typename T>
class BezierCalculator2
{
public:
    /**
     * Constructor for bezier calculator
     * @param polygon reference to the processing polygon
     */
    BezierCalculator2(const Polygon<T>& polygon, T step = 0.1f);

    /**
     * Process the bezier approximation
     * @return the bezier curve
     */
    Polygon<T> process();
private:
    /**
     * Calculate Bezier constant or point at t
     * @param i the point's index
     * @param t the process number
     * @return the B bezier constant for t
     */
    T calcB(const size_t i, const T& t);

    /**
     * Calculate Bezier point for t
     * @param t the process number
     * @return point on the curve
     */
    Vector2<T> calcPoint(const T& t);

    /**
     * Calculate the
     */

    /**
     * The polygon reference
     */
    const Polygon<T>& mPolygon;

    /**
     * Step volume per point
     */
    T mStep;
};

/**
 * Constructor for bezier calculator
 * @param polygon reference to the processing polygon
 */
template<typename T>
BezierCalculator2<T>::BezierCalculator2(const Polygon<T>& polygon, T step) :
        mPolygon(polygon), mStep(step)
{
}

/**
 * Process the bezier approximation
 * @return the bezier curve
 */
template<typename T>
Polygon<T> BezierCalculator2<T>::process()
{
    // Result polygon
    Polygon<T> curve;
    // For all steps
    for (T t = 0; t < 1; t += mStep)
        curve.push_back(calcPoint(t));
    // Return result
    return curve;
}

/**
 * Calculate Bezier constant or point at t
 * @param i the point's index
 * @param t the process number
 * @return the B bezier constant for t
 */
template<typename T>
T BezierCalculator2<T>::calcB(const size_t i, const T& t)
{
    // The number at first is 1
    T Bi = 1;
    // For begining
    for (size_t j = 1; j <= i; ++j)
        Bi *= t * (mPolygon.size() - j) / j;
    // For end
    for (size_t j = i + 1; j < mPolygon.size(); ++j)
        Bi *= (1 - t);
    // Return the number
    return Bi;
}

/**
 * Calculate Bezier point for t
 * @param t the process number
 * @return point on the curve
 */
template<typename T>
Vector2<T> BezierCalculator2<T>::calcPoint(const T& t)
{
    // The bezier point
    Vector2<T> p;
    // For all point calc and sum
    for (size_t i = 0; i < mPolygon.size(); ++i)
        p += mPolygon[i] * calcB(i, t);
    // Return the point
    return p;
}

} // Namespace math

#endif // POLYGON_HPP
