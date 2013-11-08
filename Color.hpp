#ifndef COLOR_HPP
#define COLOR_HPP

// STD
#include <iostream>
#include <cmath>

namespace graphics {

/*********************************
 * Definition
 *********************************/

template<typename T,int MAX>
class Color
{
public:
    /**
     * The components as RGBA
     */
    T r, g, b, a;

    constexpr static const T Min = 0;
    constexpr static const T Max = MAX;

    /**
     * @brief Color default constructor
     */
    Color();

    /**
     * @brief Color contructor with 4 component
     * @param r red component
     * @param g green component
     * @param b blue component
     * @param a alpha component, default is max
     */
    Color(T r, T g, T b, T a = Max);

    /**
     * @brief size of color type
     */
    static const size_t size = 4;

    /**
     * @brief set the components. Alpha remain the same
     * @param r red component
     * @param g green component
     * @param b blue component
     */
    inline void set(T r, T g, T b);

    /**
     * @brief set the components
     * @param r red component
     * @param g green component
     * @param b blue component
     * @param a alpha component
     */
    inline void set(T r, T g, T b, T a);

    /**
     * @brief Get color as const array
     * @return the pointer to the array
     */
    inline const T* array() const;

    /**
     * @brief operator [] to reference
     * @param i index of component (0 -> r, 1 -> g, 2 -> b, 3 -> a)
     * @return reference of the component
     */
    T& operator[](const size_t& i);

    /**
     * @brief operator [] to reference
     * @param i index of component (0 -> r, 1 -> g, 2 -> b, 3 -> a)
     * @return const reference of the component
     */
    const T& operator[](const size_t& i) const;

    /**
     * @brief fromHSV
     * @param h hue of the color (0-360°)
     * @param s saturation of the color (0-1)
     * @param v value of the color (0-1)
     * @param a aplha
     * @return the rgba representation
     */
    static Color fromHSV(float H, float S, float V, T a = Max);

    /**
     * @brief Add the color componentwise, alpha become avg
     * @param col the other color
     * @return the added colors
     */
    Color add(const Color& col) const;

    /**
     * @brief Average componentwise
     * @param col the other color
     * @return the avg color
     */
    Color mix(const Color& col) const;

    /**
     * Constant colors
     */
    const static Color Black;
    const static Color Red;
    const static Color Green;
    const static Color Blue;
    const static Color Cyan;
    const static Color Magenta;
    const static Color Yellow;
    const static Color White;
};

/**
 * Typedefs
 */
typedef Color<unsigned char, 255> Colorb;
typedef Color<float, 1> Colorf;

/***********************************
 * Implementation
 ***********************************/

/**
 * @brief Color default constructor
 */
template<typename T,int MAX>
Color<T,MAX>::Color() : r(0), g(0), b(0), a(Max)
{}

/**
 * @brief Color contructor with 4 component
 * @param r red component
 * @param g green component
 * @param b blue component
 * @param a alpha component, default is max
 */
template<typename T,int MAX>
Color<T,MAX>::Color(T r, T g, T b, T a) : r(r), g(g), b(b), a(a)
{}

/**
 * @brief set the components. Alpha remain the same
 * @param r red component
 * @param g green component
 * @param b blue component
 */
template<typename T,int MAX>
inline void Color<T,MAX>::set(T r, T g, T b)
{
    this->r = r;
    this->g = g;
    this->b = b;
}

/**
 * @brief set the components
 * @param r red component
 * @param g green component
 * @param b blue component
 * @param a alpha component
 */
template<typename T,int MAX>
inline void Color<T,MAX>::set(T r, T g, T b, T a)
{
    this->r = r;
    this->g = g;
    this->b = b;
    this->a = a;
}

/**
 * @brief Get color as const array
 * @return the pointer to the array
 */
template<typename T,int MAX>
inline const T* Color<T,MAX>::array() const
{
    return (T*)&this->r;
}

/**
 * @brief operator [] to reference
 * @param i index of component (0 -> r, 1 -> g, 2 -> b, 3 -> a)
 * @return reference of the component
 */
template<typename T,int MAX>
T& Color<T,MAX>::operator[](const size_t& i)
{
    switch (i) {
    case 0:
        return r;
    case 1:
        return g;
    case 2:
        return b;
    case 3:
        return a;
    default:
        static T nullref;
        return nullref;
    }
}

/**
 * @brief operator [] to reference
 * @param i index of component (0 -> r, 1 -> g, 2 -> b, 3 -> a)
 * @return const reference of the component
 */
template<typename T,int MAX>
const T& Color<T,MAX>::operator[](const size_t& i) const
{
    switch (i) {
    case 0:
        return r;
    case 1:
        return g;
    case 2:
        return b;
    case 3:
        return a;
    default:
        static T nullref;
        return nullref;
    }
}

/**
 * @brief fromHSV
 * @param h hue of the color (0-360°)
 * @param s saturation of the color (0-1)
 * @param v value of the color (0-1)
 * @param a aplha
 * @return the rgba representation
 */
template<typename T,int MAX>
Color<T,MAX> Color<T,MAX>::fromHSV(float H, float S, float V, T a)
{
    // If gray
    if(S == 0)
        return Color(V * Max,V * Max,V * Max,a);
    // Tmp values
    H /= 60;
    int i = floor(H);
    float f = H - i;
    float p = V * (1 - S);
    float q = V * (1 - S * f);
    float t = V * (1 - S * (1 - f));
    // Switch by sixths
    switch (i) {
    case 0:
        return Color<T,MAX>(V * Max, t * Max, p * Max, a);
    case 1:
        return Color<T,MAX>(q * Max, V * Max, p * Max, a);
    case 2:
        return Color<T,MAX>(p * Max, V * Max, t * Max, a);
    case 3:
        return Color<T,MAX>(p * Max, q * Max, V * Max, a);
    case 4:
        return Color<T,MAX>(t * Max, p * Max, V * Max, a);
    case 5:
    default:
        return Color<T,MAX>(V * Max, p * Max, q * Max, a);
    }
}

/**
 * @brief Add the color componentwise, alpha become avg
 * @param col the other color
 * @return the added colors
 */
template<typename T,int MAX>
Color<T,MAX> Color<T,MAX>::add(const Color<T,MAX>& col) const
{
    return Color<T,MAX>(r+col.r,g+col.g,b+col.b,((float)a+(float)col.a)/2);
}

/**
 * @brief Average componentwise
 * @param col the other color
 * @return the avg color
 */
template<typename T,int MAX>
Color<T,MAX> Color<T,MAX>::mix(const Color<T,MAX>& col) const
{
    Color<T,MAX> mixed;
    for(size_t i = 0; i < Color<T,MAX>::size; ++i)
        mixed[i] = ((float)(*this)[i] + (float)col[i]) / 2;
    return mixed;
}

/***********************************
 * Static constants
 ***********************************/

/**
 * Const color black
 */
template<typename T,int MAX>
const Color<T,MAX> Color<T,MAX>::Black(0,0,0);

/**
 * Const color red
 */
template<typename T,int MAX>
const Color<T,MAX> Color<T,MAX>::Red(MAX,0,0);

/**
 * Const color green
 */
template<typename T,int MAX>
const Color<T,MAX> Color<T,MAX>::Green(0,MAX,0);

/**
 * Const color blue
 */
template<typename T,int MAX>
const Color<T,MAX> Color<T,MAX>::Blue(0,0,MAX);

/**
 * Const color cyan
 */
template<typename T,int MAX>
const Color<T,MAX> Color<T,MAX>::Cyan(0,MAX,MAX);

/**
 * Const color magenta
 */
template<typename T,int MAX>
const Color<T,MAX> Color<T,MAX>::Magenta(MAX,0,MAX);

/**
 * Const color yellow
 */
template<typename T,int MAX>
const Color<T,MAX> Color<T,MAX>::Yellow(MAX,MAX,0);

/**
 * Const color white
 */
template<typename T,int MAX>
const Color<T,MAX> Color<T,MAX>::White(MAX,MAX,MAX);

/***********************************
 * Operators
 ***********************************/

template<typename T,int MAX>
std::ostream& operator <<(std::ostream& out, const Color<T,MAX>& col)
{
    out << '{' << (float)col.r << ';' << (float)col.g << ';' << (float)col.b << ';' << (float)col.a << '}';
    return out;
}

} // Namespace graphics

#endif // COLOR_HPP
