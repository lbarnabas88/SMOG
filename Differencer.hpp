#ifndef DIFFERENCER_HPP
#define DIFFERENCER_HPP

template <typename T>
class Differencer
{
public:
    Differencer(const T &startValue) : mValue(startValue) {}
    T calcDiff(T value)
    {
        T diff = value - mValue;
        mValue = value;
        return diff;
    }
private:
    T mValue;
};

#endif // DIFFERENCER_HPP
