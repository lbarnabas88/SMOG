#ifndef LASPOINT_HPP
#define LASPOINT_HPP

struct _LasPoint
{
    double x,y,z;
    unsigned short returns;
    unsigned short returnId;
};

#endif // LASPOINT_HPP
