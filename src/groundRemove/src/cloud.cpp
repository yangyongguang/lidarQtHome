#include "cloud.h"

// explicit point(const point & pt)
// {
//     _point(pt.x(), pt.y(), pt.z());
//     _intensity = pt.i();
// }

void point::operator=(const point & other)
{
    _point = other.AsEigenVector();
    _intensity = other.i();
}

void point::operator=(const Eigen::Vector3f& other) 
{
    _point = other;
}

bool point::operator==(const point & other) const
{
     return x() == other.x() && y() == other.y() &&
         z() == other.z() && i() == other.i();
}

