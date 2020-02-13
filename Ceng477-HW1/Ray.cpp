#include "Ray.h"

Ray::Ray()
{
}

Ray::Ray(const Vector3f& origin, const Vector3f& direction)
    : origin(origin), direction(direction)
{
}

/* Takes a parameter t and returns the point accoring to t. t is the parametric variable in the ray equation o+t*d.*/
Vector3f Ray::getPoint(float t) const 
{
    float x,y,z;
    x = origin.x+t*direction.x;
    y = origin.y+t*direction.y;
    z = origin.z+t*direction.z;
    Vector3f vector = {x,y,z};
    return vector;
}

/* Takes a point p and returns the parameter t according to p such that p = o+t*d. */
float Ray::gett(const Vector3f & p) const
{
    float a;
    a=((p.x-origin.x)/direction.x);
    if(a<0)
        a=a*(-1);

    return a;
}

