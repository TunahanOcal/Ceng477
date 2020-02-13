#include "Light.h"
#include <math.h>

/* Constructor. Implemented for you. */
PointLight::PointLight(const Vector3f & position, const Vector3f & intensity)
    : position(position), intensity(intensity)
{
}

// Compute the contribution of light at point p using the
// inverse square law formula
Vector3f PointLight::computeLightContribution(const Vector3f& p)
{
    Vector3f direction_vector = {p.x-position.x,p.y-position.y,p.z-position.z};
    float distance = sqrt(pow(direction_vector.x,2)+ pow(direction_vector.y,2)+pow(direction_vector.z,2));
    float light_contrubition_x = (intensity.x/(pow(distance,2)));
    float light_contrubition_y = (intensity.y/(pow(distance,2)));
    float light_contrubition_z = (intensity.z/(pow(distance,2)));

    return {light_contrubition_x,light_contrubition_y,light_contrubition_z};
}
