#include "Camera.h"
#include <string.h>
#include <string>

Camera::Camera(int id,                      // Id of the camera
               const char* imageName,       // Name of the output PPM file 
               const Vector3f& pos,         // Camera position
               const Vector3f& gaze,        // Camera gaze direction
               const Vector3f& up,          // Camera up direction
               const ImagePlane& imgPlane)  // Image plane parameters
{

    strcpy(this->imageName,imageName);
    this->id=id;
    this->imgPlane=imgPlane;
    this->gaze=gaze;
    this->pos=pos;
    this->up=up;
}

/* Takes coordinate of an image pixel as row and col, and
 * returns the ray going through that pixel. 
 */
Ray Camera::getPrimaryRay(int col, int row) const
{
    Vector3f w = {gaze.x*(-1),gaze.y*(-1),gaze.z*(-1)};

    float s_u = (col+0.5)*((imgPlane.right-imgPlane.left)/imgPlane.nx);
    float s_v = (row+0.5)*((imgPlane.top-imgPlane.bottom)/imgPlane.ny);

    /*calculation of the m vector */
    float mx= pos.x + gaze.x*imgPlane.distance;
    float my = pos.y + gaze.y*imgPlane.distance;
    float mz = pos.z + gaze.z*imgPlane.distance;
    Vector3f m = {mx,my,mz};

    /*calculation of the v vector */
    float ux = (up.y*w.z)-(up.z*w.y);
    float uy = (-1)*((up.x*w.z)-(up.z*w.x));
    float uz = (up.x*w.y)-(up.y*w.x);
    Vector3f u = {ux,uy,uz};

    float qx = m.x + imgPlane.left*u.x + imgPlane.top*up.x;
    float qy = m.y + imgPlane.left*u.y + imgPlane.top*up.y;
    float qz = m.z + imgPlane.left*u.z + imgPlane.top*up.z;

    float sx = qx + (s_u*u.x) - s_v*up.x;
    float sy = qy + (s_u*u.y) - s_v*up.y;
    float sz = qz + (s_u*u.z) - s_v*up.z;

    Ray r;
    r.origin = {pos.x,pos.y,pos.z};
    r.direction = {sx-pos.x,sy-pos.y,sz-pos.z};

    return r;

}

