
#include "Shape.h"
#include "Scene.h"
#include <cstdio>
#include <math.h>

Shape::Shape(void)
{
}

Shape::Shape(int id, int matIndex)
        : id(id), matIndex(matIndex)
{
}

Sphere::Sphere(void)
{}

/* Constructor for sphere. You will implement this. */
Sphere::Sphere(int id, int matIndex, int cIndex, float R,vector<Vector3f> *vertices)
        : Shape(id, matIndex)
{
    this->vertices=vertices;
    this->R=R;
    this->cIndex=cIndex;
}

/* Sphere-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will need. It is in defs.h file. */
ReturnVal Sphere::intersect(const Ray & ray) const
{
    bool is_intersected;
    Vector3f intersect_point;
    Vector3f center_vector = pScene->vertices[cIndex-1];
    Vector3f o_c_vector ={(ray.origin.x-center_vector.x),(ray.origin.y-center_vector.y),(ray.origin.z-center_vector.z)};
    float b = {ray.direction.x*o_c_vector.x+ray.direction.y*o_c_vector.y+ray.direction.z*o_c_vector.z};
    float o_c_dot = {o_c_vector.x*o_c_vector.x+o_c_vector.y*o_c_vector.y+o_c_vector.z*o_c_vector.z};
    float d_dot = {ray.direction.x*ray.direction.x+ray.direction.y*ray.direction.y+ray.direction.z*ray.direction.z};
    float t_x = sqrtf(powf(b,2)-d_dot*(o_c_dot-powf(R,2)));
    float disc = powf(b,2)-d_dot*(o_c_dot-powf(R,2));
    float t1,t2,t3;


    ReturnVal return_val;
    return_val.is_intersected=false;


    if(disc<pScene->intTestEps){
        return_val.is_intersected=false;
        return return_val;
    }
    t1 = (-1)*(b+t_x)/d_dot;
    t2 = (-1)*(b-t_x)/d_dot;
    t3 = min(t1,t2);

    if( t1 >= pScene->intTestEps && t2 >= pScene->intTestEps )
        t3 = t3;
    else if(t1 <pScene->intTestEps && t2 >= pScene->intTestEps)
        t3 = t2;
    else if(t2 < pScene->intTestEps && t1 >=pScene->intTestEps)
        t3 = t1;
    else
        return return_val;

    intersect_point = ray.getPoint(t3);
    Vector3f normal_first = {intersect_point.x-center_vector.x,intersect_point.y-center_vector.y,intersect_point.z-center_vector.z};
    Vector3f normal_vector = {normal_first.x/R,normal_first.y/R,normal_first.z/R};

    return_val.is_intersected = true;
    return_val.intersect_point = ray.getPoint(t3);
    return_val.t=t3;
    return_val.normal_vector=normal_vector;
    return_val.mat_ID=this->matIndex;
    return return_val;
}

Triangle::Triangle(void)
{}

/* Constructor for triangle. You will implement this. */
Triangle::Triangle(int id, int matIndex, int p1Index, int p2Index, int p3Index,vector<Vector3f> *vertices)
        : Shape(id, matIndex)
{
    this->matIndex=matIndex;
    this->p1Index=p1Index;
    this->p2Index=p2Index;
    this->p3Index=p3Index;
    this->vertices=vertices;
}

/* Triangle-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will need. It is in defs.h file. */
ReturnVal Triangle::intersect(const Ray & ray) const
{
    ReturnVal returnVal;
    float EPSILON = pScene->intTestEps;
    bool is_intersected;
    Vector3f p1=pScene->vertices[p1Index-1];
    Vector3f p2=pScene->vertices[p2Index-1];
    Vector3f p3=pScene->vertices[p3Index-1];
    float ray_direction_length = sqrtf(powf(ray.direction.x,2)+powf(ray.direction.y,2)+powf(ray.direction.z,2));
    Vector3f direction_normal = {(ray.direction.x/ray_direction_length),(ray.direction.y/ray_direction_length),(ray.direction.z/ray_direction_length)};
    // finding normal of triangle

    Vector3f p1_p3 = {p1.x-p3.x,p1.y-p3.y,p1.z-p3.z};//(p1-p3)
    Vector3f p2_p3 = {p2.x-p3.x,p2.y-p3.y,p2.z-p3.z};//(p2-p3)
    Vector3f normal_vector_first;
    //cross product of (p1-p3)x(p2-p3)
    normal_vector_first.x = (p1_p3.y*p2_p3.z)-(p1_p3.z*p2_p3.y);
    normal_vector_first.y = (-1)*((p1_p3.x*p2_p3.z)-(p1_p3.z*p2_p3.x));
    normal_vector_first.z = (p1_p3.x*p2_p3.y)-(p1_p3.y*p2_p3.x);
    float normal_vector_length = sqrtf(powf(normal_vector_first.x,2)+powf(normal_vector_first.y,2)+powf(normal_vector_first.z,2));
    Vector3f normal_vector = {normal_vector_first.x/normal_vector_length,normal_vector_first.y/normal_vector_length,normal_vector_first.z/normal_vector_length};

    float a,f,u,v;
    Vector3f h,s,q;
    h.x =(ray.direction.y*p2_p3.z)-(ray.direction.z*p2_p3.y);
    h.y = (-1)*((ray.direction.x*p2_p3.z)-(ray.direction.z*p2_p3.x));
    h.z = (ray.direction.x*p2_p3.y)-(ray.direction.y*p2_p3.x);

    a=(p1_p3.x*h.x)+(p1_p3.y*h.y)+(p1_p3.z*h.z);

    if(a>-EPSILON && a<EPSILON){
        returnVal.is_intersected=false;
        return returnVal;
    }
    f=1.0/a;

    s = {ray.origin.x-p3.x,ray.origin.y-p3.y,ray.origin.z-p3.z};
    u = f*(s.x*h.x+s.y*h.y+s.z*h.z);
    if(u<0.0 || u>1.0) {
        returnVal.is_intersected = false;
        return returnVal;
    }
    q.x = (s.y*p1_p3.z)-(s.z*p1_p3.y);
    q.y = (-1)*((s.x*p1_p3.z)-(s.z*p1_p3.x));
    q.z = (s.x*p1_p3.y)-(s.y*p1_p3.x);

    v= f*(ray.direction.x*q.x+ray.direction.y*q.y+ray.direction.z*q.z);
    if(v<0.0 || u+v>1.0) {
         returnVal.is_intersected = false;
         return  returnVal;
    }
    float t = f*(p2_p3.x*q.x+p2_p3.y*q.y+p2_p3.z*q.z);
    Vector3f intersection_point;
    if(t>EPSILON && t<1/EPSILON){
        intersection_point=ray.getPoint(t);
        is_intersected=true;
    }else{
        is_intersected=false;
    }

    returnVal.is_intersected=is_intersected;
    returnVal.intersect_point=intersection_point;
    returnVal.normal_vector=normal_vector;
    returnVal.t=t;
    returnVal.mat_ID=this->matIndex;

    return returnVal;

}

Mesh::Mesh()
{}

/* Constructor for mesh. You will implement this. */
Mesh::Mesh(int id, int matIndex, const vector<Triangle>& faces, vector<int> *pIndices, vector<Vector3f> *pVertices)
        : Shape(id, matIndex)
{
    this->vertices= *pVertices;
    this->pIndices= *pIndices;
    this->faces = faces;
}

/* Mesh-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will need. It is in defs.h file. */
ReturnVal Mesh::intersect(const Ray & ray) const
{
    ReturnVal returnVal1;
    float EPSILON = pScene->intTestEps;
    returnVal1.is_intersected= false;
    float t_min=INFINITY;
    for(int face = 0 ; face<faces.size();face++){
        ReturnVal returnVal;
        bool is_intersected;
        Vector3f p1=vertices[pIndices[face*3+0]-1];
        Vector3f p2=vertices[pIndices[face*3+1]-1];
        Vector3f p3=vertices[pIndices[face*3+2]-1];
        // finding normal of triangle
        float ray_direction_length = sqrtf(powf(ray.direction.x,2)+powf(ray.direction.y,2)+powf(ray.direction.z,2));
        Vector3f direction_normal = {(ray.direction.x/ray_direction_length),(ray.direction.y/ray_direction_length),(ray.direction.z/ray_direction_length)};
        // finding normal of triangle

        Vector3f p1_p3 = {p1.x-p3.x,p1.y-p3.y,p1.z-p3.z};//(p1-p3)
        Vector3f p2_p3 = {p2.x-p3.x,p2.y-p3.y,p2.z-p3.z};//(p2-p3)
        Vector3f normal_vector_first;
        //cross product of (p1-p3)x(p2-p3)
        normal_vector_first.x = (p1_p3.y*p2_p3.z)-(p1_p3.z*p2_p3.y);
        normal_vector_first.y = (-1)*((p1_p3.x*p2_p3.z)-(p1_p3.z*p2_p3.x));
        normal_vector_first.z = (p1_p3.x*p2_p3.y)-(p1_p3.y*p2_p3.x);
        float normal_vector_length = sqrtf(pow(normal_vector_first.x,2)+powf(normal_vector_first.y,2)+powf(normal_vector_first.z,2));
        Vector3f normal_vector = {normal_vector_first.x/normal_vector_length,normal_vector_first.y/normal_vector_length,normal_vector_first.z/normal_vector_length};

        float a,f,u,v;
        Vector3f h,s,q;
        h.x =(ray.direction.y*p2_p3.z)-(ray.direction.z*p2_p3.y);
        h.y = (-1)*((ray.direction.x*p2_p3.z)-(ray.direction.z*p2_p3.x));
        h.z = (ray.direction.x*p2_p3.y)-(ray.direction.y*p2_p3.x);

        a=(p1_p3.x*h.x)+(p1_p3.y*h.y)+(p1_p3.z*h.z);

        if(a>-EPSILON && a<EPSILON) {
            returnVal.is_intersected = false;
            continue;
        }
        f=1.0/a;

        s = {ray.origin.x-p3.x,ray.origin.y-p3.y,ray.origin.z-p3.z};
        u = f*(s.x*h.x+s.y*h.y+s.z*h.z);
        if(u<0.0 || u>1.0) {
          returnVal.is_intersected = false;
            continue;
        }
        q.x = (s.y*p1_p3.z)-(s.z*p1_p3.y);
        q.y = (-1)*((s.x*p1_p3.z)-(s.z*p1_p3.x));
        q.z = (s.x*p1_p3.y)-(s.y*p1_p3.x);

        v= f*(ray.direction.x*q.x+ray.direction.y*q.y+ray.direction.z*q.z);
        if(v<0.0 || u+v>1.0) {
            returnVal.is_intersected = false;
            continue;
        }
        float t = f*(p2_p3.x*q.x+p2_p3.y*q.y+p2_p3.z*q.z);
        Vector3f intersection_point;
        if(t>EPSILON && t<1/EPSILON){
            intersection_point=ray.getPoint(t);
            is_intersected=true;

        }else{
            is_intersected=false;
        }

        returnVal.is_intersected=is_intersected;
        returnVal.intersect_point=intersection_point;
        returnVal.normal_vector=normal_vector;

        if (returnVal.is_intersected && ray.gett((returnVal.intersect_point))<t_min){
            t_min=ray.gett(returnVal.intersect_point);
            returnVal1.intersect_point=intersection_point;
            returnVal1.is_intersected=true;
            returnVal1.normal_vector=normal_vector;
            returnVal1.t=t;
            returnVal1.mat_ID=this->matIndex;
        }

    }
    return  returnVal1;
}


