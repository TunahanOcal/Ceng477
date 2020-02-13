#include "Scene.h"
#include "Camera.h"
#include "Light.h"
#include "Material.h"
#include "Shape.h"
#include "tinyxml2.h"
#include "Image.h"
#include  <math.h>

using namespace tinyxml2;

/* 
 * Must render the scene from each camera's viewpoint and create an image.
 * You can use the methods of the Image class to save the image as a PPM file. 
 */
void Scene::renderScene(void) {
    Ray ray;
    float t_min = INFINITY;
    float t;
    Camera *camera;
    Image *image;
    Shape *object;
    Shape *object_front = NULL;
    ReturnVal returnVal;
    ReturnVal returnVal1;

    for (int i = 0; i < pScene->cameras.size(); i++) {
        camera = pScene->cameras[i];
        image = new Image(camera->imgPlane.nx, camera->imgPlane.ny);
        for (int nx = 0; nx < pScene->cameras[i]->imgPlane.nx; nx++) {
            for (int ny = 0; ny < pScene->cameras[i]->imgPlane.ny; ny++) {

                returnVal1.is_intersected = false;
                t_min = INFINITY;
                for (int obj = 0; obj < pScene->objects.size(); obj++) {
                    ray = camera->getPrimaryRay(nx, ny);
                    object = pScene->objects[obj];
                    returnVal = object->intersect(ray);
                    if (returnVal.is_intersected) {
                        t = ray.gett(returnVal.intersect_point);
                        if (t < t_min) {
                            t_min = t;
                            returnVal1 = returnVal;
                        }
                    }
                }
                if (returnVal1.is_intersected) {

                    Vector3f color_vector = shade_and_shadow(ray, maxRecursionDepth, returnVal1);
                    Color color;
                    color.red=color_vector.r;
                    color.blu=color_vector.b;
                    color.grn=color_vector.g;
                    image->setPixelValue(nx, ny, color);

                } else {
                    Color *color = new Color();
                    color->blu = backgroundColor.b;
                    color->grn = backgroundColor.g;
                    color->red = backgroundColor.r;
                    image->setPixelValue(nx, ny, *color);
                }
            }
            image->saveImage(camera->imageName);
        }
    }
}
Vector3f Scene::shade_and_shadow(Ray &ray, int maxRecursionDepth, ReturnVal &returnVal1){


    Vector3f reflection_color = {0,0,0};
    Vector3f color;
    if(maxRecursionDepth<0){
        color.g = 0;
        color.b = 0;
        color.r = 0;

        return color;
    }
    Vector3f color_vector;
    Material *material;
    material = pScene->materials[(returnVal1.mat_ID)-1];

    //Ambient Light
    color_vector.b = material->ambientRef.b*ambientLight.b;
    color_vector.g= material->ambientRef.g*ambientLight.g;
    color_vector.r= material->ambientRef.r*ambientLight.r;


    for(int ligth=0;ligth<lights.size();ligth++) {
        PointLight *pointLight = lights[ligth];
        bool flag = false;
        Vector3f ligth_contribution = lights[ligth]->computeLightContribution(returnVal1.intersect_point);

        Vector3f wi = {pointLight->position.x - returnVal1.intersect_point.x,
                       pointLight->position.y - returnVal1.intersect_point.y,
                       pointLight->position.z - returnVal1.intersect_point.z};
        float wi_length = sqrtf(powf(wi.x, 2) + powf(wi.y, 2) + powf(wi.z, 2));
        wi = {wi.x / wi_length, wi.y / wi_length, wi.z / wi_length};
        float wi_dot_normal = (wi.x * returnVal1.normal_vector.x + wi.y * returnVal1.normal_vector.y +
                               wi.z * returnVal1.normal_vector.z);

        Ray shadowRay;
        Vector3f origin_ray1 = {returnVal1.intersect_point.x + wi.x * shadowRayEps, returnVal1.intersect_point.y + wi.y * shadowRayEps,
                                returnVal1.intersect_point.z + wi.z * shadowRayEps};
       // float origin_ray1_length = sqrtf(powf(origin_ray1.x,2)+powf(origin_ray1.y,2)+powf(origin_ray1.z,2));
       // origin_ray1 = {origin_ray1.x/origin_ray1_length,origin_ray1.y/origin_ray1_length,origin_ray1.z/origin_ray1_length};
        shadowRay.direction = wi;
        shadowRay.origin = origin_ray1;
        float tLigth = (pointLight->position.x - shadowRay.origin.x) / shadowRay.direction.x;

        for (Shape *shape:pScene->objects) {

            ReturnVal returnVal = shape->intersect(shadowRay);

            if (returnVal.is_intersected) {
                if (tLigth > returnVal.t) {
                    flag = true;
                    continue;
                }
            }

        }


        if (!flag) {
            float costeta = 0.0;
            Vector3f normal = returnVal1.normal_vector;

            Vector3f w0 = {ray.origin.x - returnVal1.intersect_point.x, ray.origin.y - returnVal1.intersect_point.y,
                           ray.origin.z - returnVal1.intersect_point.z};
            float w0_length = sqrtf(powf(w0.x, 2) + powf(w0.y, 2) + powf(w0.z, 2));

            w0 = {w0.x / w0_length, w0.y / w0_length, w0.z / w0_length};
            float w0_dot_normal = (w0.x * returnVal1.normal_vector.x + w0.y * returnVal1.normal_vector.y,
                    w0.z * returnVal1.normal_vector.z);
            costeta = std::max(0.0f, wi_dot_normal);

            Vector3f h = {wi.x + w0.x, wi.y + w0.y, wi.z + w0.z};
            float h_length = sqrtf(pow(h.x, 2) + powf(h.y, 2) + powf(h.z, 2));;
            h = {h.x / h_length, h.y / h_length, h.z / h_length};

            float h_dot_normal = (h.x * returnVal1.normal_vector.x + h.y * returnVal1.normal_vector.y +
                                  h.z * returnVal1.normal_vector.z);
            Vector3f wr = {(-1) * w0.x +(2 * normal.x * w0_dot_normal), (-1) * w0.y +( 2 * normal.y * w0_dot_normal),
                           (-1) * w0.z + (2 * normal.z * w0_dot_normal)};
            float cosalpha = std::max(0.0f, h_dot_normal);
            float phong = pow(cosalpha,(float)material->phongExp);

            color_vector.b += material->diffuseRef.b * ligth_contribution.b * costeta;
            color_vector.g += material->diffuseRef.g * ligth_contribution.g * costeta;
            color_vector.r += material->diffuseRef.r * ligth_contribution.r * costeta;

            color_vector.b += material->specularRef.b * ligth_contribution.b * phong;
            color_vector.g += material->specularRef.g * ligth_contribution.g * phong;
            color_vector.r += material->specularRef.r * ligth_contribution.r * phong;

            Ray reflectionRay;
            Vector3f origin_ray = {returnVal1.intersect_point.x+ wr.x * shadowRayEps, returnVal1.intersect_point.y + wr.y * shadowRayEps,
                                   returnVal1.intersect_point.z + wr.z * shadowRayEps};
            reflectionRay.origin = origin_ray;
            reflectionRay.direction = wr;
            if(maxRecursionDepth>0 && material->mirrorRef.x>0 &&  material->mirrorRef.y>0 &&  material->mirrorRef.z>0) {
                for (Shape *shape1:pScene->objects) {
                    ReturnVal returnVal = shape1->intersect(reflectionRay);

                    if (returnVal.is_intersected) {
                        reflection_color = color_adding(reflection_color,
                                                        shade_and_shadow(reflectionRay, maxRecursionDepth - 1,
                                                                         returnVal));
                        color_vector.r += reflection_color.r * material->mirrorRef.r;
                        color_vector.g += reflection_color.g * material->mirrorRef.g;
                        color_vector.b += reflection_color.b * material->mirrorRef.b;

                    }

                }
            }

        }
    }
            if (color_vector.b >= 255)
                color_vector.b = 255;
            if (color_vector.g >= 255)
                color_vector.g = 255;
            if (color_vector.r >= 255)
                color_vector.r = 255;

            return color_vector;


    }
Vector3f Scene::color_adding(Vector3f color1, Vector3f color2) {
    Vector3f color;
    color.r = color1.r+color2.r;
    color.g = color1.g+color2.g;
    color.b = color1.b+color2.b;

    return color;
}



// Parses XML file. 
Scene::Scene(const char *xmlPath)
{
    const char *str;
    XMLDocument xmlDoc;
    XMLError eResult;
    XMLElement *pElement;

    maxRecursionDepth = 1;
    shadowRayEps = 0.001;

    eResult = xmlDoc.LoadFile(xmlPath);

    XMLNode *pRoot = xmlDoc.FirstChild();

    pElement = pRoot->FirstChildElement("MaxRecursionDepth");
    if(pElement != nullptr)
        pElement->QueryIntText(&maxRecursionDepth);

    pElement = pRoot->FirstChildElement("BackgroundColor");
    str = pElement->GetText();
    sscanf(str, "%f %f %f", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

    pElement = pRoot->FirstChildElement("ShadowRayEpsilon");
    if(pElement != nullptr)
        pElement->QueryFloatText(&shadowRayEps);

    pElement = pRoot->FirstChildElement("IntersectionTestEpsilon");
    if(pElement != nullptr)
        eResult = pElement->QueryFloatText(&intTestEps);

    // Parse cameras
    pElement = pRoot->FirstChildElement("Cameras");
    XMLElement *pCamera = pElement->FirstChildElement("Camera");
    XMLElement *camElement;
    while(pCamera != nullptr)
    {
        int id;
        char imageName[64];
        Vector3f pos, gaze, up;
        ImagePlane imgPlane;

        eResult = pCamera->QueryIntAttribute("id", &id);
        camElement = pCamera->FirstChildElement("Position");
        str = camElement->GetText();
        sscanf(str, "%f %f %f", &pos.x, &pos.y, &pos.z);
        camElement = pCamera->FirstChildElement("Gaze");
        str = camElement->GetText();
        sscanf(str, "%f %f %f", &gaze.x, &gaze.y, &gaze.z);
        camElement = pCamera->FirstChildElement("Up");
        str = camElement->GetText();
        sscanf(str, "%f %f %f", &up.x, &up.y, &up.z);
        camElement = pCamera->FirstChildElement("NearPlane");
        str = camElement->GetText();
        sscanf(str, "%f %f %f %f", &imgPlane.left, &imgPlane.right, &imgPlane.bottom, &imgPlane.top);
        camElement = pCamera->FirstChildElement("NearDistance");
        eResult = camElement->QueryFloatText(&imgPlane.distance);
        camElement = pCamera->FirstChildElement("ImageResolution");
        str = camElement->GetText();
        sscanf(str, "%d %d", &imgPlane.nx, &imgPlane.ny);
        camElement = pCamera->FirstChildElement("ImageName");
        str = camElement->GetText();
        strcpy(imageName, str);

        cameras.push_back(new Camera(id, imageName, pos, gaze, up, imgPlane));

        pCamera = pCamera->NextSiblingElement("Camera");
    }

    // Parse materals
    pElement = pRoot->FirstChildElement("Materials");
    XMLElement *pMaterial = pElement->FirstChildElement("Material");
    XMLElement *materialElement;
    while(pMaterial != nullptr)
    {
        materials.push_back(new Material());

        int curr = materials.size() - 1;

        eResult = pMaterial->QueryIntAttribute("id", &materials[curr]->id);
        materialElement = pMaterial->FirstChildElement("AmbientReflectance");
        str = materialElement->GetText();
        sscanf(str, "%f %f %f", &materials[curr]->ambientRef.r, &materials[curr]->ambientRef.g, &materials[curr]->ambientRef.b);
        materialElement = pMaterial->FirstChildElement("DiffuseReflectance");
        str = materialElement->GetText();
        sscanf(str, "%f %f %f", &materials[curr]->diffuseRef.r, &materials[curr]->diffuseRef.g, &materials[curr]->diffuseRef.b);
        materialElement = pMaterial->FirstChildElement("SpecularReflectance");
        str = materialElement->GetText();
        sscanf(str, "%f %f %f", &materials[curr]->specularRef.r, &materials[curr]->specularRef.g, &materials[curr]->specularRef.b);
        materialElement = pMaterial->FirstChildElement("MirrorReflectance");
        if(materialElement != nullptr)
        {
            str = materialElement->GetText();
            sscanf(str, "%f %f %f", &materials[curr]->mirrorRef.r, &materials[curr]->mirrorRef.g, &materials[curr]->mirrorRef.b);
        }
        else
        {
            materials[curr]->mirrorRef.r = 0.0;
            materials[curr]->mirrorRef.g = 0.0;
            materials[curr]->mirrorRef.b = 0.0;
        }
        materialElement = pMaterial->FirstChildElement("PhongExponent");
        if(materialElement != nullptr)
            materialElement->QueryIntText(&materials[curr]->phongExp);

        pMaterial = pMaterial->NextSiblingElement("Material");
    }

    // Parse vertex data
    pElement = pRoot->FirstChildElement("VertexData");
    int cursor = 0;
    Vector3f tmpPoint;
    str = pElement->GetText();
    while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
        cursor++;
    while(str[cursor] != '\0')
    {
        for(int cnt = 0 ; cnt < 3 ; cnt++)
        {
            if(cnt == 0)
                tmpPoint.x = atof(str + cursor);
            else if(cnt == 1)
                tmpPoint.y = atof(str + cursor);
            else
                tmpPoint.z = atof(str + cursor);
            while(str[cursor] != ' ' && str[cursor] != '\t' && str[cursor] != '\n')
                cursor++;
            while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
                cursor++;
        }
        vertices.push_back(tmpPoint);
    }

    // Parse objects
    pElement = pRoot->FirstChildElement("Objects");

    // Parse spheres
    XMLElement *pObject = pElement->FirstChildElement("Sphere");
    XMLElement *objElement;
    while(pObject != nullptr)
    {
        int id;
        int matIndex;
        int cIndex;
        float R;

        eResult = pObject->QueryIntAttribute("id", &id);
        objElement = pObject->FirstChildElement("Material");
        eResult = objElement->QueryIntText(&matIndex);
        objElement = pObject->FirstChildElement("Center");
        eResult = objElement->QueryIntText(&cIndex);
        objElement = pObject->FirstChildElement("Radius");
        eResult = objElement->QueryFloatText(&R);

        objects.push_back(new Sphere(id, matIndex, cIndex, R, &vertices));

        pObject = pObject->NextSiblingElement("Sphere");
    }

    // Parse triangles
    pObject = pElement->FirstChildElement("Triangle");
    while(pObject != nullptr)
    {
        int id;
        int matIndex;
        int p1Index;
        int p2Index;
        int p3Index;

        eResult = pObject->QueryIntAttribute("id", &id);
        objElement = pObject->FirstChildElement("Material");
        eResult = objElement->QueryIntText(&matIndex);
        objElement = pObject->FirstChildElement("Indices");
        str = objElement->GetText();
        sscanf(str, "%d %d %d", &p1Index, &p2Index, &p3Index);

        objects.push_back(new Triangle(id, matIndex, p1Index, p2Index, p3Index, &vertices));

        pObject = pObject->NextSiblingElement("Triangle");
    }

    // Parse meshes
    pObject = pElement->FirstChildElement("Mesh");
    while(pObject != nullptr)
    {
        int id;
        int matIndex;
        int p1Index;
        int p2Index;
        int p3Index;
        int cursor = 0;
        int vertexOffset = 0;
        vector<Triangle> faces;
        vector<int> *meshIndices = new vector<int>;

        eResult = pObject->QueryIntAttribute("id", &id);
        objElement = pObject->FirstChildElement("Material");
        eResult = objElement->QueryIntText(&matIndex);
        objElement = pObject->FirstChildElement("Faces");
        objElement->QueryIntAttribute("vertexOffset", &vertexOffset);
        str = objElement->GetText();
        while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
            cursor++;
        while(str[cursor] != '\0')
        {
            for(int cnt = 0 ; cnt < 3 ; cnt++)
            {
                if(cnt == 0)
                    p1Index = atoi(str + cursor) + vertexOffset;
                else if(cnt == 1)
                    p2Index = atoi(str + cursor) + vertexOffset;
                else
                    p3Index = atoi(str + cursor) + vertexOffset;
                while(str[cursor] != ' ' && str[cursor] != '\t' && str[cursor] != '\n')
                    cursor++;
                while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
                    cursor++;
            }
            faces.push_back(*(new Triangle(-1, matIndex, p1Index, p2Index, p3Index, &vertices)));
            meshIndices->push_back(p1Index);
            meshIndices->push_back(p2Index);
            meshIndices->push_back(p3Index);
        }

        objects.push_back(new Mesh(id, matIndex, faces, meshIndices, &vertices));

        pObject = pObject->NextSiblingElement("Mesh");
    }

    // Parse lights
    int id;
    Vector3f position;
    Vector3f intensity;
    pElement = pRoot->FirstChildElement("Lights");

    XMLElement *pLight = pElement->FirstChildElement("AmbientLight");
    XMLElement *lightElement;
    str = pLight->GetText();
    sscanf(str, "%f %f %f", &ambientLight.r, &ambientLight.g, &ambientLight.b);

    pLight = pElement->FirstChildElement("PointLight");
    while(pLight != nullptr)
    {
        eResult = pLight->QueryIntAttribute("id", &id);
        lightElement = pLight->FirstChildElement("Position");
        str = lightElement->GetText();
        sscanf(str, "%f %f %f", &position.x, &position.y, &position.z);
        lightElement = pLight->FirstChildElement("Intensity");
        str = lightElement->GetText();
        sscanf(str, "%f %f %f", &intensity.r, &intensity.g, &intensity.b);

        lights.push_back(new PointLight(position, intensity));

        pLight = pLight->NextSiblingElement("PointLight");
    }
}


