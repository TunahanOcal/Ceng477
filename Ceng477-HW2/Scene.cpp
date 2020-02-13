#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <fstream>
#include <cmath>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"
#include <algorithm>

using namespace tinyxml2;
using namespace std;

/*
	Transformations, clipping, culling, rasterization are done here.
	You can define helper functions inside Scene class implementation.
*/
void Scene::forwardRenderingPipeline(Camera *camera){

    for(Model *model:models){

        for(int i = 0; i <model->numberOfTriangles;i++){
            Triangle triangle = model->triangles[i];

            Vec3 *vec_triangle1 = vertices[triangle.getFirstVertexId()-1];
            Vec3 *vec_triangle2= vertices[triangle.getSecondVertexId()-1];
            Vec3 *vec_triangle3= vertices[triangle.getThirdVertexId()-1];

            Vec4 *vector1 = new Vec4();
            vector1->x = vec_triangle1->x;
            vector1->y = vec_triangle1->y;
            vector1->z = vec_triangle1->z;
            vector1->colorId = vec_triangle1->colorId;
            vector1->t = 1;
            Vec4 *vector2 = new Vec4();
            vector2->x = vec_triangle2->x;
            vector2->y = vec_triangle2->y;
            vector2->z = vec_triangle2->z;
            vector2->colorId = vec_triangle2->colorId;
            vector2->t = 1;
            Vec4 *vector3 = new Vec4();
            vector3->x = vec_triangle3->x;
            vector3->y = vec_triangle3->y;
            vector3->z = vec_triangle3->z;
            vector3->colorId = vec_triangle3->colorId;
            vector3->t = 1;


            for(int i = 0 ; i<model->numberOfTransformations;i++){

                    if(model->transformationTypes[i] == 'r'){
                         vector1 = rotate(vector1,rotations[model->transformationIds[i]-1]);
                         vector2 = rotate(vector2,rotations[model->transformationIds[i]-1]);
                         vector3 = rotate(vector3,rotations[model->transformationIds[i]-1]);
                      }
                    else if(model->transformationTypes[i]=='t'){
                        vector1 = translate(vector1,translations[model->transformationIds[i]-1]);
                        vector2 = translate(vector2,translations[model->transformationIds[i]-1]);
                        vector3 = translate(vector3,translations[model->transformationIds[i]-1]);
                    }
                    else if(model->transformationTypes[i]=='s'){
                        vector1 = scale(vector1,scalings[model->transformationIds[i]-1]);
                        vector2 = scale(vector2,scalings[model->transformationIds[i]-1]);
                        vector3 = scale(vector3,scalings[model->transformationIds[i]-1]);
                    }
            }

            if(cullingEnabled){
                if(!backFaceCulling(vector1, vector2, vector3,*camera)){
                    continue;
                }
            }
            *vector1 = multiplyMatrixWithVector4(cameraTransformation(camera),vector1);
            *vector2 = multiplyMatrixWithVector4(cameraTransformation(camera),vector2);
            *vector3 = multiplyMatrixWithVector4(cameraTransformation(camera),vector3);

            if(projectionType) {
                *vector1 = multiplyMatrixWithVector4(perspectiveProjection(camera),vector1);
                *vector2 = multiplyMatrixWithVector4(perspectiveProjection(camera),vector2);
                *vector3 = multiplyMatrixWithVector4(perspectiveProjection(camera),vector3);
            }else{
                *vector1 = multiplyMatrixWithVector4(orthographicProjection(camera),vector1);
                *vector2 = multiplyMatrixWithVector4(orthographicProjection(camera),vector2);
                *vector3 = multiplyMatrixWithVector4(orthographicProjection(camera),vector3);
            }

            vector1 = perspectiveDivide(vector1);
            vector2 = perspectiveDivide(vector2);
            vector3 = perspectiveDivide(vector3);

            Vec3 viewport1 = viewportProjection(camera,vector1);
            Vec3 viewport2 = viewportProjection(camera,vector2);
            Vec3 viewport3 = viewportProjection(camera,vector3);



            if(model->type==0) {

                liangBarsky(viewport1, viewport2, camera);
                liangBarsky(viewport1, viewport3, camera);
                liangBarsky(viewport2,viewport3,camera);

            }

            if(model->type == 1) {
                triangleRasterize(viewport1, viewport2, viewport3,camera);
            }
        }//Triangles

    }//Models

}//forwardRendering

Vec4* Scene::rotate(Vec4 *vector, Rotation *rotation){

    Vec3 u;
    Vec3 v;
    Vec3 w;
    u.x = rotation->ux;
    u.y = rotation->uy;
    u.z = rotation->uz;
    normalizeVec3(u);
    if(min(abs(rotation->ux),abs(rotation->uy))>abs(rotation->uz)){
        v.x=-1*rotation->uy;
        v.y=rotation->ux;
        v.z = 0;
    }else if(abs(rotation->ux)>abs(rotation->uy)){
        v.x = -1*rotation->uz;
        v.y = 0;
        v.z = rotation->ux;
    }else{
        v.x = 0;
        v.y = -1*rotation->uz;
        v.z = rotation->uy;
    }
    normalizeVec3(v);
    w = crossProductVec3(u,v);
    normalizeVec3(w);

    double theta = (rotation->angle*M_PI)/180.0;
    double m_matrix[4][4] = {{u.x,u.y,u.z,0},{v.x,v.y,v.z,0},{w.x,w.y,w.z,0},{0,0,0,1}};
    double m_inverse_matrix[4][4] = {{u.x,v.x,w.x,0},{u.y,v.y,w.y,0},{u.z,v.z,w.z,0},{0,0,0,1}};
    double rotation_matrix[4][4] = {{1,0,0,0},{0,cos(theta),-1*(sin(theta)),0},{0,sin(theta),cos(theta),0},{0,0,0,1}};

    Vec4 *rotatedVector = new Vec4();
    *rotatedVector = multiplyMatrixWithVector4(Matrix4(m_matrix),vector);
    *rotatedVector = multiplyMatrixWithVector4(Matrix4(rotation_matrix),rotatedVector);
    *rotatedVector = multiplyMatrixWithVector4(Matrix4(m_inverse_matrix),rotatedVector);


    return rotatedVector;
}

Vec4* Scene::scale(Vec4 *vector,Scaling *scaling){

    double matrix[4][4] = {{scaling->sx,0,0,0},{0,scaling->sy,0,0},{0,0,scaling->sz,0},{0,0,0,1}};
    Vec4 *scaledVector = new Vec4();
    *scaledVector = multiplyMatrixWithVector4(Matrix4(matrix),vector);

    return scaledVector;
}

Vec4* Scene::translate(Vec4 *vector, Translation *translation){

    double matrix[4][4] = {{1,0,0,translation->tx},{0,1,0,translation->ty},{0,0,1,translation->tz},{0,0,0,1}};
    Vec4 *translatedVector = new Vec4();
    *translatedVector = multiplyMatrixWithVector4(Matrix4(matrix),vector);

    return translatedVector;

}
//Orthographic Projection Transformation Matrix
Matrix4 Scene::orthographicProjection(Camera *camera){
    double l = camera->left;
    double r = camera->right;
    double b = camera->bottom;
    double t = camera->top;
    double f = camera->far;
    double n = camera->near;
    double projectionMatrix[4][4] = {{2/(r-l),0,0,-1*((r+l)/(r-l))},
                                    {0,2/(t-b),0,-1*((t+b)/(t-b))},
                                    {0,0,-1*(2/(f-n)),-1*((f+n)/(f-n))},
                                    {0,0,0,1}};
    return Matrix4(projectionMatrix);
}

// Perspective Projection Matrix Function
Matrix4 Scene::perspectiveProjection(Camera *camera){
    double l = camera->left;
    double r = camera->right;
    double b = camera->bottom;
    double t = camera->top;
    double f = camera->far;
    double n = camera->near;

    double projectionMatrix[4][4] = {{(2*n)/(r-l),0,(r+l)/(r-l),0},
                                     {0,(2*n)/(t-b),(t+b)/(t-b),0},
                                     {0,0,-1*((f+n)/(f-n)),-1*((2*f*n)/(f-n))},
                                     {0,0,-1,0}};

    return Matrix4(projectionMatrix);

}
// Camera Transformation Matrix Function
Matrix4 Scene::cameraTransformation(Camera *camera){

    double camera_m_matrix[4][4] = {{camera->u.x,camera->u.y,camera->u.z,0},{camera->v.x,camera->v.y,camera->v.z,0},{camera->w.x,camera->w.y,camera->w.z,0},{0,0,0,1}};
    double camera_e_matrix[4][4] = {{1,0,0,-1*camera->pos.x},{0,1,0,-1*camera->pos.y},{0,0,1,-1*camera->pos.z},{0,0,0,1}};
    Matrix4 camera_transformation_matrix =  multiplyMatrixWithMatrix(Matrix4(camera_m_matrix),Matrix4(camera_e_matrix));

    return  camera_transformation_matrix;

}
Vec4* Scene::perspectiveDivide(Vec4 *vec4){
    vec4->x /=vec4->t;
    vec4->y /=vec4->t;
    vec4->z /=vec4->t;
    vec4->t /=vec4->t;
    return vec4;
}

//Liang-Barsky Algorithm Main Function
void Scene::liangBarsky(Vec3 vector1, Vec3 vector2,Camera *camera){

    double te = 0, tl = 1;
    vector1.x = (int)vector1.x;
    vector2.x = (int)vector2.x;
    vector1.y = (int)vector1.y;
    vector2.y = (int)vector2.y;

    double dx = vector2.x-vector1.x;
    double dy = vector2.y-vector1.y;
    int xmin = 0;
    int xmax = camera->horRes-1;
    int ymin = 0;
    int ymax = camera->verRes-1;



    if(visible(-1*dx,(vector1.x-xmin),&te,&tl)) {
        if (visible(dx, (xmax-vector1.x), &te, &tl)) {
            if (visible(-1*dy, (vector1.y-ymin), &te, &tl)){
                if (visible(dy, ( ymax-vector1.y), &te, &tl)) {
                    if (tl < 1) {
                        vector2.x = vector1.x + dx * tl;
                        vector2.y = vector1.y + dy * tl;
                    }
                    if (te > 0) {
                        vector1.x = vector1.x + dx * te;
                        vector1.y = vector1.y + dy * te;
                    }
                    midPointLine(vector1,vector2);

                }
            }
        }
    }

}
// Liang-Barsky Algorithm  Visible Function
bool Scene::visible(double den, double num, double *te, double *tl){
    double t;

    if(den<0){
        t = num/den;
        if(t>*tl)
            return false;
        if(t>*te)
            *te=t;

    }else if(den>0){
        t = num/den;
        if(t<*te)
            return false;
        if(t<*tl)
            *tl = t;

    }else if(num<0){
        return  false;
    }
    return true;
}

//backFace Culling Function
bool Scene::backFaceCulling(Vec4 *a,Vec4 *b, Vec4 *c , Camera camera){
    Vec3 a1;
    Vec3 b1;
    Vec3 c1;
    a1.x = a->x;
    a1.y = a->y;
    a1.z = a->z;
    b1.x = b->x;
    b1.y = b->y;
    b1.z = b->z;
    c1.x = c->x;
    c1.y = c->y;
    c1.z = c->z;

    Vec3 c_a = subtractVec3(c1,a1);
    Vec3 b_a = subtractVec3(b1,a1);
    Vec3 triangleNormal = crossProductVec3(b_a,c_a);
    triangleNormal = normalizeVec3(triangleNormal);



    Vec3 cameraToTriangle = subtractVec3(a1,camera.pos);

    if(dotProductVec3(cameraToTriangle,triangleNormal)<0)
        return true;
    else
        return false;


}
Vec3 Scene::viewportProjection(Camera *camera, Vec4 *point){
    int nx = camera->horRes;
    int ny = camera->verRes;

    Vec3 viewPoint;

    viewPoint.x = point->x*(nx/2)+(nx-1)/2;

    viewPoint.y = point->y*(ny/2)+(ny-1)/2;

    viewPoint.colorId =point->colorId;
    return  viewPoint;
}

int Scene::edgeFunction(int x0,int y0, int x1,int y1,int x,int y){

    int edgeValue = x*(y0-y1) + y*(x1-x0) + x0*y1-y0*x1;

    return edgeValue;
}

void Scene::triangleRasterize(Vec3 vec0, Vec3 vec1, Vec3 vec2,Camera* camera){

    double xmin;
    double ymin;
    double xmax;
    double ymax;

    xmin = min(vec0.x,min(vec1.x,vec2.x));
    ymin = min(vec0.y,min(vec1.y,vec2.y));
    xmax = max(vec0.x,max(vec1.x,vec2.x));
    ymax = max(vec0.y,max(vec1.y,vec2.y));

    Vec3 point;
    Color colorVector;
    Color *color0;
    Color *color1;
    Color *color2;
    int x0 = static_cast<int>(vec0.x);
    int y0 = static_cast<int>(vec0.y);
    int x1 = static_cast<int>(vec1.x);
    int y1 = static_cast<int>(vec1.y);
    int x2 = static_cast<int>(vec2.x);
    int y2 = static_cast<int>(vec2.y);

    for(int x = (int)xmin;x <= (int)xmax ;x++){
        for(int y = (int)ymin; y<=(int)ymax;y++ ){
            if(x<camera->horRes && x>=0 && y<camera->verRes && y>=0) {
                point.x = x;
                point.y = y;

                double alpha =(double)edgeFunction(x2,y2,x1,y1,x,y)/(double)edgeFunction(x2,y2,x1,y1,x0,y0);
                double beta = (double)edgeFunction(x0,y0,x2,y2,x,y)/(double)edgeFunction(x0,y0,x2,y2,x1,y1);
                double teta =(double)edgeFunction(x1,y1,x0,y0,x,y)/(double)edgeFunction(x1,y1,x0,y0,x2,y2);


                color0 = colorsOfVertices[vec0.colorId - 1];
                color1 = colorsOfVertices[vec1.colorId - 1];
                color2 = colorsOfVertices[vec2.colorId - 1];

                if (alpha >= 0 && beta >= 0 && teta >= 0) {
                    colorVector.r = makeBetweenZeroAnd255(color0->r * alpha + color1->r * beta + color2->r * teta);
                    colorVector.b = makeBetweenZeroAnd255(color0->b * alpha + color1->b * beta + color2->b * teta);
                    colorVector.g = makeBetweenZeroAnd255(color0->g * alpha + color1->g * beta + color2->g * teta);
                    image[x][y] = colorVector;
                }

            }
        }
    }

}

void Scene::midPointLine(Vec3 vector0 , Vec3 vector1) {
    Color color;
    Color dc;

    bool controlys = abs(vector1.y - vector0.y) > abs(vector1.x- vector0.x);
    if(controlys) {
        swap(vector0.x,vector0.y);
        swap(vector1.x, vector1.y);
    }
    if(vector1.x<vector0.x){
        swap(vector0,vector1);
    }
    int step;
    if(vector0.y<vector1.y)
        step = 1;
    else
        step = -1;
    int x1_x0 = vector1.x - vector0.x;
    int y1_y0 = abs(vector1.y-vector0.y);
    int y = vector0.y;
    float d = -1*y1_y0 + 0.5 * (x1_x0);

    color.r = colorsOfVertices[vector0.colorId - 1]->r;
    color.b = colorsOfVertices[vector0.colorId - 1]->b;
    color.g = colorsOfVertices[vector0.colorId - 1]->g;

    dc.r = (colorsOfVertices[vector1.colorId - 1]->r - color.r) / x1_x0;
    dc.b = (colorsOfVertices[vector1.colorId - 1]->b - color.b) / x1_x0;
    dc.g = (colorsOfVertices[vector1.colorId - 1]->g - color.g) / x1_x0;


    for (int x = (int)vector0.x; x <=(int)vector1.x; x++) {

        color.r = makeBetweenZeroAnd255(color.r);
        color.b = makeBetweenZeroAnd255(color.b);
        color.g = makeBetweenZeroAnd255(color.g);
        if(controlys){
            image[y][x] = color;
        }else{
            image[x][y] = color;
        }

        if (d < 0) {
            y = y + step;
            d += (-1*y1_y0+x1_x0);
        } else {
            d += -1*y1_y0;

        }
        color.r += dc.r;
        color.b += dc.b;
        color.g += dc.g;
    }
}

Vec4 Scene::multiplyMatrixWithVector4(Matrix4 m, Vec4 *v)
{
    double values[4];
    double total;

    for (int i = 0; i < 4; i++)
    {
        total = 0;
        for (int j = 0; j < 4; j++)
        {
            total += m.val[i][j] * v->getElementAt(j);
        }
        values[i] = total;
    }

    return Vec4(values[0], values[1], values[2], values[3], v->colorId);
}


/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL)
		pElement->QueryBoolText(&cullingEnabled);

	// read projection type
	pElement = pRoot->FirstChildElement("ProjectionType");
	if (pElement != NULL)
		pElement->QueryIntText(&projectionType);

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read models
	pElement = pRoot->FirstChildElement("Models");

	XMLElement *pModel = pElement->FirstChildElement("Model");
	XMLElement *modelElement;
	while (pModel != NULL)
	{
		Model *model = new Model();

		pModel->QueryIntAttribute("id", &model->modelId);
		pModel->QueryIntAttribute("type", &model->type);

		// read model transformations
		XMLElement *pTransformations = pModel->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		pTransformations->QueryIntAttribute("count", &model->numberOfTransformations);

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			model->transformationTypes.push_back(transformationType);
			model->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		// read model triangles
		XMLElement *pTriangles = pModel->FirstChildElement("Triangles");
		XMLElement *pTriangle = pTriangles->FirstChildElement("Triangle");

		pTriangles->QueryIntAttribute("count", &model->numberOfTriangles);

		while (pTriangle != NULL)
		{
			int v1, v2, v3;

			str = pTriangle->GetText();
			sscanf(str, "%d %d %d", &v1, &v2, &v3);

			model->triangles.push_back(Triangle(v1, v2, v3));

			pTriangle = pTriangle->NextSiblingElement("Triangle");
		}

		models.push_back(model);

		pModel = pModel->NextSiblingElement("Model");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	// if image is filled before, just change color rgb values with the background color
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFileName.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}