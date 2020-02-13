#ifndef _SCENE_H_
#define _SCENE_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Matrix4.h"

using namespace std;

class Scene
{
public:
	Color backgroundColor;
	bool cullingEnabled;
	int projectionType;

	vector< vector<Color> > image;
	vector< Camera* > cameras;
	vector< Vec3* > vertices;
	vector< Color* > colorsOfVertices;
	vector< Scaling* > scalings;
	vector< Rotation* > rotations;
	vector< Translation* > translations;
	vector< Model* > models;

	Scene(const char *xmlPath);

	void initializeImage(Camera* camera);
	void forwardRenderingPipeline(Camera* camera);
	int makeBetweenZeroAnd255(double value);
	void writeImageToPPMFile(Camera* camera);
	void convertPPMToPNG(string ppmFileName, int osType);



	Matrix4 orthographicProjection(Camera *camera);

    Matrix4 perspectiveProjection(Camera *camera);

    Matrix4 cameraTransformation(Camera *camera);

    bool backFaceCulling(Vec4 *a,Vec4 *b, Vec4 *c , Camera camera);

    Vec3 viewportProjection(Camera *camera, Vec4 *point);

    void triangleRasterize(Vec3 vec0, Vec3 vec1, Vec3 vec2,Camera* camera);

    void midPointLine(Vec3 vector0, Vec3 vector1);

    void liangBarsky(Vec3 vector1, Vec3 vector2, Camera *camera);

    Vec4 *translate(Vec4 *vector, Translation *translation);

    Vec4 *scale(Vec4 *vector, Scaling *scaling);

    Vec4 *rotate(Vec4 *vector, Rotation *rotation);

    Vec4 multiplyMatrixWithVector4(Matrix4 m, Vec4 *v);

    Vec4 *perspectiveDivide(Vec4 *vec4);

    int edgeFunction(int x0, int y0, int x1, int y1, int x, int y);

    bool visible(double den, double num, double *te, double *tl);
};

#endif
