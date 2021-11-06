#ifndef CAMERA_H
#define CAMERA_H

#include "ray.h"

class Camera {
public:
	Camera(){}
	void Init(const Vector3f& eye, const Quaternionf& orientation, float ry, int width, int height);
	Ray GenerateRay(int x, int y);

private:
	Vector3f eye;
	Vector3f X, Y, Z;
	int width = 0, height = 0;
};

#endif