#include "geom.h"
#include "camera.h"
#include "utils.h"

void Camera::Init(const Vector3f& eye_, const Quaternionf& orientation
	, float ry, int width_, int height_)
{
	eye = eye_;
	width = width_;
	height = height_;
	float rx = ry * width / height;
	X = rx * orientation._transformVector(Vector3f::UnitX());
	Y = ry * orientation._transformVector(Vector3f::UnitY());
	Z = -1 * orientation._transformVector(Vector3f::UnitZ());
}

Ray Camera::GenerateRay(int x, int y)
{
	float dx = 2 * (x + random_float()) / width - 1;
	float dy = 2 * (y + random_float()) / height - 1;
	Vector3f dir = dx * X + dy * Y + Z;
	dir.normalize();
	return Ray(eye, dir);
}
