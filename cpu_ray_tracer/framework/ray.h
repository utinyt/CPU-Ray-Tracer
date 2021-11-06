#ifndef RAY_H
#define RAY_H

class Ray {
public:
	Ray(const Vector3f& orig, const Vector3f& dir) : origin(orig), direction(dir) {};
	Vector3f Eval(float t) const { return origin + t * direction; }

	Vector3f origin;
	Vector3f direction;
};

#endif
