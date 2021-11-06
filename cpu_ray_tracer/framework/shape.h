#ifndef SHAPE_H
#define SHAPE_H

#include <limits>
#include "meshData.h"

const float FINFINITY = std::numeric_limits<float>::infinity();

class Shape;

///////////////////////////////////////////////////////////////////////
//Intersection
struct Intersection {
	Shape* shape_hit = nullptr;
	float t = FINFINITY;
	Vector3f pos;
	Vector3f normal;
};

///////////////////////////////////////////////////////////////////////
//Slab
struct Slab {
	Vector3f normal;
	float d0, d1;
};

///////////////////////////////////////////////////////////////////////
//Interval
class Interval {
public:
	Interval() : t0(0), t1(FINFINITY) {}
	Interval(float t0, float t1, const Vector3f& n0, const Vector3f& n1);

	void Empty() { t0 = 0; t1 = -1; }

	void Intersect(const Interval& interval);
	void Intersect(const Ray& ray, const Slab& slab);

public:
	float t0, t1;
	Vector3f n0, n1;
};

///////////////////////////////////////////////////////////////////////
//Shape (Sphere & Box & Cylinder & Triangle)

class Shape {
public:
	Shape(const Material& m);
	virtual ~Shape() {}
	virtual bool Intersect(const Ray& ray, Intersection& info) = 0;
	virtual Bbox BoundingBox() const { return bbox; };
	virtual float Distance(const Vector3f& point) const = 0;

	const Material& mat;
	Bbox bbox;
};

class Sphere final: public Shape {
public:
	Sphere(const Vector3f c, float r, const Material& mat);
	virtual bool Intersect(const Ray& ray, Intersection& info) override;
	virtual float Distance(const Vector3f& point) const override;

	const Vector3f& GetCenter() const { return center; }
	float GetRadius() const { return radius; }

private:
	Vector3f center;
	float radius;
};

class Box final : public Shape {
public:
	Box(const Vector3f b, const Vector3f d, const Material& mat);
	virtual bool Intersect(const Ray& ray, Intersection& info) override;
	virtual float Distance(const Vector3f& point) const override;

private:
	Vector3f base;
	Vector3f diag;
};

class Cylinder final : public Shape {
public:
	Cylinder(const Vector3f b, const Vector3f a, float r, const Material& mat);
	virtual bool Intersect(const Ray& ray, Intersection& info) override;
	virtual float Distance(const Vector3f& point) const override;

private:
	Vector3f base;
	Vector3f axis;
	float radius;
};

class Triangle final : public Shape {
public:
	Triangle(const std::vector<VertexData>& vertices, const TriData& indices, const Material& mat);
	virtual bool Intersect(const Ray& ray, Intersection& info) override;
	virtual float Distance(const Vector3f& point) const override;

private:
	const std::vector<VertexData>& vertices;
	const TriData& indices;
};

class RaymarchShape : public Shape {
public:
	RaymarchShape(Shape* A, Shape* B, const Material& mat)
		: a(A), b(B), Shape(mat) {}
	virtual ~RaymarchShape();
	virtual bool Intersect(const Ray& ray, Intersection& info) override;
	virtual float Distance(const Vector3f& point) const = 0;

protected:
	Shape* a = nullptr;
	Shape* b = nullptr;
};

class Union : public RaymarchShape {
public:
	Union(Shape* A, Shape* B, const Material& mat);
	virtual float Distance(const Vector3f& point) const override;
};

class Intersect : public RaymarchShape {
public:
	Intersect(Shape* A, Shape* B, const Material& mat);
	virtual float Distance(const Vector3f& point) const override;
};

class Difference : public RaymarchShape {
public:
	Difference(Shape* A, Shape* B, const Material& mat);
	virtual float Distance(const Vector3f& point) const override;
};

#endif