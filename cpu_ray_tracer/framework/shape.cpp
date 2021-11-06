#include <algorithm>
#include "geom.h"
#include "ray.h"
#include "shape.h"

const float EPSILON = 0.0001f;

float sign(float f) {
    if (f == 0) return 0.f;
    return f > 0 ? 1.f : -1.f;
}

float dot2(const Vector3f& v) {
    return v.dot(v);
}

///////////////////////////////////////////////////////////////////////
//Interval
Interval::Interval(float t0_, float t1_, const Vector3f& n0_, const Vector3f& n1_) {
    t0 = t0_;
    t1 = t1_;
    n0 = n0_;
    n1 = n1_;
    if (t1 < t0) {
        std::swap(t0, t1);
    }
}

void Interval::Intersect(const Interval& interval) {
    //Maximum of the minimum
    if (t0 < interval.t0) {
        t0 = interval.t0;
        n0 = interval.n0;
    }

    //Minimum of the maximum
    if (t1 > interval.t1) {
        t1 = interval.t1;
        n1 = interval.n1;
    }
}

void Interval::Intersect(const Ray& ray, const Slab& slab) {
    float ND = slab.normal.dot(ray.direction);
    float NQ = slab.normal.dot(ray.origin);

    //Ray intersects both slab planes
    if (ND != 0) {
        float new_t0 = -(slab.d0 + NQ) / ND;
        float new_t1 = -(slab.d1 + NQ) / ND;
        
        Vector3f new_n0, new_n1;
        if (ND < 0) {
            new_n0 = slab.normal;
            new_n1 = -slab.normal;
        }
        else {
            new_n0 = -slab.normal;
            new_n1 = slab.normal;
        }

        Intersect(Interval(new_t0, new_t1, new_n0, new_n1));
    }
    //Ray is parallel to slab planes
    else {
        //plane vs origin test
        float s0 = NQ + slab.d0;
        float s1 = NQ + slab.d1;

        bool sign_difference = (s0 > 0 && s1 <= 0) || (s0 <= 0 && s1 > 0);        
        
        //Same signs - full ray is outside planes
        if(!sign_difference)
            Empty();

        /*
        * Different signs - full ray is between planes
        * intersection *this between [0, infinite] is the same as *this interval
        */
    }
}

///////////////////////////////////////////////////////////////////////
//Shape (Sphere & Box & Cylinder & Triangle)

Shape::Shape(const Material& m) : mat(m) {};

Sphere::Sphere(const Vector3f c, float r, const Material& mat)
    : Shape(mat), center(c), radius(r) {
    bbox = Bbox(center.array() - radius, center.array() + radius);
}

bool Sphere::Intersect(const Ray& ray, Intersection& info) 
{
    Vector3f QC = ray.origin - center;
    float QCD = QC.dot(ray.direction);
    float discriminant = QCD * QCD - QC.dot(QC) + radius * radius;

    if (discriminant < 0)
        return false;

    float discriminant_sqrt = sqrt(discriminant);
    float t0 = -QCD - discriminant_sqrt;
    float t1 = -QCD + discriminant_sqrt;

    float t = t0;
    if (t < EPSILON) // 0 -> EPSILON
    { 
        t = t1;
        if(t < EPSILON) // 0 -> EPSILON
            return false;
    }
    Vector3f pos = ray.Eval(t);
    Vector3f normal = pos - center;
    normal.normalize();
    info = Intersection{ this, t, pos, normal };
    return true;
}

float Sphere::Distance(const Vector3f& point) const
{
    return (point - center).norm() - radius;
}

Box::Box(const Vector3f b, const Vector3f d, const Material& mat)
    : Shape(mat), base(b), diag(d) {
    bbox = Bbox(base, base + diag);
}

bool Box::Intersect(const Ray& ray, Intersection& info)
{
    Slab slabs[3] = {
        Slab{Vector3f(1, 0, 0), -base.x(), -base.x() - diag.x()},
        Slab{Vector3f(0, 1, 0), -base.y(), -base.y() - diag.y()},
        Slab{Vector3f(0, 0, 1), -base.z(), -base.z() - diag.z()}
    };

    Interval interval;
    for (int i = 0; i < 3; ++i) {
        interval.Intersect(ray, slabs[i]);
    }
    
    if (interval.t0 > interval.t1)
        return false;
    else
    {
        float t = interval.t0;
        Vector3f normal = interval.n0;
        if (t < EPSILON) {
            t = interval.t1;
            normal = interval.n1;
            if (t < EPSILON)
                return false;
        }
        info = Intersection{ this, t, ray.Eval(t), normal};
    }

    return true;
}

float Box::Distance(const Vector3f& point) const
{
    Vector3f center = base + 0.5f * diag;
    Vector3f translated_p = point - center;
    Vector3f box_size = (center - base).cwiseAbs();
    Vector3f diagonal_dist = translated_p.cwiseAbs() - box_size; //

    return Vector3f(diagonal_dist.array().max(Vector3f(0, 0, 0).array())).norm();
}

Cylinder::Cylinder(const Vector3f b, const Vector3f a, float r, const Material& mat)
    : Shape(mat), base(b), axis(a), radius(r) {
    Vector3f bot_min = base.array() - radius;
    Vector3f bot_max = base.array() + radius;
    Vector3f top_min = (base + axis).array() - radius;
    Vector3f top_max = (base + axis).array() + radius;

    Vector3f min;
    min.x() = std::min(bot_min.x(), top_min.x());
    min.y() = std::min(bot_min.y(), top_min.y());
    min.z() = std::min(bot_min.z(), top_min.z());

    Vector3f max;
    max.x() = std::max(bot_max.x(), top_max.x());
    max.y() = std::max(bot_max.y(), top_max.y());
    max.z() = std::max(bot_max.z(), top_max.z());

    bbox = Bbox(min, max);
}

bool Cylinder::Intersect(const Ray& ray, Intersection& info)
{
    Quaternionf q = Quaternionf::FromTwoVectors(axis, Vector3f::UnitZ());
    Vector3f new_dir = q._transformVector(ray.direction);
    new_dir.normalize();
    Vector3f new_orig = q._transformVector(ray.origin - base);

    //1st interval : [0, INFINITE]
    Interval interval;
    
    //2nd interval : 1st interval vs slab
    interval.Intersect(Ray(new_orig, new_dir), Slab{Vector3f(0, 0, 1), 0, -axis.norm()});

    //3rd interval :
    float a = new_dir.x() * new_dir.x() + new_dir.y() * new_dir.y();
    float b = 2 * (new_dir.x() * new_orig.x() + new_dir.y() * new_orig.y());
    float c = new_orig.x() * new_orig.x() + new_orig.y() * new_orig.y() - radius * radius;

    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
        return false;

    float discriminant_sqrt = sqrt(discriminant);

    float t0 = (-b - discriminant_sqrt) / (2 * a);
    Vector3f m = new_orig + t0 * new_dir;
    Vector3f n0 = q.conjugate()._transformVector(Vector3f(m.x(), m.y(), 0));
    n0.normalize();

    float t1 = (-b + discriminant_sqrt) / (2 * a);
    m = new_orig + t1 * new_dir;
    Vector3f n1 = q.conjugate()._transformVector(Vector3f(m.x(), m.y(), 0));
    n1.normalize();

    //2nd interval vs infinite cylinder
    interval.Intersect(Interval(t0, t1, n0, n1));

    if (interval.t0 > interval.t1)
        return false;
    else
    {
        float t = interval.t0;
        Vector3f normal = interval.n0;
        if (t < EPSILON) {
            t = interval.t1;
            normal = interval.n1;
            if (t < EPSILON)
                return false;
        }
        info = Intersection{ this, t, ray.Eval(t), normal };
    }

    return true;
}

float Cylinder::Distance(const Vector3f& point) const
{
    Vector3f a = base;
    Vector3f b = a + axis;
    Vector3f ap = point - a;
    Vector3f ab = b - a;
    float t = ap.dot(ab) / dot2(ab);
    Vector3f c = a + t * ab;
    float d = (point - c).norm() - radius;

    float y = (std::abs(t - 0.5f) - 0.5f) * ab.norm();
    float e = Vector2f(Vector2f(d, y).array().max(Vector2f(0, 0).array())).norm();
    float i = std::min(std::max(d, y), 0.f);
    return e + i;
}

Triangle::Triangle(const std::vector<VertexData>& vertices, const TriData& indices,
    const Material& mat) : Shape(mat), vertices(vertices), indices(indices) {

    Vector3f v0 = vertices[indices.x()].pnt;
    Vector3f v1 = vertices[indices.y()].pnt;
    Vector3f v2 = vertices[indices.z()].pnt;

    Vector3f min;
    min.x() = std::min(v0.x(), std::min(v1.x(), v2.x()));
    min.y() = std::min(v0.y(), std::min(v1.y(), v2.y()));
    min.z() = std::min(v0.z(), std::min(v1.z(), v2.z()));

    Vector3f max;
    max.x() = std::max(v0.x(), std::max(v1.x(), v2.x()));
    max.y() = std::max(v0.y(), std::max(v1.y(), v2.y()));
    max.z() = std::max(v0.z(), std::max(v1.z(), v2.z()));

    bbox = Bbox(min, max);
}

bool Triangle::Intersect(const Ray& ray, Intersection& info)
{
    Vector3f e1 = vertices[indices.y()].pnt - vertices[indices.x()].pnt;
    Vector3f e2 = vertices[indices.z()].pnt - vertices[indices.x()].pnt;
    Vector3f p = ray.direction.cross(e2);
    float d = p.dot(e1);

    if (d < 0)
        return false; // Ray is parallel to triangle

    Vector3f s = ray.origin - vertices[indices.x()].pnt;
    float u = p.dot(s) / d;

    if (u < 0 || u > 1)
        return false; //Ray intersects plane but outside e2 edge

    Vector3f q = s.cross(e1);
    float v = ray.direction.dot(q) / d;

    if (v < 0 || (u + v) > 1)
        return false; //Ray intersects plane but outside e1 edge

    float t = e2.dot(q) / d;
    if (t < EPSILON)
        return false; // Ray's negative half intersects triangle

    Vector3f normal = (1 - u - v) * vertices[indices.x()].nrm +
        u * vertices[indices.y()].nrm + v * vertices[indices.z()].nrm;

    normal.normalize();
    info = Intersection{this, t, ray.Eval(t), normal};

    return true;
}

float Triangle::Distance(const Vector3f& point) const
{
    Vector3f a = vertices[indices.x()].pnt;
    Vector3f b = vertices[indices.y()].pnt;
    Vector3f c = vertices[indices.z()].pnt;

    Vector3f ba = b - a;
    Vector3f cb = c - b;
    Vector3f ac = a - c;
    Vector3f pa = point - a;
    Vector3f pb = point - b;
    Vector3f pc = point - c;

    Vector3f normal = ba.cross(ac);

    return std::sqrt((
        sign(pa.dot(ba.cross(normal))) +
        sign(pb.dot(cb.cross(normal))) +
        sign(pc.dot(ac.cross(normal))))
        ?
        std::min(std::min(
        dot2(ba * std::clamp(ba.dot(pa) / dot2(ba), 0.f, 1.f) - pa),
        dot2(cb * std::clamp(cb.dot(pb) / dot2(cb), 0.f, 1.f) - pb)),
        dot2(ac * std::clamp(ac.dot(pc) / dot2(ac), 0.f, 1.f) - pc)
        )
        :
        pa.dot(normal) * pa.dot(normal) / dot2(normal)
        );
}

RaymarchShape::~RaymarchShape()
{
    delete a;
    delete b;
}

bool RaymarchShape::Intersect(const Ray& ray, Intersection& info)
{
    float h = 0.001f;
    float t = h;

    int step = 0;
    while (true) {
        //too many steps - give up
        if (step > 2500) return false;

        Vector3f P = ray.origin + t * ray.direction;
        float dist = Distance(P);
        t = t + std::abs(dist);

        //found intersection
        if (std::abs(dist) < 0.000001f) break;

        //limit t
        if (t > 10000.f) return false;

        step++;
    }

    //avoid self intersection
    if (t < h) return false;

    info.t = t;
    info.pos = ray.origin + t * ray.direction;
    info.shape_hit = this;
    
    //normal
    float nx = Distance(Vector3f(info.pos.x() + h, info.pos.y(), info.pos.z())) -
        Distance(Vector3f(info.pos.x() - h, info.pos.y(), info.pos.z()));
    float ny = Distance(Vector3f(info.pos.x(), info.pos.y() + h, info.pos.z())) -
        Distance(Vector3f(info.pos.x(), info.pos.y() - h, info.pos.z()));
    float nz = Distance(Vector3f(info.pos.x(), info.pos.y(), info.pos.z() + h)) -
        Distance(Vector3f(info.pos.x(), info.pos.y(), info.pos.z() - h));
    info.normal = Vector3f(nx, ny, nz).normalized();

    return true;
}

Union::Union(Shape* A, Shape* B, const Material& mat)
    : RaymarchShape(A, B, mat) {
    Vector3f amin = a->bbox.min();
    Vector3f amax = a->bbox.max();
    Vector3f bmin = b->bbox.min();
    Vector3f bmax = b->bbox.max();

    Vector3f min = amin.array().min(bmin.array());
    Vector3f max = amax.array().max(bmax.array());
    bbox = Bbox(min, max);
}

float Union::Distance(const Vector3f& point) const
{
    return std::min(a->Distance(point), b->Distance(point));
}

Intersect::Intersect(Shape* A, Shape* B, const Material& mat)
    : RaymarchShape(A, B, mat) {
    Vector3f amin = a->bbox.min();
    Vector3f amax = a->bbox.max();
    Vector3f bmin = b->bbox.min();
    Vector3f bmax = b->bbox.max();

    Vector3f min = amin.array().max(bmin.array());
    Vector3f max = amax.array().min(bmax.array());
    bbox = Bbox(min, max);
}

float Intersect::Distance(const Vector3f& point) const
{
    return std::max(a->Distance(point), b->Distance(point));
}

Difference::Difference(Shape* A, Shape* B, const Material& mat)
    : RaymarchShape(A, B, mat) {
    bbox = a->bbox;
}

float Difference::Distance(const Vector3f& point) const
{
    return std::max(a->Distance(point), -b->Distance(point));
}
