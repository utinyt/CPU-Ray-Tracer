#ifndef MINIMIZER_H
#define MINIMIZER_H

class Minimizer {
public:
    typedef float Scalar;
    Ray ray;
    Intersection closest_info;

    Minimizer(const Ray& ray) : ray(ray) {}

    float minimumOnObject(Shape* shape) {
        Intersection current_info;
        if (shape->Intersect(ray, current_info)) {
            if (current_info.t < closest_info.t) {
                closest_info = current_info;
            }
        }
        return closest_info.t;
    }

    float minimumOnVolume(const Bbox& box) {
        Vector3f base = box.min();
        Vector3f diag = box.max() - base;

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
            return FINFINITY;
        else
        {
            float t = interval.t0;
            Vector3f normal = interval.n0;
            if (t < 0) {
                t = interval.t1;
                normal = interval.n1;
                if (t < 0)
                    return FINFINITY;
                else
                    return 0;
            }
            else
                return t;
        }
    }
};

Bbox bounding_box(const Shape* obj) {
    return obj->bbox;
}

#endif
