#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"
#include "vec3.h"
#include "interval.h"

class sphere : public hittable {
    public:
    sphere(const point3& center, double radius, shared_ptr<material> mat) : center(center), radius(std::fmax(0,radius)), mat(mat) {}

    bool hit(const ray& ray, interval ray_t, hit_record& rec) const override{
            vec3 oc = center - ray.origin();
            double a = ray.direction().length_squared();
            double h = dot(ray.direction(), oc);
            double c = oc.length_squared() - radius * radius;
            double delta = h*h - a*c;
            if (delta < 0)
                return false;

            double sqrt_delta = std::sqrt(delta);
            auto root = (h - sqrt_delta) / a;

            if (!ray_t.surrounds(root)){
                root = (h + sqrt_delta) / a;
                if (!ray_t.surrounds(root)) {
                    return false;
                }
            }

            rec.p = ray.at(root);
            rec.t = root;
            rec.set_face_normal(ray, unit_vector(rec.p - center));
            rec.mat = mat;
            return true;
    }

    private:
    point3 center;
    double radius;
    shared_ptr<material> mat;
};

#endif