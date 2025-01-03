#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"
#include "vec3.h"
#include "interval.h"

class sphere : public hittable {
    public:
    sphere(const point3& static_center, double radius, shared_ptr<material> mat) : center(static_center, vec3(0, 0, 0)), radius(std::fmax(0, radius)), mat(mat) 
    {
        auto rvec = vec3(radius, radius, radius);
        bbox = aabb(static_center - rvec, static_center + rvec);
    }
    
    sphere(const point3& center1, const point3& center2, double radius, shared_ptr<material> mat) : center(center1, center2-center1), radius(std::fmax(0, radius)), mat(mat) 
    {
        auto rvec = vec3(radius, radius, radius);
        aabb bbox1(center.at(0) - rvec, center.at(0) + rvec);
        aabb bbox2(center.at(1) - rvec, center.at(1) + rvec);
        bbox = aabb(bbox1, bbox2);
    }

    bool hit(const ray& ray, interval ray_t, hit_record& rec) const override{
        auto current_center = center.at(ray.time());
        vec3 oc = current_center - ray.origin();
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
        auto outward_normal = unit_vector(rec.p - current_center);
        rec.set_face_normal(ray, outward_normal);
        get_shpere_uv(outward_normal, rec.u, rec.v);
        rec.mat = mat;
        return true;
    }

    aabb bounding_box() const override { return bbox;}

    private:
    ray center;
    double radius;
    shared_ptr<material> mat;
    aabb bbox;

    static void get_shpere_uv(const point3& p, double& u, double& v) {
        auto theta = std::acos(-p.y());
        auto phi = std::atan2(-p.z(), p.x()) + pi;

        u = phi / (2*pi);
        v = theta / pi;
    }
};

#endif