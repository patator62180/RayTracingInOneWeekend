#ifndef CAMERA_H
#define CAMERA_H

#include "rtweekend.h"
#include "hittable.h"
#include "material.h"
#include <thread>
#include <chrono>

class camera {
    public:
        double aspect_ratio = 16.0 / 9.0;
        int image_width = 400;
        int samples_per_pixel = 1000;
        int max_depth = 10;
        color background;

        double vfov = 20;
        point3 lookfrom = point3(-2,2,1);
        point3 lookat = point3(0,0,-1);
        vec3 vup = vec3(0,1,0);

        double defocus_angle = 10.0;
        double focus_dist = 3.4;
        std::vector<color> color_buffer;
        int lines_computed;

        void render_section(const hittable& world, int start, int end) {
            for(int j = start; j < end; ++j){
                for (int i = 0; i < image_width; ++i){
                    color pixel_color(0,0,0);
                    for (int sample = 0; sample < samples_per_pixel; ++sample ) {
                        ray ray = get_ray(i, j);
                        pixel_color += ray_color(ray, max_depth, world);
                    }
                    color_buffer[image_width * j + i] = pixel_samples_scale * pixel_color;
                }
                    ++lines_computed;
                    std::clog << "\rProgress: " << lines_computed << " / " << image_height << " ("<< 100*(lines_computed / double(image_height)) << " %)" << std::endl;
            }
        }

        void render(const hittable& world){        
            auto start_time = std::chrono::high_resolution_clock::now();        
            initialize();
            std::cout << "P3\n" << image_width << " " << image_height << "\n255\n";

            int num_threads = std::thread::hardware_concurrency();
            int section_height = image_height / num_threads;
            std::vector<std::thread> threads;

            for (int t = 0; t < num_threads; ++t) {
                int start = t * section_height;
                int end = (t == num_threads - 1) ? image_height : start + section_height;
                threads.push_back(std::thread(&camera::render_section, this, std::ref(world), start, end));
            }

            for (auto& thread : threads) {
                thread.join();
            }

            for(color& c : color_buffer) {
                write_color(std::cout, c);
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
            std::clog << "\rDone in " << duration << " seconds" << std::endl;
        }

    private:
        int image_height;
        double pixel_samples_scale;
        vec3 pixel00_loc;
        vec3 pixel_delta_u;
        vec3 pixel_delta_v;
        vec3 center;
        vec3 u,v,w;
        vec3 defocus_disk_u;
        vec3 defocus_disk_v;

        void initialize() {
            image_height = int(image_width / aspect_ratio);
            image_height = (image_height < 1) ? 1 : image_height;

            pixel_samples_scale = 1.0 / samples_per_pixel;

            auto theta = degrees_to_radians(vfov);
            auto h = std::tan(theta/2);
            auto viewport_height = 2 * h * focus_dist;
            auto viewport_width = viewport_height * (double(image_width)/image_height);
            center = lookfrom;

            w = unit_vector(lookfrom - lookat);
            u = unit_vector(cross(vup, w));
            v = cross(w, u);

            auto viewport_u = viewport_width * u;
            auto viewport_v = viewport_height * -v;

            pixel_delta_u = viewport_u/image_width;
            pixel_delta_v = viewport_v/image_height;

            auto viewport_upper_left = center - (focus_dist*w) - viewport_u/2 - viewport_v/2;
            pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

            auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
            defocus_disk_u = u * defocus_radius;
            defocus_disk_v = v * defocus_radius;

            color_buffer.resize(image_width * image_height);
            lines_computed = 0;
        }

        color ray_color(const ray& r, int depth, const hittable& world) const {
            if(depth <= 0)
                return color();

            hit_record rec;

            if(!world.hit(r, interval(0.001, infinity), rec))
                return background;

            ray scattered;
            color attenuation;
            color color_from_emission = rec.mat->emitted(rec.u, rec.v, rec.p);

            if(!rec.mat->scatter(r, rec, attenuation, scattered))
                return color_from_emission;

            color color_from_scatter = attenuation * ray_color( scattered, depth-1, world);

            return color_from_emission + color_from_scatter;
        }

        ray get_ray(int i, int j) const {
            auto offset = sample_square();
            auto pixel_sample = pixel00_loc + ((i + offset.x()) * pixel_delta_u) + ((j + offset.y()) * pixel_delta_v);

            auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
            auto ray_direction = pixel_sample - ray_origin;
            auto ray_time = random_double();

            return ray(ray_origin, ray_direction, ray_time);
        }

        vec3 sample_square() const {
            return vec3(random_double() - 0.5, random_double() - 0.5, 0);
        }

        point3 defocus_disk_sample() const {
            auto p = random_in_unit_disk();
            return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
        }
};

#endif