#include "Plane.hpp"

namespace GLOO {
    float Plane::IntersectTime(const Ray &ray) const {
        glm::vec3 dir = ray.GetDirection();
        float denom = glm::dot(dir, normal);
        // Oops, wrong logic: P(t) = Ro + tRd
        // Plug into n·P = d
        // t = (d - n·Ro) / (n·Rd)
        return denom == 0.f ? std::numeric_limits<float>::max() : (d - glm::dot(ray.GetOrigin(), normal)) / denom;
    }

    bool Plane::Intersect(const Ray &ray, float t_min, HitRecord &record) const {
        // TODO: Implement ray-plane intersection.
        float t_int = IntersectTime(ray);
        if (t_min < t_int && t_int < record.time) {
            record.time = t_int;
            record.normal = normal;
            return true;
        }
        return false;
    }
}  // namespace GLOO
