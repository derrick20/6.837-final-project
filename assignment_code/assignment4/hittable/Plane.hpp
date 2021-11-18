#ifndef PLANE_H_
#define PLANE_H_

#include "HittableBase.hpp"

namespace GLOO {
    class Plane : public HittableBase {
    public:
        Plane(const glm::vec3 &normal, float d) : normal(normal), d(d) {}

        float IntersectTime(const Ray &ray) const;

        bool Intersect(const Ray &ray, float t_min, HitRecord &record) const override;

    private:
        glm::vec3 normal;
        float d;
    };
}  // namespace GLOO

#endif
