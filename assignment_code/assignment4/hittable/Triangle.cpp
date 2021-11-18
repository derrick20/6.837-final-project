#include "Triangle.hpp"

#include <iostream>
#include <stdexcept>

#include <glm/common.hpp>
#include <glm/gtx/string_cast.hpp>

#include "Plane.hpp"

namespace GLOO {
    Triangle::Triangle(const glm::vec3 &p0,
                       const glm::vec3 &p1,
                       const glm::vec3 &p2,
                       const glm::vec3 &n0,
                       const glm::vec3 &n1,
                       const glm::vec3 &n2) {
        positions_ = {p0, p1, p2};
        normals_ = {n0, n1, n2};
    }

    Triangle::Triangle(const std::vector<glm::vec3> &positions,
                       const std::vector<glm::vec3> &normals) {
        positions_ = positions;
        normals_ = normals;
    }

    bool Triangle::Intersect(const Ray &ray, float t_min, HitRecord &record) const {
        /// Compute barycentric coordinates.
        /// System of equations combining the parametric and barycentric representations
        /// Since it's column major, A[0] will be the column 0, etc.
        glm::mat3 A(positions_[1] - positions_[0], positions_[2] - positions_[0], -ray.GetDirection());
        glm::vec3 b = ray.GetOrigin() - positions_[0];
        glm::vec3 x = glm::inverse(A) * b;
        float t_int = x[2];
        glm::vec3 bary_coords(1.f - x[0] - x[1], x[0], x[1]);

        if (t_min < t_int && t_int < record.time) {
            for (int i = 0; i < 3; i++) {
                if (!(0 <= bary_coords[i] && bary_coords[i] <= 1)) {
                    return false;
                }
            }
            glm::vec3 interpolated_normal(0);
            for (int i = 0; i < 3; i++) {
                interpolated_normal += bary_coords[i] * normals_[i];
            }
            record.time = t_int;
            record.normal = glm::normalize(interpolated_normal); /// HAVE TO NORMALIZE
            return true;
        }
        return false;
    }
}  // namespace GLOO
