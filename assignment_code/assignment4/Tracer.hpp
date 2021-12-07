#ifndef TRACER_H_
#define TRACER_H_

#include <random>
#include "gloo/Scene.hpp"
#include "gloo/Material.hpp"
#include "gloo/lights/LightBase.hpp"
#include "gloo/components/LightComponent.hpp"

#include "Ray.hpp"
#include "HitRecord.hpp"
#include "TracingComponent.hpp"
#include "CubeMap.hpp"
#include "PerspectiveCamera.hpp"

namespace GLOO {
    class Tracer {
    public:
        Tracer(const CameraSpec &camera_spec,
               const glm::ivec2 &image_size,
               size_t max_bounces,
               const glm::vec3 &background_color,
               const CubeMap *cube_map,
               bool shadows_enabled)
                : camera_(camera_spec),
                  image_size_(image_size),
                  max_bounces_(max_bounces),
                  background_color_(background_color),
                  cube_map_(cube_map),
                  shadows_enabled_(shadows_enabled),
                  scene_ptr_(nullptr) {
        }

        void Render(const Scene &scene, const std::string &output_file);

    private:
        glm::vec3 TraceRay(const Ray &ray, size_t bounces_left, HitRecord &record) const;

        int RayIntersection(const Ray &ray, HitRecord &record) const;

        glm::vec3 GetBackgroundColor(const glm::vec3 &direction) const;

        Ray sampleHemisphereRay(glm::vec3 &point, glm::vec3 &normal) const;

        glm::vec3 sampleHemisphere(float r1, float r2) const;

        glm::mat3 localPlaneToWorld(glm::vec3 &point, glm::vec3 &normal) const;

        PerspectiveCamera camera_;
        glm::ivec2 image_size_;

        size_t max_bounces_;
        std::vector<TracingComponent *> tracing_components_;
        std::vector<LightComponent *> light_components_;
        glm::vec3 background_color_;
        const CubeMap *cube_map_;
        const float epsilon_ = 1e-2;

        bool shadows_enabled_;
        int N_ = 1;
        float sample_pdf_ = 1.f / (2.f * M_PI);

        const Scene *scene_ptr_;

        void BlurImage(Image &resImage, Image &preImage, std::vector<std::vector<int>> &pixelPlane);

        std::vector<float> kernel_;
    };
}  // namespace GLOO

#endif
