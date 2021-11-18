#include "Tracer.hpp"

#include <glm/gtx/string_cast.hpp>
#include <stdexcept>
#include <algorithm>

#include "gloo/Transform.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/lights/AmbientLight.hpp"

#include "gloo/Image.hpp"
#include "Illuminator.hpp"
#include "HitRecord.hpp"

namespace GLOO {
    void Tracer::Render(const Scene &scene, const std::string &output_file) {
        scene_ptr_ = &scene;

        auto &root = scene_ptr_->GetRootNode();
        tracing_components_ = root.GetComponentPtrsInChildren<TracingComponent>();
        light_components_ = root.GetComponentPtrsInChildren<LightComponent>();

        Image image(image_size_.x, image_size_.y);
        for (size_t y = 0; y < image_size_.y; y++) {
            for (size_t x = 0; x < image_size_.x; x++) {
                // Transforms the pixel to [-1, 1] x [-1, 1]
                float x_coord = 2.f * x / image_size_.x - 1.f;
                float y_coord = 2.f * y / image_size_.y - 1.f;
                // TODO: For each pixel, cast a ray, and update its value in the image.
                Ray ray = camera_.GenerateRay(glm::vec2(x_coord, y_coord));
                HitRecord record;
                glm::vec3 color = TraceRay(ray, max_bounces_, record);
                image.SetPixel(x, y, color);
            }
        }
        if (output_file.size()) {
            image.SavePNG(output_file);
        }
    }

    int Tracer::RayIntersection(const Ray &ray, HitRecord &record) const {
        int hit_comp = -1;
        for (int i = 0; i < tracing_components_.size(); i++) {
            auto &comp = tracing_components_[i];
            glm::mat4 world_to_local = glm::inverse(comp->GetNodePtr()->GetTransform().GetLocalToWorldMatrix());
            Ray new_ray = ray;
            new_ray.ApplyTransform(world_to_local);
            if (comp->GetHittable().Intersect(new_ray, 0, record)) {
                // We updated our earliest hit still above lower bound hit time
                // We need to fix the normal by sending it back to za world
                /// KEY BUG: vectors should have w = 0!!!!!!
                /// 2nd, evil evil bug: need to normalize when it's been converted back to a 3D vector
                record.normal = glm::normalize(glm::vec3(glm::transpose(world_to_local) * glm::vec4(record.normal, 0)));
                hit_comp = i;
            }
        }
        return hit_comp;
    }

    glm::vec3 Tracer::TraceRay(const Ray &ray, size_t bounces, HitRecord &record) const {
        /// We first find the object first hit by the ray of our eye
        int hit_comp = RayIntersection(ray, record);
        if (hit_comp == -1) {
            return GetBackgroundColor(ray.GetDirection());
        }
        auto &comp = tracing_components_[hit_comp];
        Material &obj_material = comp->GetNodePtr()->GetComponentPtr<MaterialComponent>()->GetMaterial();
        glm::vec3 ambient_contrib(0), diffuse_contrib(0), specular_contrib(0);

        glm::vec3 hit_pos = ray.At(record.time);
        glm::vec3 surface_normal = glm::normalize(record.normal); // assumed to be normalized
        glm::vec3 eye_ray = glm::normalize(ray.GetDirection());
        /// If we are going towards the surface, this will flip us from that direction
        /// and move us to be reflected
        glm::vec3 reflected_eye = glm::normalize(eye_ray - 2.f * glm::dot(eye_ray, surface_normal) * surface_normal);

        /// Now, we consider going from the point hit to all light sources
        for (auto &comp : light_components_) {
            LightBase *light_ptr = comp->GetLightPtr();
            if (comp->GetLightPtr()->GetType() == LightType::Ambient) {
                auto ambient_light_ptr = static_cast<AmbientLight *>(light_ptr);
                /// We simply collect all light intensities (colors)
                /// We always do this because ambient light supposedly can enter from any direction
                ambient_contrib += ambient_light_ptr->GetAmbientColor();
                //std::cout << glm::to_string(ambient_contrib) << "\n";
            } else {
                glm::vec3 dir_to_light, light_intensity;
                float dist_to_light;
                Illuminator::GetIllumination(*comp, hit_pos, dir_to_light, light_intensity, dist_to_light);
                bool shadowed = false;
                if (shadows_enabled_) {
                    /// Check for shadows to the light source.
                    /// If tracing from hit point to light source has an intersection,
                    /// we can't get any light here.
                    Ray light_ray(hit_pos + epsilon_ * dir_to_light, dir_to_light);
                    HitRecord shadow_record;
                    int blocking_comp = RayIntersection(light_ray, shadow_record);
                    if (blocking_comp != -1) {
                        float block_dist = glm::length(light_ray.At(shadow_record.time) - hit_pos); /// KEY BUG: needed to use the right record time!
                        if (block_dist < dist_to_light) {
                            shadowed = true;
                        }
                    }
                }
                if (!shadowed) {
                    /// Get the amount of light based on how directly the light hits it
                    /// (imagine the surface will scatter the result in all directions)
                    float clamped_lambert = glm::max(0.f, glm::dot(surface_normal, dir_to_light));
                    diffuse_contrib += clamped_lambert * light_intensity;

                    /// Get the amount of light proportional to how close it is the the specular lobe
                    float clamped_lobe_cosine = glm::max(0.f, glm::dot(reflected_eye, dir_to_light)); // both have magnitude 1
                    glm::vec3 specular_amt = (float) glm::pow(clamped_lobe_cosine, obj_material.GetShininess()) * light_intensity;
                    specular_contrib += specular_amt;
                    //std::cout << glm::to_string(specular_amt * obj_material.GetSpecularColor()) << "\n";
                }
            }
        }
        /// For each, we elementwise scale the object's ambient color
        ambient_contrib *= obj_material.GetDiffuseColor(); /// KEY BUG, not AmbientColor!
        diffuse_contrib *= obj_material.GetDiffuseColor();
        specular_contrib *= obj_material.GetSpecularColor();

        glm::vec3 indirect_lighting(0);
        /// When we add reflection, we allow the ray to move in the new direction
        if (bounces > 0) {
            Ray reflected_ray(hit_pos + reflected_eye * epsilon_, reflected_eye);
            HitRecord new_record = HitRecord();
            indirect_lighting = TraceRay(reflected_ray, bounces - 1, new_record) * obj_material.GetSpecularColor();
            /// TODO: Also check random directions in hemisphere
            ///
        }
        return ambient_contrib + diffuse_contrib + specular_contrib + indirect_lighting;
    }


    glm::vec3 Tracer::GetBackgroundColor(const glm::vec3 &direction) const {
        if (cube_map_ != nullptr) {
            return cube_map_->GetTexel(direction);
        } else
            return background_color_;
    }
}  // namespace GLOO
