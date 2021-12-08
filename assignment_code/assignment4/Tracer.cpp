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

        kernel_ = {1, 4, 7, 4, 1};
        //kernel_ = {0, 0, 1, 0, 0};

        /// Goal is to estimate the integral of the light transport function
        /// The Monte Carlo Estimator will have expectation equal to the integral,
        /// Use N_ trials

        /// Considering the solid angles, the total hemisphere surface area is 2pi,
        /// and we want a uniform distribution over it -> sample_pdf_ = 1/2pi
        Image image(image_size_.x, image_size_.y);
        Image preImage(image_size_.x, image_size_.y);
        std::vector<std::vector<int>> pixelPlane(image_size_.x, std::vector<int>(image_size_.y));

        float focusLower = 0, focusUpper = 12;
        // For each pixel, cast a ray, and update its value in the image.

        for (size_t y = 0; y < image_size_.y; y++) {
            for (size_t x = 0; x < image_size_.x; x++) {
                // Transforms the pixel to [-1, 1] x [-1, 1]
                float x_coord = 2.f * x / image_size_.x - 1.f;
                float y_coord = 2.f * y / image_size_.y - 1.f;

                Ray ray = camera_.GenerateRay(glm::vec2(x_coord, y_coord));
                glm::vec3 estimated_color(0);
                glm::vec3 hit_pos;
                for (int trial = 0; trial < N_; trial++) {
                    HitRecord record;
                    glm::vec3 color = TraceRay(ray, max_bounces_, record);
                    estimated_color += color;
                    hit_pos = ray.At(record.time);
                }
                estimated_color /= (float) N_ * sample_pdf_;

                preImage.SetPixel(x, y, estimated_color);
                float dist = glm::distance(camera_.center_, hit_pos);
                pixelPlane[x][y] = focusLower <= dist && dist <= focusUpper ? 1 :
                                                         (dist < focusLower ? 0 : 2);
            }
        }
        if (output_file.size()) {
            preImage.SavePNG("PreImage" + output_file);
            BlurImage(image, preImage, pixelPlane);
            image.SavePNG(output_file);
        }
    }

    void Tracer::BlurImage(Image &resImage, Image &preImage, std::vector<std::vector<int>> &pixelPlane) {
        /// Gaussian Blur, clever two passes to reduce a factor of k (kernel size)
        size_t offset = kernel_.size() / 2; // radius basically
        Image intermImage(image_size_.x, image_size_.y);

        for (size_t y = 0; y < image_size_.y; y++) {
            for (size_t x = 0; x < image_size_.x; x++) {
                if (pixelPlane[x][y] == 1) {
                        intermImage.SetPixel(x, y, preImage.GetPixel(x, y));
                } else {
                    glm::vec3 contrib(0);
                    float totalWeight = 0;
                    for (size_t k = 0; k < kernel_.size(); k++) {
                        size_t x2 = x + k - offset;
                        if (0 <= x2 && x2 < image_size_.x && pixelPlane[x2][y] == pixelPlane[x][y]) {
                            //std::cout << glm::to_string(preImage.GetPixel(x2, y)) << "\n";

                            contrib += kernel_[k] * preImage.GetPixel(x2, y);
                            totalWeight += kernel_[k];
                        }
                    }
                    //std::cout << glm::to_string(contrib / totalWeight) << " vs " << glm::to_string(preImage.GetPixel(x, y)) << "\n";
                    intermImage.SetPixel(x, y, contrib / totalWeight);
                }
            }
        }
        intermImage.SavePNG("intermediateImage.png");
        for (size_t y = 0; y < image_size_.y; y++) {
            for (size_t x = 0; x < image_size_.x; x++) {
                if (pixelPlane[x][y] == 1) {
                    resImage.SetPixel(x, y, intermImage.GetPixel(x, y));
                } else {
                    glm::vec3 contrib(0);
                    float totalWeight = 0;
                    for (size_t k = 0; k < kernel_.size(); k++) {
                        size_t y2 = y + k - offset;
                        if (0 <= y2 && y2 < image_size_.y && pixelPlane[x][y2] == pixelPlane[x][y]) {
                            contrib += kernel_[k] * intermImage.GetPixel(x, y2); // Apply along the OTHER direction
                            totalWeight += kernel_[k];
                        }
                    }
                    resImage.SetPixel(x, y, contrib / totalWeight);
                }
            }
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

    glm::vec3 Tracer::TraceRay(const Ray &ray, size_t bounces_left, HitRecord &record) const {
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
                    // glm::vec3 specular_amt(0.0f);
                    // if (hit_comp == 0){
                    //     specular_amt = reflected_eye;
                    // } else {
                    //     /// Get the amount of light proportional to how close it is the the specular lobe
                    //     float clamped_lobe_cosine = glm::max(0.f, glm::dot(reflected_eye, dir_to_light)); // both have magnitude 1
                    //     specular_amt = (float) glm::pow(clamped_lobe_cosine, obj_material.GetShininess()) * light_intensity;
                    // }
                    // /// Get the amount of light proportional to how close it is the the specular lobe
                    float clamped_lobe_cosine = glm::max(0.f, glm::dot(reflected_eye, dir_to_light)); // both have magnitude 1
                    glm::vec3 specular_amt = (float) glm::pow(clamped_lobe_cosine, obj_material.GetShininess()) * light_intensity;
                    specular_contrib += specular_amt;
                }
            }
        }
        /// For each, we elementwise scale the object's ambient color
        ambient_contrib *= obj_material.GetDiffuseColor(); /// KEY BUG, not AmbientColor!
        diffuse_contrib *= obj_material.GetDiffuseColor();
        specular_contrib *= obj_material.GetSpecularColor();

        glm::vec3 indirect_specular(0);
        glm::vec3 indirect_diffuse(0);
        /// When we add reflection, we allow the ray to move in the new direction
        if (bounces_left > 0 && hit_comp <= 1) {
            Ray reflected_ray(hit_pos + reflected_eye * epsilon_, reflected_eye);
            HitRecord specular_record = HitRecord();

            if (hit_comp == 0){
                indirect_specular = 0.8f * TraceRay(reflected_ray, bounces_left - 1, specular_record);
                // indirect_specular = glm::vec3(0.0f);
                specular_contrib = glm::vec3(0.0f);
            } else {
                indirect_specular = TraceRay(reflected_ray, bounces_left - 1, specular_record) * obj_material.GetSpecularColor();
            }
            // indirect_specular = TraceRay(reflected_ray, bounces_left - 1, specular_record) * obj_material.GetSpecularColor();

            /// Also check random directions in hemisphere
            HitRecord diffuse_record;
            Ray sampled_ray = sampleHemisphereRay(hit_pos, surface_normal);
            float lambert_cosine = glm::dot(sampled_ray.GetDirection(), surface_normal);
            //std::cout << "Product" << glm::to_string(lambert_cosine * obj_material.GetDiffuseColor()) << "\n";
            glm::vec3 incoming = TraceRay(sampled_ray, bounces_left - 1, diffuse_record);
            indirect_diffuse = incoming * lambert_cosine * 0.25f; /// not sure if this is albedo?
            //std::cout << glm::to_string(incoming) << " " << lambert_cosine << " " << glm::to_string(obj_material.GetDiffuseColor()) << "\n";
            // std::cout << "Hit_Comp: " << hit_comp << " " << "Indirect_specular: " << glm::to_string(indirect_specular) << " " << "Specular_contrib: " << glm::to_string(specular_contrib) << "\n";

        }
        // cancel the
        //std::cout << glm::to_string(diffuse_contrib) << " vs " << glm::to_string(indirect_diffuse) << "\n";
        //return diffuse_contrib + indirect_diffuse;
        // return (ambient_contrib + diffuse_contrib) + indirect_diffuse;
        return (ambient_contrib + diffuse_contrib + indirect_specular + specular_contrib) + indirect_diffuse;
        // return (ambient_contrib + diffuse_contrib + specular_contrib) + indirect_diffuse;

    }

    Ray Tracer::sampleHemisphereRay(glm::vec3 &point, glm::vec3 &normal) const {
        //std::default_random_engine generator_(time(nullptr));
        //std::uniform_int_distribution<float> uniform_distribution_(0, 1); /// Not sure how to make it const globally outside?
        float r1 = rand() / (RAND_MAX + 1.f);// uniform_distribution_(generator_);
        float r2 = rand() / (RAND_MAX + 1.f); // uniform_distribution_(generator_);
        glm::vec3 sampledRay = sampleHemisphere(r1, r2);
        if (glm::dot(sampledRay, normal) < 0) {
            sampledRay -= 2 * glm::dot(sampledRay, normal) * normal;
        }
        return Ray(point + epsilon_ * sampledRay, sampledRay);
    }

    glm::mat3 Tracer::localPlaneToWorld(glm::vec3 &point, glm::vec3 &normal) const {
        /// Transform a ray in a local coordinate space to world space using
        /// the plane defined by normal and point.
        /// Pick an arbitrary 2nd vector and get the 3rd from cross product
        glm::vec3 nx = glm::normalize(glm::vec3(normal.z, 0, -normal.x)); // we guarantee n1·normal = 0
        glm::vec3 nz = glm::cross(nx, normal); // x cross y is z
        return {nx, normal, nz}; // use the normal as y axis
    }

    glm::vec3 Tracer::sampleHemisphere(float r1, float r2) const {
        /// New method: just pick phi and theta
        float theta = 2 * M_PI * r1;
        float phi = acos(2 * r2 - 1);
        float x = sin(phi) * cos(theta);
        float y = sin(phi) * sin(theta);
        float z = cos(phi);
        return glm::vec3(x, y, z);
    }

    glm::vec3 Tracer::GetBackgroundColor(const glm::vec3 &direction) const {
        if (cube_map_ != nullptr) {
            return cube_map_->GetTexel(direction);
        } else
            return background_color_;
    }
}  // namespace GLOO
