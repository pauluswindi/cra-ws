#include "vision/reconstruction/reconstruction_pipeline.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace vision::reconstruction
{

namespace
{

bool isFinite(const geometry_msgs::msg::Point& point)
{
    return std::isfinite(point.x) &&
           std::isfinite(point.y) &&
           std::isfinite(point.z);
}

bool validCapsule(const human::Capsule& capsule)
{
    return isFinite(capsule.start) &&
           isFinite(capsule.end) &&
           std::isfinite(capsule.radius) &&
           capsule.radius > 0.0;
}

}  // namespace

ReconstructionPipeline::ReconstructionPipeline(
    const ReconstructionConfig& config)
    : config_(config)
{
    if (!std::isfinite(config_.roi_margin) ||
        config_.roi_margin < 0.0 ||
        !std::isfinite(config_.voxel_size) ||
        config_.voxel_size <= 0.0 ||
        !std::isfinite(config_.min_depth) ||
        config_.min_depth <= 0.0 ||
        !std::isfinite(config_.max_depth) ||
        config_.max_depth <= config_.min_depth ||
        !std::isfinite(config_.radius_smoothing_alpha) ||
        config_.radius_smoothing_alpha <= 0.0 ||
        config_.radius_smoothing_alpha > 1.0) {
        throw std::invalid_argument(
            "Invalid reconstruction pipeline configuration");
    }
}

void ReconstructionPipeline::filterRadii(
    std::vector<human::Capsule>& capsules,
    RadiusMap& radii) const
{
    for (auto& capsule : capsules) {
        const RadiusKey key{
            capsule.body_id,
            capsule.segment_id
        };

        auto it = radii.find(key);

        if (it == radii.end()) {
            radii.emplace(key, capsule.radius);
            continue;
        }

        double& previous = it->second;

        if (!capsule.radius_valid) {
            capsule.radius = previous;
            continue;
        }

        if (capsule.radius >= previous) {
            previous = capsule.radius;
        } else {
            previous =
                config_.radius_smoothing_alpha * capsule.radius +
                (1.0 - config_.radius_smoothing_alpha) * previous;
        }

        capsule.radius = previous;
    }
}

bool ReconstructionPipeline::process(
    const sensor_msgs::msg::Image& depth,
    const sensor_msgs::msg::Image& body_index,
    const sensor_msgs::msg::CameraInfo& camera_info,
    const geometry_msgs::msg::TransformStamped& transform,
    const std::map<int, SkeletonData>& skeletons,
    const std::set<int>& active_body_ids,
    bool debug_voxels,
    ReconstructionResult& result)
{
    ReconstructionResult next_result;
    RadiusMap next_radii = filtered_radii_;

    next_result.bodies.reserve(active_body_ids.size());

    for (const int body_id : active_body_ids) {
        const auto skeleton_it = skeletons.find(body_id);

        if (skeleton_it == skeletons.end() ||
            skeleton_it->second.joints.empty()) {
            return false;
        }

        const auto& skeleton = skeleton_it->second;

        const auto roi = roi_generator_.generate(
            skeleton.points,
            config_.roi_margin,
            static_cast<std::uint32_t>(body_id),
            "human");

        if (!roi.valid) {
            return false;
        }

        const auto human_mask =
            human_segmenter_.generateMask(
                body_index,
                static_cast<std::uint32_t>(body_id));

        const auto points =
            depth_processor_.extractPoints(
                depth,
                camera_info,
                transform,
                roi,
                config_.min_depth,
                config_.max_depth,
                &human_mask);

        auto voxels =
            voxel_reconstruction_.reconstruct(
                points,
                config_.voxel_size);

        if (voxels.empty()) {
            return false;
        }

        auto capsules = human_capsule_.generate(
            skeleton.joints,
            voxels,
            static_cast<std::uint32_t>(body_id),
            config_.capsule);

        if (capsules.empty()) {
            return false;
        }

        for (const auto& capsule : capsules) {
            if (!validCapsule(capsule)) {
                return false;
            }
        }

        filterRadii(capsules, next_radii);

        BodyReconstruction body;
        body.body_id = static_cast<std::uint32_t>(body_id);
        body.capsules = std::move(capsules);

        if (debug_voxels) {
            body.coverage = human_capsule_.evaluateCoverage(
                body.capsules,
                voxels,
                config_.voxel_size);

            body.voxels = std::move(voxels);
        }

        next_result.bodies.push_back(std::move(body));
    }

    if (next_result.bodies.empty()) {
        return false;
    }

    for (auto it = next_radii.begin();
         it != next_radii.end();) {
        if (active_body_ids.count(
                static_cast<int>(it->first.first)) == 0) {
            it = next_radii.erase(it);
        } else {
            ++it;
        }
    }

    filtered_radii_ = std::move(next_radii);
    result = std::move(next_result);

    return true;
}

void ReconstructionPipeline::reset()
{
    filtered_radii_.clear();
}

}  // namespace vision::reconstruction