#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "vision/human/human_capsule.hpp"
#include "vision/reconstruction/depth_processor.hpp"
#include "vision/reconstruction/roi_generator.hpp"
#include "vision/reconstruction/voxel_reconstruction.hpp"
#include "vision/segmentation/human_segmenter.hpp"

namespace vision::reconstruction
{

struct SkeletonData
{
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<human::BodyJoint> joints;
};

struct ReconstructionConfig
{
    double roi_margin = 0.15;
    double voxel_size = 0.02;

    double min_depth = 0.3;
    double max_depth = 3.0;

    double radius_smoothing_alpha = 0.2;

    human::CapsuleConfig capsule;
};

struct BodyReconstruction
{
    std::uint32_t body_id = 0;

    std::vector<human::Capsule> capsules;

    // Populated only when debugging is enabled.
    std::vector<Voxel> voxels;

    human::CoverageResult coverage;
};

struct ReconstructionResult
{
    std::vector<BodyReconstruction> bodies;
};

class ReconstructionPipeline
{
public:
    explicit ReconstructionPipeline(
        const ReconstructionConfig& config);

    bool process(
        const sensor_msgs::msg::Image& depth,
        const sensor_msgs::msg::Image& body_index,
        const sensor_msgs::msg::CameraInfo& camera_info,
        const geometry_msgs::msg::TransformStamped& transform,
        const std::map<int, SkeletonData>& skeletons,
        const std::set<int>& active_body_ids,
        bool debug_voxels,
        ReconstructionResult& result);

    void reset();

private:
    using RadiusKey = std::pair<std::uint32_t, int>;
    using RadiusMap = std::map<RadiusKey, double>;

    void filterRadii(
        std::vector<human::Capsule>& capsules,
        RadiusMap& radii) const;

    ReconstructionConfig config_;

    ROIGenerator roi_generator_;
    DepthProcessor depth_processor_;
    VoxelReconstruction voxel_reconstruction_;

    segmentation::HumanSegmenter human_segmenter_;

    human::HumanCapsule human_capsule_;

    RadiusMap filtered_radii_;
};

}  // namespace vision::reconstruction