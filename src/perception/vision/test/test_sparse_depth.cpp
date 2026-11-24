#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "vision_fixture.hpp"

using vision::reconstruction::DepthProcessor;
using vision::segmentation::HumanSegmenter;
using vision_test::Fixture;

namespace
{

void expectSamePoints(Fixture& fixture, std::uint32_t body = 1)
{
    HumanSegmenter segmenter;
    const auto mask = segmenter.generateMask(fixture.body, body);
    const auto pixels = segmenter.generatePixelRuns(fixture.body, body);
    std::vector<std::size_t> expected;
    for (std::size_t i = 0; i < mask.size(); ++i) {
        if (mask[i]) { expected.push_back(i); }
    }
    std::vector<std::size_t> expanded;
    for (const auto& run : pixels) {
        for (auto i = run.begin; i < run.end; ++i) { expanded.push_back(i); }
    }
    ASSERT_EQ(expanded, expected);
    DepthProcessor processor;
    const auto full = processor.extractPoints(fixture.depth, fixture.info, fixture.transform,
                                              fixture.roi, .3, 3., &mask);
    const auto selected = processor.extractPoints(fixture.depth, fixture.info, fixture.transform,
                                                  fixture.roi, .3, 3., nullptr, &pixels);
    ASSERT_EQ(selected.size(), full.size());
    for (std::size_t i = 0; i < full.size(); ++i) {
        EXPECT_EQ(selected[i].x, full[i].x);
        EXPECT_EQ(selected[i].y, full[i].y);
        EXPECT_EQ(selected[i].z, full[i].z);
    }
}

TEST(SparseDepth, ExactPointsForU16AndF32IncludingPaddedRows)
{
    for (const bool floating : {false, true}) {
        for (const unsigned padding : {0u, 7u}) {
            Fixture fixture(65, 49, floating, padding);
            expectSamePoints(fixture);
        }
    }
}

TEST(SparseDepth, EmptySparseAndDenseBodySelectionsRetainEveryMatchingPixel)
{
    for (const int percent : {0, 1, 10, 50, 100}) {
        Fixture fixture(64, 48, false, 0, percent);
        expectSamePoints(fixture);
    }
}

TEST(SparseDepth, BodyIdentityUsesTheSameDriverModuloConvention)
{
    Fixture fixture;
    expectSamePoints(fixture, 256);
    expectSamePoints(fixture, 2);
    expectSamePoints(fixture, 254);
    expectSamePoints(fixture, 255);
}

TEST(SparseDepth, FragmentedBodyMasksRetainExactPoints)
{
    Fixture fixture(65, 49, false, 7);
    for (std::size_t y = 0; y < fixture.body.height; ++y) {
        for (std::size_t x = 0; x < fixture.body.width; ++x) {
            fixture.body.data[y * fixture.body.step + x] = (x + y) % 2 ? 255 : 1;
        }
    }
    expectSamePoints(fixture);
}

TEST(SparseDepth, DistortionRotationTranslationAndRoiClippingAreUnchanged)
{
    Fixture fixture(64, 48, true, 4, 100);
    fixture.info.distortion_model = "rational_polynomial";
    fixture.info.d = {.1, -.02, .001, -.001, .001, .01, 0., 0.};
    fixture.transform.transform.rotation.y = std::sin(.2);
    fixture.transform.transform.rotation.w = std::cos(.2);
    fixture.transform.transform.translation.x = .1;
    fixture.roi.min.x = -.1;
    fixture.roi.max.x = .45;
    expectSamePoints(fixture);
}

TEST(SparseDepth, InvalidFloatDepthNeverBecomesAReconstructedPoint)
{
    Fixture fixture(64, 48, true, 0, 100);
    const std::array<float, 6> bad{{0.f, -.1f, .2f, 4.f,
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}};
    for (std::size_t i = 0; i < bad.size(); ++i) {
        std::memcpy(&fixture.depth.data[i * sizeof(float)], &bad[i], sizeof(float));
    }
    expectSamePoints(fixture);
}

TEST(SparseDepth, CalibrationChangesInvalidateCachedLookup)
{
    Fixture fixture;
    DepthProcessor processor;
    const auto pixels = HumanSegmenter().generatePixelRuns(fixture.body, 1);
    const auto first = processor.extractPoints(fixture.depth, fixture.info, fixture.transform,
                                               fixture.roi, .3, 3., nullptr, &pixels);
    fixture.info.k[0] *= 1.3;
    const auto changed = processor.extractPoints(fixture.depth, fixture.info, fixture.transform,
                                                 fixture.roi, .3, 3., nullptr, &pixels);
    ASSERT_FALSE(first.empty());
    ASSERT_EQ(first.size(), changed.size());
    EXPECT_NE(first[0].x, changed[0].x);
    expectSamePoints(fixture);
}

TEST(SparseDepth, RejectsDuplicateUnorderedOutOfRangeAndMixedSelections)
{
    Fixture fixture;
    DepthProcessor processor;
    for (const std::vector<vision::segmentation::PixelRun>& pixels : {
            std::vector<vision::segmentation::PixelRun>{{1, 3}, {2, 4}},
            {{2, 4}, {0, 1}}, {{0, 64 * 48 + 1}}, {{2, 2}}}) {
        EXPECT_THROW(processor.extractPoints(fixture.depth, fixture.info, fixture.transform,
                     fixture.roi, .3, 3., nullptr, &pixels), std::invalid_argument);
    }
    auto mask = HumanSegmenter().generateMask(fixture.body, 1);
    auto pixels = HumanSegmenter().generatePixelRuns(fixture.body, 1);
    EXPECT_THROW(processor.extractPoints(fixture.depth, fixture.info, fixture.transform,
                 fixture.roi, .3, 3., &mask, &pixels), std::invalid_argument);
}

TEST(SparseDepth, RejectsMalformedImagesFramesTransformsAndDistortionEvenForEmptySelection)
{
    for (int kind = 0; kind < 8; ++kind) {
        Fixture fixture;
        const std::vector<vision::segmentation::PixelRun> pixels;
        switch (kind) {
            case 0: fixture.depth.step = 1; break;
            case 1: fixture.depth.data.resize(1); break;
            case 2: fixture.depth.encoding = "mono8"; break;
            case 3: fixture.depth.is_bigendian = 1; break;
            case 4: fixture.transform.child_frame_id = "wrong"; break;
            case 5: fixture.transform.transform.rotation.w = 0.; break;
            case 6: fixture.info.k[0] = 0.; break;
            case 7: fixture.info.d = {.1}; fixture.info.distortion_model = "unsupported"; break;
        }
        EXPECT_THROW(DepthProcessor().extractPoints(fixture.depth, fixture.info, fixture.transform,
                     fixture.roi, .3, 3., nullptr, &pixels), std::invalid_argument) << kind;
    }
}

TEST(SparseDepth, SegmentationRetainsMalformedInputRejection)
{
    for (int kind = 0; kind < 4; ++kind) {
        Fixture fixture;
        switch (kind) {
            case 0: fixture.body.encoding = "16UC1"; break;
            case 1: fixture.body.step = 0; break;
            case 2: fixture.body.width = 0; break;
            case 3: fixture.body.data.resize(1); break;
        }
        EXPECT_THROW(HumanSegmenter().generatePixelRuns(fixture.body, 1), std::invalid_argument);
        EXPECT_THROW(HumanSegmenter().generateMask(fixture.body, 1), std::invalid_argument);
    }
}

TEST(SparseDepth, CompletePipelineCapsulesMatchOriginalMaskRoute)
{
    Fixture fixture(128, 96, false, 0, 35);
    vision::reconstruction::ReconstructionConfig config;
    const auto& skeleton = fixture.skeletons.at(1);
    const auto roi = vision::reconstruction::ROIGenerator().generate(
        skeleton.points, config.roi_margin, 1, "human");
    const auto mask = HumanSegmenter().generateMask(fixture.body, 1);
    const auto points = DepthProcessor().extractPoints(
        fixture.depth, fixture.info, fixture.transform, roi, config.min_depth, config.max_depth, &mask);
    const auto voxels = vision::reconstruction::VoxelReconstruction().reconstruct(
        points, config.voxel_size);
    const auto reference = vision::human::HumanCapsule().generate(
        skeleton.joints, voxels, 1, config.capsule);
    vision::reconstruction::ReconstructionPipeline pipeline(config);
    vision::reconstruction::ReconstructionResult output;
    ASSERT_TRUE(pipeline.process(fixture.depth, fixture.body, fixture.info, fixture.transform,
                                fixture.skeletons, {1}, true, output));
    ASSERT_EQ(output.bodies.size(), 1u);
    ASSERT_EQ(output.bodies[0].capsules.size(), reference.size());
    ASSERT_FALSE(reference.empty());
    const auto& actual = output.bodies[0].capsules;
    for (std::size_t i = 0; i < reference.size(); ++i) {
        EXPECT_EQ(actual[i].radius, reference[i].radius);
        EXPECT_EQ(actual[i].radius_valid, reference[i].radius_valid);
        EXPECT_EQ(actual[i].segment_id, reference[i].segment_id);
        EXPECT_EQ(actual[i].start, reference[i].start);
        EXPECT_EQ(actual[i].end, reference[i].end);
    }
    ASSERT_EQ(output.bodies[0].voxels.size(), voxels.size());
    for (std::size_t i = 0; i < voxels.size(); ++i) {
        EXPECT_EQ(output.bodies[0].voxels[i].center, voxels[i].center);
    }
}

TEST(SparseDepth, PipelineRejectsMismatchedBodyIndexDimensions)
{
    Fixture fixture;
    fixture.body.width /= 2;
    vision::reconstruction::ReconstructionPipeline pipeline({});
    vision::reconstruction::ReconstructionResult output;
    EXPECT_THROW(pipeline.process(fixture.depth, fixture.body, fixture.info, fixture.transform,
                 fixture.skeletons, {1}, false, output), std::invalid_argument);
}

}  // namespace
