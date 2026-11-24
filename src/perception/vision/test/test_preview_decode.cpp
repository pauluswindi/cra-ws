#include <limits>

#include "gtest/gtest.h"
#include "vision/frame/preview_decode.hpp"

namespace
{

cv::Mat image(int width = 2048, int height = 1536)
{
    cv::Mat result(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            result.at<cv::Vec3b>(y, x) = cv::Vec3b(x % 256, y % 256, (x + y) % 256);
        }
    }
    return result;
}

cv::Mat reference(const std::vector<std::uint8_t>& encoded, double scale)
{
    auto decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (!decoded.empty() && scale < 1.) {
        cv::resize(decoded, decoded, cv::Size(static_cast<int>(decoded.cols * scale),
                                             static_cast<int>(decoded.rows * scale)));
    }
    return decoded;
}

TEST(PreviewDecode, KeepsConfiguredPreviewSizeForJpegAndProgressiveJpeg)
{
    for (const int progressive : {0, 1}) {
        std::vector<std::uint8_t> encoded;
        ASSERT_TRUE(cv::imencode(".jpg", image(), encoded, {cv::IMWRITE_JPEG_PROGRESSIVE, progressive}));
        const auto original = encoded;
        ASSERT_EQ(vision::frame::jpegSize(encoded), cv::Size(2048, 1536));
        for (const double scale : {1., .5, .25, .125}) {
            const auto current = vision::frame::decodePreview(encoded, scale);
            ASSERT_EQ(current.size(), reference(encoded, scale).size());
            ASSERT_EQ(current.type(), CV_8UC3);
        }
        EXPECT_EQ(encoded, original);
    }
}

TEST(PreviewDecode, FullResolutionOddDimensionsAndOtherScalesUseOriginalPath)
{
    for (const cv::Size& size : {cv::Size(2048, 1536), cv::Size(641, 481)}) {
        std::vector<std::uint8_t> encoded;
        ASSERT_TRUE(cv::imencode(".jpg", image(size.width, size.height), encoded));
        for (const double scale : {1., .3, .7}) {
            EXPECT_EQ(cv::norm(reference(encoded, scale),
                              vision::frame::decodePreview(encoded, scale), cv::NORM_INF), 0.);
        }
        if (size.width % 2) {
            EXPECT_EQ(cv::norm(reference(encoded, .5),
                              vision::frame::decodePreview(encoded, .5), cv::NORM_INF), 0.);
        }
    }
}

TEST(PreviewDecode, NonJpegEncodedImagesRetainTheOriginalFallback)
{
    std::vector<std::uint8_t> encoded;
    ASSERT_TRUE(cv::imencode(".png", image(320, 240), encoded));
    EXPECT_EQ(vision::frame::jpegSize(encoded), cv::Size());
    EXPECT_EQ(cv::norm(reference(encoded, .5),
                      vision::frame::decodePreview(encoded, .5), cv::NORM_INF), 0.);
}

TEST(PreviewDecode, RejectsInvalidScaleAndSafelyHandlesMalformedHeaders)
{
    for (const double scale : {0., -.1, 2., std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_THROW(vision::frame::decodePreview({}, scale), cv::Exception);
    }
    EXPECT_TRUE(vision::frame::decodePreview({}, .5).empty());
    for (const std::vector<std::uint8_t>& data : {
            std::vector<std::uint8_t>{0xff, 0xd8, 0xff, 0xe0, 0xff, 0xff},
            {0xff, 0xd8, 0xff, 0xff}, {0xff, 0xd8, 0xff, 0xc0, 0, 1}, {0, 1}}) {
        EXPECT_EQ(vision::frame::jpegSize(data), cv::Size());
    }
}

}  // namespace
