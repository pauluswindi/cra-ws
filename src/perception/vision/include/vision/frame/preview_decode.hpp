#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace vision::frame
{

inline cv::Size jpegSize(const std::vector<std::uint8_t>& data)
{
    if (data.size() < 4 || data[0] != 0xff || data[1] != 0xd8) {
        return {};
    }
    std::size_t offset = 2;
    while (offset < data.size()) {
        if (data[offset++] != 0xff) { return {}; }
        while (offset < data.size() && data[offset] == 0xff) { ++offset; }
        if (offset == data.size()) { return {}; }
        const auto marker = data[offset++];
        if (marker == 0xda || marker == 0xd9) { return {}; }
        if (marker == 0x01 || (marker >= 0xd0 && marker <= 0xd8)) { continue; }
        if (data.size() - offset < 2) { return {}; }
        const std::size_t length = (data[offset] << 8) | data[offset + 1];
        if (length < 2 || length > data.size() - offset) { return {}; }
        const bool sof = marker >= 0xc0 && marker <= 0xcf
                         && marker != 0xc4 && marker != 0xc8 && marker != 0xcc;
        if (sof && length >= 8) {
            return cv::Size((data[offset + 5] << 8) | data[offset + 6],
                            (data[offset + 3] << 8) | data[offset + 4]);
        }
        offset += length;
    }
    return {};
}

// Preview only: the original JPEG, depth and perception inputs are untouched.
inline cv::Mat decodePreview(const std::vector<std::uint8_t>& data, double scale)
{
    if (!std::isfinite(scale) || scale <= 0. || scale > 1.) {
        CV_Error(cv::Error::StsBadArg, "Invalid preview scale");
    }
    if (data.empty()) { return {}; }
    const auto size = jpegSize(data);
    int divisor = 1;
    int flag = cv::IMREAD_COLOR;
    if (size.width > 0 && size.height > 0) {
        for (const int factor : {8, 4, 2}) {
            if (scale == 1. / factor && size.width % factor == 0 && size.height % factor == 0) {
                divisor = factor;
                flag = factor == 8 ? cv::IMREAD_REDUCED_COLOR_8
                     : factor == 4 ? cv::IMREAD_REDUCED_COLOR_4 : cv::IMREAD_REDUCED_COLOR_2;
                break;
            }
        }
    }
    auto frame = cv::imdecode(data, flag);
    if (frame.empty()) { return frame; }
    // EXIF rotation or unexpected dimensions use the original full-decode path.
    if (divisor > 1 && frame.size() != cv::Size(size.width / divisor, size.height / divisor)) {
        frame = cv::imdecode(data, cv::IMREAD_COLOR);
        divisor = 1;
    }
    if (divisor == 1 && scale < 1. && !frame.empty()) {
        const cv::Size target(static_cast<int>(frame.cols * scale),
                              static_cast<int>(frame.rows * scale));
        cv::resize(frame, frame, target);
    }
    return frame;
}

}  // namespace vision::frame
