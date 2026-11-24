#include <algorithm>
#include <chrono>
#include <iostream>

#include "vision/frame/preview_decode.hpp"

int main()
{
    cv::Mat input(1536, 2048, CV_8UC3);
    cv::RNG random(42);
    random.fill(input, cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(input, input, cv::Size(5, 5), 0.);
    std::vector<std::uint8_t> encoded;
    cv::imencode(".jpg", input, encoded, {cv::IMWRITE_JPEG_QUALITY, 90});
    std::vector<double> full, reduced;
    for (int sample = 0; sample < 42; ++sample) {
        for (const int mode : {sample % 2, 1 - sample % 2}) {
            const auto began = std::chrono::steady_clock::now();
            cv::Mat preview;
            if (mode == 0) {
                auto decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
                cv::resize(decoded, preview, cv::Size(1024, 768));
            } else {
                preview = vision::frame::decodePreview(encoded, .5);
            }
            const auto elapsed = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - began).count();
            if (preview.size() != cv::Size(1024, 768)) { return 1; }
            if (sample > 1) { (mode == 0 ? full : reduced).push_back(elapsed); }
        }
    }
    std::sort(full.begin(), full.end());
    std::sort(reduced.begin(), reduced.end());
    std::cout << "jpeg_bytes,full_decode_resize_median_ms,reduced_decode_median_ms\n"
              << encoded.size() << ',' << full[full.size() / 2] << ','
              << reduced[reduced.size() / 2] << '\n';
}
