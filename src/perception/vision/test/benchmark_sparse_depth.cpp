#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>

#include "vision_fixture.hpp"

int main()
{
    using Clock = std::chrono::steady_clock;
    std::cout << "density_percent,selected_pixels,points,mask_median_ms,runs_median_ms\n";
    for (const int percent : {0, 1, 10, 35, 50, 100}) {
        vision_test::Fixture fixture(512, 512, false, 0, percent);
        vision::segmentation::HumanSegmenter segmenter;
        vision::reconstruction::DepthProcessor processor;
        const auto mask = segmenter.generateMask(fixture.body, 1);
        const auto indices = segmenter.generatePixelRuns(fixture.body, 1);
        const auto reference = processor.extractPoints(fixture.depth, fixture.info,
            fixture.transform, fixture.roi, .3, 3., &mask);
        const auto optimized = processor.extractPoints(fixture.depth, fixture.info,
            fixture.transform, fixture.roi, .3, 3., nullptr, &indices);
        if (reference != optimized) {
            throw std::runtime_error("Depth points differ");
        }
        std::vector<double> full_times, selected_times;
        std::size_t checksum = 0;
        for (int iteration = 0; iteration < 22; ++iteration) {
            for (const int mode : {iteration % 2, 1 - iteration % 2}) {
                const auto began = Clock::now();
                const auto generated_mask = mode == 0 ? segmenter.generateMask(fixture.body, 1)
                                                     : std::vector<std::uint8_t>{};
                const auto generated_indices = mode == 1
                    ? segmenter.generatePixelRuns(fixture.body, 1) : std::vector<vision::segmentation::PixelRun>{};
                const auto points = processor.extractPoints(fixture.depth, fixture.info,
                    fixture.transform, fixture.roi, .3, 3., mode == 0 ? &generated_mask : nullptr,
                    mode == 1 ? &generated_indices : nullptr);
                checksum += points.size();
                if (iteration >= 2) {
                    const double ms = std::chrono::duration<double, std::milli>(
                        Clock::now() - began).count();
                    (mode == 0 ? full_times : selected_times).push_back(ms);
                }
            }
        }
        if (checksum != reference.size() * 44) {
            throw std::runtime_error("Measured output differs");
        }
        std::sort(full_times.begin(), full_times.end());
        std::sort(selected_times.begin(), selected_times.end());
        std::cout << percent << ',' << reference.size() << ',' << reference.size() << ','
                  << full_times[full_times.size() / 2] << ','
                  << selected_times[selected_times.size() / 2] << '\n';
    }
}
