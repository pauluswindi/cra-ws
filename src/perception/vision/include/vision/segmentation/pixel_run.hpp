#pragma once

#include <cstddef>

namespace vision::segmentation
{

// Half-open range of matching row-major pixels; rows may contain byte padding.
struct PixelRun
{
    std::size_t begin;
    std::size_t end;
};

}  // namespace vision::segmentation
