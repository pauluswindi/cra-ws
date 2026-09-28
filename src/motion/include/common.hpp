#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <array>
#include <cstdint>
#include <chrono>

namespace Utility {

class TimeCounter {
public:
    TimeCounter() { Reset(); }
    void Reset() { start_time_ = std::chrono::steady_clock::now(); }
    float Count() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<float, std::milli>(now - start_time_).count();
    }
private:
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace Utility

namespace servo {
    constexpr int kServoCount = 2;
    constexpr int kMinServoId = 0;
    constexpr int kMaxServoId = 13;
}

namespace MotionState {
    enum class State {
        Init,
        Test,
        Ready,
        Reset,
        Teleop
    };
}

#endif // COMMON_HPP_