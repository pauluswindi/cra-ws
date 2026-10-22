#ifndef MOTION_PROFILE_HPP_
#define MOTION_PROFILE_HPP_

#include <string>

namespace motion {

class MotionProfile {
public:
    enum class Type {
        Linear,
        EaseIn,
        EaseOut,
        SmoothStep,
        SmootherStep
    };

    static Type fromString(const std::string& name);

    static double position(Type type, double t);

    static double velocity(Type type, double t);

    static double acceleration(Type type, double t);
};

}  // namespace motion

#endif  // MOTION_PROFILE_HPP_