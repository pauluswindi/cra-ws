#include "trajectory/motion_profile.hpp"

#include <algorithm>
#include <stdexcept>

namespace motion {

MotionProfile::Type MotionProfile::fromString(
    const std::string& name
) {
    if (name == "linear") {
        return Type::Linear;
    }

    if (name == "ease_in") {
        return Type::EaseIn;
    }

    if (name == "ease_out") {
        return Type::EaseOut;
    }

    if (name == "smooth_step") {
        return Type::SmoothStep;
    }

    if (name == "smoother_step") {
        return Type::SmootherStep;
    }

    throw std::invalid_argument(
        "Unsupported motion profile: " + name
    );
}

double MotionProfile::position(Type type, double t) {
    t = std::clamp(t, 0.0, 1.0);

    switch (type) {
        case Type::Linear:
            return t;

        case Type::EaseIn:
            return t * t;

        case Type::EaseOut:
            return 1.0 - (1.0 - t) * (1.0 - t);

        case Type::SmoothStep:
            return 3.0 * t * t - 2.0 * t * t * t;

        case Type::SmootherStep:
            return 6.0 * t * t * t * t * t
                 - 15.0 * t * t * t * t
                 + 10.0 * t * t * t;
    }

    throw std::invalid_argument(
        "Invalid motion profile type"
    );
}

double MotionProfile::velocity(Type type, double t) {
    t = std::clamp(t, 0.0, 1.0);

    switch (type) {
        case Type::Linear:
            return 1.0;

        case Type::EaseIn:
            return 2.0 * t;

        case Type::EaseOut:
            return 2.0 * (1.0 - t);

        case Type::SmoothStep:
            return 6.0 * t - 6.0 * t * t;

        case Type::SmootherStep:
            return 30.0 * t * t
                 - 60.0 * t * t * t
                 + 30.0 * t * t * t * t;
    }

    throw std::invalid_argument(
        "Invalid motion profile type"
    );
}

double MotionProfile::acceleration(Type type, double t) {
    t = std::clamp(t, 0.0, 1.0);

    switch (type) {
        case Type::Linear:
            return 0.0;

        case Type::EaseIn:
            return 2.0;

        case Type::EaseOut:
            return -2.0;

        case Type::SmoothStep:
            return 6.0 - 12.0 * t;

        case Type::SmootherStep:
            return 60.0 * t
                 - 180.0 * t * t
                 + 120.0 * t * t * t;
    }

    throw std::invalid_argument(
        "Invalid motion profile type"
    );
}

}  // namespace motion