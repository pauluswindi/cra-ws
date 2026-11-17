// Offline-only C ABI for exercising the production controller and guards.
// This file is not part of the Motion component or its installed ROS targets.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>

#include "controllers/execution_monitor.hpp"
#include "controllers/joint_controller.hpp"
#include "modules/motion_guard.hpp"

namespace {
using Vector = motion::RobotKinematics::JointVector;
using Time = motion::JointController::TimePoint;

Time at(double seconds) {
    return Time{} + std::chrono::duration_cast<Time::duration>(
        std::chrono::duration<double>(seconds));
}

struct Replay {
    motion::RobotKinematics kinematics;
    motion::JointController controller;
    motion::MotionGuard guard;
    motion::ExecutionMonitor monitor;

    Replay(const char* xml, const double* p, const char* curve)
        : kinematics(xml),
          controller(kinematics,
                     {p[0], 0.6 * p[1], p[2], p[3], p[4]},
                     {p[5], p[6], curve}),
          guard(kinematics, p[1]),
          monitor({p[7], p[8], p[9], p[10]}) {}
};
}  // namespace

extern "C" {
void* tracking_create(const char* xml, const double* parameters,
                      const char* curve, const double* waypoints, int count,
                      const double* initial, char* error, int capacity) {
    try {
        auto replay = std::make_unique<Replay>(xml, parameters, curve);
        trajectory_msgs::msg::JointTrajectory message;
        message.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5"};
        for (int i = 0; i < count; ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions.assign(waypoints + 5 * i, waypoints + 5 * (i + 1));
            message.points.push_back(point);
        }
        if (!replay->controller.setTrajectory(message, Eigen::Map<const Vector>(initial),
                                             at(0.0))) {
            throw std::runtime_error(replay->controller.lastError());
        }
        replay->monitor.reset(at(0.0));
        return replay.release();
    } catch (const std::exception& exception) {
        std::snprintf(error, capacity, "%s", exception.what());
        return nullptr;
    }
}

// Output: reference q, reference dq, raw dq, guarded dq, controller state,
// monitor state, total trajectory time. The caller sends zero on a fault,
// matching MotionComponent::RunJointTrajectoryControl.
int tracking_step(void* handle, double seconds, const double* feedback, double* out) {
    try {
        auto& replay = *static_cast<Replay*>(handle);
        const Vector current = Eigen::Map<const Vector>(feedback);
        const auto sample = replay.controller.update(current, at(seconds));
        Vector filtered = Vector::Zero();
        auto monitor = motion::ExecutionMonitor::State::Normal;
        if (sample.state == motion::JointController::State::Running) {
            if (!replay.guard.filterVelocity(sample.velocity, current, filtered)) {
                return -1;
            }
            monitor = replay.monitor.update(at(seconds), replay.controller.totalTime(),
                sample.reference_position, current, filtered,
                replay.controller.goalTolerance());
        }
        for (int i = 0; i < 5; ++i) {
            out[i] = sample.reference_position[i];
            out[5 + i] = sample.reference_velocity[i];
            out[10 + i] = sample.velocity[i];
            out[15 + i] = filtered[i];
        }
        out[20] = static_cast<int>(sample.state);
        out[21] = static_cast<int>(monitor);
        out[22] = replay.controller.totalTime();
        return 0;
    } catch (...) {
        return -1;
    }
}

void tracking_destroy(void* handle) {
    delete static_cast<Replay*>(handle);
}
}
