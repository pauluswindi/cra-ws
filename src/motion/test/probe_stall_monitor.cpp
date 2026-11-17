// Offline diagnostic of the unchanged production stall detector.
// Synthetic feedback/reference/command sequences are not robot execution.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>

#include "controllers/execution_monitor.hpp"

namespace {
using Vector = motion::ExecutionMonitor::JointVector;
using Time = motion::ExecutionMonitor::TimePoint;

bool run(const std::string& name, int mode) {
    motion::ExecutionMonitor monitor({10.0, 2.0, 0.2, 0.5});
    monitor.reset(Time{});
    auto state = motion::ExecutionMonitor::State::Normal;
    double previous = 0.0;
    double travelled = 0.0;
    double commanded = 0.0;
    double max_error = 0.0;
    double elapsed = 0.0;
    double actual = 0.0;
    double reference = 0.0;
    for (int i = 0; i <= 400; ++i) {
        elapsed = i * 0.005;
        double velocity = 0.15;
        if (mode == 0) {
            actual = 0.06 * std::sin(std::numbers::pi * elapsed);
            reference = 0.06 * std::sin(std::numbers::pi * (elapsed + 0.15));
            velocity = 0.06 * std::numbers::pi * std::cos(std::numbers::pi * elapsed);
        } else if (mode == 1) {
            actual = 0.0;
            reference = 0.08;
        } else {
            actual = 0.15 * elapsed;
            reference = actual + 0.03;
        }
        travelled += std::abs(actual - previous);
        if (i > 0) {
            commanded += std::abs(velocity) * 0.005;
        }
        previous = actual;
        max_error = std::max(max_error, std::abs(reference - actual));
        Vector q = Vector::Zero();
        Vector target = Vector::Zero();
        Vector command = Vector::Zero();
        q[0] = actual;
        target[0] = reference;
        command[0] = velocity;
        const auto now = Time{} + std::chrono::milliseconds(i * 5);
        state = monitor.update(now, 8.0, target, q, command, 0.02);
        if (state != motion::ExecutionMonitor::State::Normal) {
            break;
        }
    }
    const bool expected = (mode < 2)
        ? state == motion::ExecutionMonitor::State::Stalled
        : state == motion::ExecutionMonitor::State::Normal;
    const bool reversal_witness = mode != 0 || (
        travelled > 0.23 && std::abs(actual) < 0.02 &&
        std::abs(reference - actual) > 0.02 && max_error < 0.2);
    const bool passed = expected && reversal_witness;
    std::cout << std::setprecision(12)
              << "{\"case\":\"" << name << "\",\"state\":\""
              << motion::ExecutionMonitor::stateName(state)
              << "\",\"elapsed_s\":" << elapsed
              << ",\"travelled_rad\":" << travelled
              << ",\"net_displacement_rad\":" << std::abs(actual)
              << ",\"commanded_absolute_rad\":" << commanded
              << ",\"final_error_rad\":" << std::abs(reference - actual)
              << ",\"max_error_rad\":" << max_error
              << ",\"expected_observation_verified\":" << (passed ? "true" : "false")
              << "}\n";
    return passed;
}
}  // namespace

int main() {
    const bool reversal = run("moving_then_reversing", 0);
    const bool stationary = run("truly_stationary_control", 1);
    const bool monotonic = run("monotonic_motion_control", 2);
    return reversal && stationary && monotonic ? 0 : 1;
}
