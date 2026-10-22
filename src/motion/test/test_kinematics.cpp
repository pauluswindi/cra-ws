#include "kinematics/robot_kinematics.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr
            << "Usage: test_robot_kinematics "
            << "/path/to/cognitive_robotics_arm.xml\n";

        return 2;
    }

    try {
        using Robot = motion::RobotKinematics;
        using JointVector = Robot::JointVector;

        Robot robot(argv[1]);

        const JointVector lower = robot.lowerLimits();
        const JointVector upper = robot.upperLimits();

        constexpr double epsilon = 1e-6;
        constexpr double tolerance = 1e-5;

        double max_error = 0.0;

        for (int sample = 0; sample < 5; ++sample) {
            JointVector q;

            for (int i = 0; i < 5; ++i) {
                q[i] = lower[i]
                    + (upper[i] - lower[i])
                    * (sample + 1.0) / 6.0;
            }

            robot.setJointPositions(q);

            const auto jacobian = robot.positionJacobian(
                Robot::Frame::World
            );

            for (int i = 0; i < 5; ++i) {
                JointVector q_plus = q;
                JointVector q_minus = q;

                q_plus[i] += epsilon;
                q_minus[i] -= epsilon;

                robot.setJointPositions(q_plus);

                const Eigen::Vector3d p_plus =
                    robot.endEffectorPosition(
                        Robot::Frame::World
                    );

                robot.setJointPositions(q_minus);

                const Eigen::Vector3d p_minus =
                    robot.endEffectorPosition(
                        Robot::Frame::World
                    );

                const Eigen::Vector3d numerical =
                    (p_plus - p_minus) / (2.0 * epsilon);

                const double error =
                    (numerical - jacobian.col(i)).norm();

                if (!std::isfinite(error)) {
                    throw std::runtime_error(
                        "Non-finite Jacobian error"
                    );
                }

                if (error > max_error) {
                    max_error = error;
                }

                if (error > tolerance) {
                    throw std::runtime_error(
                        "Jacobian finite-difference mismatch"
                    );
                }
            }
        }

        std::cout
            << "Maximum Jacobian error: "
            << max_error << '\n';

        std::cout
            << "PASS: FK/Jacobian finite-difference checks\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr
            << "FAIL: " << e.what() << '\n';

        return 1;
    }
}