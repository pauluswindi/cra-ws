#include "kinematics/robot_kinematics.hpp"
#include "kinematics/jacobian.hpp"

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <cmath>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr
            << "Usage: test_jacobian <robot_xml_path>\n";
        return 2;
    }

    try {
        using motion::RobotKinematics;
        using motion::Jacobian;

        using JointVector = RobotKinematics::JointVector;

        RobotKinematics robot(argv[1]);

        const JointVector lower = robot.lowerLimits();
        const JointVector upper = robot.upperLimits();

        JointVector joints = lower + 0.5 * (upper - lower);

        robot.setJointPositions(joints);

        const auto J = Jacobian::computeJacobian(robot);

        if (!J.allFinite()) {
            throw std::runtime_error(
                "Jacobian contains invalid values"
            );
        }

        const Eigen::Vector3d zero_velocity =
            Eigen::Vector3d::Zero();

        const JointVector zero_result =
            Jacobian::inverseJacobian(
                J,
                zero_velocity,
                0.01
            );

        if (zero_result.norm() > 1e-12) {
            throw std::runtime_error(
                "Zero Cartesian velocity must produce zero joint velocity"
            );
        }

        const Eigen::Vector3d desired_velocity(
            0.01,
            0.02,
            -0.01
        );

        constexpr double damping = 1e-4;

        const JointVector joint_velocity =
            Jacobian::inverseJacobian(
                J,
                desired_velocity,
                damping
            );

        if (!joint_velocity.allFinite()) {
            throw std::runtime_error(
                "Invalid joint velocity"
            );
        }

        const Eigen::Vector3d actual_velocity =
            J * joint_velocity;

        const double tracking_error =
            (actual_velocity - desired_velocity).norm();

        std::cout
            << "Jacobian velocity error: "
            << tracking_error << '\n';

        const Eigen::Matrix3d regularized =
            J * J.transpose() +
            damping * damping * Eigen::Matrix3d::Identity();

        const Eigen::Vector3d optimality_residual =
            J * joint_velocity - desired_velocity +
            damping * damping *
            regularized.ldlt().solve(desired_velocity);

        if (!optimality_residual.allFinite() ||
            optimality_residual.norm() > 1e-8) {
            throw std::runtime_error(
                "DLS solution does not satisfy the expected equations"
            );
        }

        bool invalid_damping_rejected = false;

        try {
            Jacobian::inverseJacobian(
                J,
                desired_velocity,
                -0.01
            );
        } catch (const std::invalid_argument&) {
            invalid_damping_rejected = true;
        }

        if (!invalid_damping_rejected) {
            throw std::runtime_error(
                "Invalid damping was not rejected"
            );
        }

        const Eigen::Vector3d target =
            robot.forwardKinematics();

        JointVector initial_joints = joints;
        initial_joints[0] += 0.02;
        initial_joints[1] -= 0.02;

        initial_joints =
            initial_joints.cwiseMax(lower).cwiseMin(upper);

        JointVector ik_solution = JointVector::Zero();

        const bool ik_success = robot.inverseKinematics(
            target,
            initial_joints,
            ik_solution
        );

        if (!ik_success) {
            throw std::runtime_error(
                "Inverse kinematics failed for a reachable target"
            );
        }

        robot.setJointPositions(ik_solution);

        const double position_error =
            (robot.forwardKinematics() - target).norm();

        std::cout
            << "IK position error: "
            << position_error << '\n';

        if (position_error > 1e-4) {
            throw std::runtime_error(
                "Inverse kinematics position error too large"
            );
        }

        JointVector unreachable_solution = JointVector::Zero();

        const bool unreachable_success =
            robot.inverseKinematics(
                Eigen::Vector3d(100.0, 100.0, 100.0),
                joints,
                unreachable_solution
            );

        if (unreachable_success) {
            throw std::runtime_error(
                "IK accepted an unreachable target"
            );
        }

        std::cout
            << "PASS: Jacobian DLS and inverse kinematics tests\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}