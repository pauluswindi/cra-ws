#include "kinematics/jacobian.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <stdexcept>

namespace motion {

Jacobian::JacobianMatrix Jacobian::computeJacobian(
    const RobotKinematics& robot,
    Frame frame
) {
    return robot.positionJacobian(frame);
}

Jacobian::JointVector Jacobian::inverseJacobian(
    const JacobianMatrix& jacobian,
    const Eigen::Vector3d& cartesian_velocity,
    double damping
) {
    if (!jacobian.allFinite() ||
        !cartesian_velocity.allFinite()) {
        throw std::invalid_argument(
            "Jacobian and Cartesian velocity must be finite"
        );
    }

    if (!std::isfinite(damping) || damping <= 0.0) {
        throw std::invalid_argument(
            "Damping must be positive and finite"
        );
    }

    const Eigen::Matrix3d regularized =
        jacobian * jacobian.transpose() +
        damping * damping * Eigen::Matrix3d::Identity();

    const Eigen::LDLT<Eigen::Matrix3d> solver(regularized);

    if (solver.info() != Eigen::Success) {
        throw std::runtime_error(
            "Failed to decompose damped Jacobian system"
        );
    }

    const Eigen::Vector3d solution =
        solver.solve(cartesian_velocity);

    if (solver.info() != Eigen::Success ||
        !solution.allFinite()) {
        throw std::runtime_error(
            "Failed to solve inverse Jacobian"
        );
    }

    const JointVector joint_velocity =
        jacobian.transpose() * solution;

    if (!joint_velocity.allFinite()) {
        throw std::runtime_error(
            "Inverse Jacobian produced invalid joint velocity"
        );
    }

    return joint_velocity;
}

}  // namespace motion