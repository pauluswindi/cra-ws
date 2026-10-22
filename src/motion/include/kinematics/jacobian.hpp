#pragma once

#include "kinematics/robot_kinematics.hpp"

#include <Eigen/Core>

namespace motion {

class Jacobian {
public:
    using JointVector = RobotKinematics::JointVector;
    using JacobianMatrix = RobotKinematics::PositionJacobian;
    using Frame = RobotKinematics::Frame;

    static JacobianMatrix computeJacobian(
        const RobotKinematics& robot,
        Frame frame = Frame::Base
    );

    static JointVector inverseJacobian(
        const JacobianMatrix& jacobian,
        const Eigen::Vector3d& cartesian_velocity,
        double damping
    );
};

}  // namespace motion