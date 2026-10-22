#include "kinematics/robot_kinematics.hpp"
#include "kinematics/jacobian.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace motion {

RobotKinematics::RobotKinematics(
    const std::string& xml_path,
    const std::string& base_body,
    const std::string& end_effector_body
) {
    char error[1024] = {};

    model_.reset(
        mj_loadXML(
            xml_path.c_str(),
            nullptr,
            error,
            sizeof(error)
        )
    );

    if (!model_) {
        throw std::runtime_error(
            "Failed to load MuJoCo XML: " + std::string(error)
        );
    }

    data_.reset(mj_makeData(model_.get()));

    if (!data_) {
        throw std::runtime_error(
            "Failed to allocate MuJoCo data"
        );
    }

    base_body_id_ = mj_name2id(
        model_.get(),
        mjOBJ_BODY,
        base_body.c_str()
    );

    end_effector_id_ = mj_name2id(
        model_.get(),
        mjOBJ_BODY,
        end_effector_body.c_str()
    );

    if (base_body_id_ < 0 || end_effector_id_ < 0) {
        throw std::runtime_error(
            "Required robot bodies not found in MuJoCo model"
        );
    }

    if (model_->nq != kJointCount ||
        model_->nv != kJointCount) {
        throw std::runtime_error(
            "Expected exactly five scalar joint DoFs"
        );
    }

    for (int i = 0; i < kJointCount; ++i) {
        const std::string name =
            "joint" + std::to_string(i + 1);

        const int id = mj_name2id(
            model_.get(),
            mjOBJ_JOINT,
            name.c_str()
        );

        if (id < 0 || model_->jnt_type[id] != mjJNT_HINGE) {
            throw std::runtime_error(
                "Expected revolute joint: " + name
            );
        }

        joint_ids_[i] = id;
        qpos_addresses_[i] = model_->jnt_qposadr[id];
        dof_addresses_[i] = model_->jnt_dofadr[id];
    }

    mj_forward(model_.get(), data_.get());
}

void RobotKinematics::setJointPositions(
    const JointVector& joints
) {
    if (!joints.allFinite()) {
        throw std::invalid_argument(
            "Joint positions must be finite"
        );
    }

    for (int i = 0; i < kJointCount; ++i) {
        data_->qpos[qpos_addresses_[i]] = joints[i];
    }

    mj_forward(model_.get(), data_.get());
}

RobotKinematics::JointVector
RobotKinematics::jointPositions() const {
    JointVector joints;

    for (int i = 0; i < kJointCount; ++i) {
        joints[i] = data_->qpos[qpos_addresses_[i]];
    }

    return joints;
}

Eigen::Vector3d RobotKinematics::forwardKinematics(
    Frame frame
) const {
    return endEffectorPosition(frame);
}

Eigen::Vector3d RobotKinematics::endEffectorPosition(
    Frame frame
) const {
    const Eigen::Vector3d world(
        data_->xpos[3 * end_effector_id_],
        data_->xpos[3 * end_effector_id_ + 1],
        data_->xpos[3 * end_effector_id_ + 2]
    );

    if (frame == Frame::World) {
        return world;
    }

    const Eigen::Vector3d base(
        data_->xpos[3 * base_body_id_],
        data_->xpos[3 * base_body_id_ + 1],
        data_->xpos[3 * base_body_id_ + 2]
    );

    const Eigen::Map<
        const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>
    > rotation(
        data_->xmat + 9 * base_body_id_
    );

    return rotation.transpose() * (world - base);
}

RobotKinematics::PositionJacobian
RobotKinematics::positionJacobian(Frame frame) const {
    Eigen::Matrix<
        double,
        3,
        Eigen::Dynamic,
        Eigen::RowMajor
    > jacobian(3, model_->nv);

    mj_jacBody(
        model_.get(),
        data_.get(),
        jacobian.data(),
        nullptr,
        end_effector_id_
    );

    PositionJacobian result;

    for (int i = 0; i < kJointCount; ++i) {
        result.col(i) = jacobian.col(dof_addresses_[i]);
    }

    if (frame == Frame::Base) {
        const Eigen::Map<
            const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>
        > rotation(
            data_->xmat + 9 * base_body_id_
        );

        result = rotation.transpose() * result;
    }

    return result;
}

RobotKinematics::JointVector
RobotKinematics::lowerLimits() const {
    JointVector limits;

    for (int i = 0; i < kJointCount; ++i) {
        const int id = joint_ids_[i];

        limits[i] = model_->jnt_limited[id]
            ? model_->jnt_range[2 * id]
            : -mjMAXVAL;
    }

    return limits;
}

RobotKinematics::JointVector
RobotKinematics::upperLimits() const {
    JointVector limits;

    for (int i = 0; i < kJointCount; ++i) {
        const int id = joint_ids_[i];

        limits[i] = model_->jnt_limited[id]
            ? model_->jnt_range[2 * id + 1]
            : mjMAXVAL;
    }

    return limits;
}

void RobotKinematics::setIKConfig(
    const IKConfig& config
) {
    if (config.max_iterations <= 0 ||
        !std::isfinite(config.tolerance) ||
        !std::isfinite(config.damping) ||
        !std::isfinite(config.step_size) ||
        !std::isfinite(config.max_joint_step) ||
        config.tolerance <= 0.0 ||
        config.damping <= 0.0 ||
        config.step_size <= 0.0 ||
        config.step_size > 1.0 ||
        config.max_joint_step <= 0.0) {
        throw std::invalid_argument(
            "Invalid inverse kinematics configuration"
        );
    }

    ik_config_ = config;
}

const RobotKinematics::IKConfig&
RobotKinematics::getIKConfig() const {
    return ik_config_;
}

bool RobotKinematics::inverseKinematics(
    const Eigen::Vector3d& target_position,
    const JointVector& initial_joints,
    JointVector& solution,
    Frame frame
) {
    if (!target_position.allFinite() ||
        !initial_joints.allFinite()) {
        throw std::invalid_argument(
            "Inverse kinematics inputs must be finite"
        );
    }

    const JointVector previous_joints = jointPositions();
    const JointVector lower = lowerLimits();
    const JointVector upper = upperLimits();

    JointVector joints =
        initial_joints.cwiseMax(lower).cwiseMin(upper);

    bool success = false;

    try {
        for (int iteration = 0;
             iteration < ik_config_.max_iterations;
             ++iteration) {

            setJointPositions(joints);

            const Eigen::Vector3d error =
                target_position - forwardKinematics(frame);

            if (error.norm() < ik_config_.tolerance) {
                solution = joints;
                success = true;
                break;
            }

            const auto J =
                Jacobian::computeJacobian(*this, frame);

            JointVector delta =
                Jacobian::inverseJacobian(
                    J,
                    error,
                    ik_config_.damping
                );

            const double step_norm = delta.norm();

            if (step_norm > ik_config_.max_joint_step) {
                delta *= ik_config_.max_joint_step / step_norm;
            }

            joints += ik_config_.step_size * delta;

            joints =
                joints.cwiseMax(lower).cwiseMin(upper);
        }
    } catch (...) {
        setJointPositions(previous_joints);
        throw;
    }

    setJointPositions(previous_joints);

    return success;
}

}  // namespace motion