#pragma once

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include <array>
#include <memory>
#include <string>

namespace motion {

class RobotKinematics {
public:
    static constexpr int kJointCount = 5;

    using JointVector = Eigen::Matrix<double, kJointCount, 1>;
    using PositionJacobian = Eigen::Matrix<double, 3, kJointCount>;

    enum class Frame {
        World,
        Base
    };

    struct IKConfig {
        int max_iterations = 200;
        double tolerance = 1e-4;
        double damping = 0.01;
        double step_size = 0.5;
        double max_joint_step = 0.1;
    };

    explicit RobotKinematics(
        const std::string& xml_path,
        const std::string& base_body = "base_link",
        const std::string& end_effector_body = "end_effector"
    );

    void setJointPositions(const JointVector& joints);

    JointVector jointPositions() const;

    Eigen::Vector3d forwardKinematics(
        Frame frame = Frame::Base
    ) const;

    bool inverseKinematics(
        const Eigen::Vector3d& target_position,
        const JointVector& initial_joints,
        JointVector& solution,
        Frame frame = Frame::Base
    );

    void setIKConfig(const IKConfig& config);

    const IKConfig& getIKConfig() const;

    JointVector lowerLimits() const;
    JointVector upperLimits() const;

    Eigen::Vector3d endEffectorPosition(
        Frame frame = Frame::Base
    ) const;

    PositionJacobian positionJacobian(
        Frame frame = Frame::Base
    ) const;

private:
    struct ModelDeleter {
        void operator()(mjModel* model) const {
            mj_deleteModel(model);
        }
    };

    struct DataDeleter {
        void operator()(mjData* data) const {
            mj_deleteData(data);
        }
    };

    std::unique_ptr<mjModel, ModelDeleter> model_;
    std::unique_ptr<mjData, DataDeleter> data_;

    std::array<int, kJointCount> joint_ids_{};
    std::array<int, kJointCount> qpos_addresses_{};
    std::array<int, kJointCount> dof_addresses_{};

    int base_body_id_ = -1;
    int end_effector_id_ = -1;

    IKConfig ik_config_;
};

}  // namespace motion