#include "../include/franka_o80/kinematics.hpp"

bool franka_o80::Kinematics::initialized_ = false;
size_t franka_o80::Kinematics::robot_joint_ids_[7];
pinocchio::Model franka_o80::Kinematics::model_;
pinocchio::Data franka_o80::Kinematics::data_;

void franka_o80::Kinematics::initialize_()
{
    pinocchio::urdf::buildModel("../model/franka.urdf", model_);
    data_ = pinocchio::Data(model_);
    for (size_t i = 0; i < 7; i++)
    {
        std::string name = "panda_joint" + std::to_string(i + 1);
        if (!model_.existJointName(name)) throw std::runtime_error("Link not found");
        robot_joint_ids_[i] = model_.getJointId(name);
    }
    initialized_ = true;
}

void franka_o80::joint_to_cartesian(States &states)
{
    if (!Kinematics::initialized_) Kinematics::initialize_();

    //Transforming to Eigen
    Eigen::VectorXd joint_positions(Kinematics::model_.nq);
    joint_positions.setZero();
    for (size_t i = 0; i < 7; i++) joint_positions(Kinematics::robot_joint_ids_[i] - 1) = states.get(joint_position[i]); //-1 because of universe joint does not go here

    //Performing forward kinematics
    pinocchio::forwardKinematics(Kinematics::model_, Kinematics::data_, joint_positions);
    pinocchio::SE3 se3 = Kinematics::data_.oMi[Kinematics::robot_joint_ids_[6]];
    
    //Fix: rotating 45 degree around "forward" and translation 0.2 along "forward"
    Eigen::Matrix3d rotation = se3.rotation_impl();
    Eigen::Vector3d forward = rotation.col(2);
    rotation = Eigen::AngleAxisd(M_PI / 4, forward) * rotation;
    Eigen::Vector3d translation = se3.translation_impl() + 0.2 * forward;

    //Returning result
    Eigen::Vector3d euler = rotation.eulerAngles(2, 1, 0);
    for (size_t i = 0; i < 3; i++) states.set(cartesian_position[i], translation(i));
    for (size_t i = 0; i < 3; i++) states.set(cartesian_orientation[i], euler(i));
}

void franka_o80::cartesian_to_joint(States &states)
{
    return cartesian_to_joint(states, default_states());
}

void franka_o80::cartesian_to_joint(States &states, const States &hint)
{
    if (!Kinematics::initialized_) Kinematics::initialize_();

    //Copying hint
    Eigen::VectorXd result(Kinematics::model_.nq);
    result.setZero();
    for (size_t i = 0; i < 7; i++) result(Kinematics::robot_joint_ids_[i] - 1) = hint.values[joint_position[i]];

    //Transforming to Eigen
    Eigen::Vector3d translation;
    for (size_t i = 0; i < 3; i++) translation(i) = states.get(cartesian_position[i]);
    Eigen::Matrix3d rotation(
        Eigen::AngleAxisd(states.get(cartesian_orientation[0]), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(states.get(cartesian_orientation[1]), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(states.get(cartesian_orientation[2]), Eigen::Vector3d::UnitX()));

    //Fix: rotating -45 degree around "forward" and translation -0.2 along "forward"
    Eigen::Vector3d forward = rotation.col(2);
    rotation = Eigen::AngleAxisd(-M_PI / 4, forward) * rotation;
    translation -= 0.2 * forward;
    const pinocchio::SE3 goal(rotation, translation);
    
    //Constants
    const double tolerance   = 1e-4;
    const int max_iteration  = 1000;
    const double step        = 1e-1;
    const double damp        = 1e-6;

    //Preallocation
    pinocchio::Data::Matrix6x jacobian(6, Kinematics::model_.nv);
    jacobian.setZero();
    Eigen::Matrix<double, 6, 1> error;
    Eigen::VectorXd gradient(Kinematics::model_.nv);
    pinocchio::Data::Matrix6 jacobian2;
    pinocchio::SE3 wtf; //What is it?

    //Loop
    for (size_t i = 0; i < max_iteration; i++)
    {
        pinocchio::forwardKinematics(Kinematics::model_, Kinematics::data_, result);
        wtf = goal.actInv(Kinematics::data_.oMi[Kinematics::robot_joint_ids_[6]]);
        error = pinocchio::log6(wtf).toVector();
        if (error.norm() < tolerance)
        {
            for (size_t j = 0; j < 7; j++) states.set(joint_position[j], result(j));
            return;
        }
        pinocchio::computeJointJacobian(Kinematics::model_, Kinematics::data_, result, Kinematics::robot_joint_ids_[6], jacobian);
        jacobian2.noalias() = jacobian * jacobian.transpose();
        jacobian2.diagonal().array() += damp;
        gradient.noalias() = -jacobian.transpose() * jacobian2.ldlt().solve(error);
        result = pinocchio::integrate(Kinematics::model_, result, gradient * step);
    }
    throw std::runtime_error("franka_o80::to_joint: Number of iterations exeeded");
}