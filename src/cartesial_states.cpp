#include "../include/franka_o80/cartesial_states.hpp"

bool franka_o80::CartesialStates::initialized_ = false;
size_t franka_o80::CartesialStates::robot_joint_ids_[7];
pinocchio::Model franka_o80::CartesialStates::model_;
pinocchio::Data franka_o80::CartesialStates::data_;

franka_o80::CartesialStates::CartesialStates()
{
    if (!initialized_)
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

    for (size_t i = 0; i < 22; i++) values[i] = 0.0;
}

void franka_o80::CartesialStates::set(int actuator, State state)
{
    values[actuator] = state;
}

franka_o80::State franka_o80::CartesialStates::get(int actuator) const
{
    return values[actuator];
}

franka_o80::CartesialStates franka_o80::to_cartesial(const States &states)
{
    //Joint space
    Eigen::VectorXd positions(CartesialStates::model_.nq);
    positions.setZero();
    for (size_t i = 0; i < 7; i++) positions(CartesialStates::robot_joint_ids_[i] - 1) = states.get(robot_positions[i]); //-1 because of universe joint does not go here

    //Cartesial space
    pinocchio::forwardKinematics(CartesialStates::model_, CartesialStates::data_, positions);
    pinocchio::SE3 se3 = CartesialStates::data_.oMi[CartesialStates::robot_joint_ids_[6]];
    
    //Rotationg upright 45 degree around translation and translating translation
    Eigen::Matrix3d rotation = se3.rotation_impl();
    Eigen::Vector3d forward = rotation.col(2);
    Eigen::Vector3d upright = rotation.col(0);
    Eigen::Vector3d right = Eigen::AngleAxisd(M_PI / 4, forward) * upright;
    Eigen::Vector3d translation = se3.translation_impl() + 0.2 * forward;

    //Returning result
    CartesialStates result;
    for (size_t i = 0; i < 7; i++) result.values[i] = states.values[i];
    for (size_t i = 0; i < 3; i++)
    {
        result.values[cartesial_positions[i]] = translation(i);
        result.values[cartesial_velocities[i]] = 0.0;
        result.values[cartesial_forces[i]] = 0.0;
        result.values[cartesial_forward[i]] = forward(i);
        result.values[cartesial_right[i]] = right(i);
    }
    return result;
}

franka_o80::States franka_o80::to_joint(const CartesialStates &cartesial, double position0)
{
    return to_joint(cartesial, default_states(), position0);
}

franka_o80::States franka_o80::to_joint(const CartesialStates &cartesial, const States &hint, double position0)
{
    //Copying hint
    Eigen::VectorXd result(CartesialStates::model_.nq);
    result.setZero();
    for (size_t i = 0; i < 7; i++) result(CartesialStates::robot_joint_ids_[i] - 1) = hint.values[robot_positions[i]];

    //Copying values
    Eigen::Vector3d translation;
    Eigen::Vector3d forward;
    Eigen::Vector3d right;
    for (size_t i = 0; i < 3; i++)
    {
        translation(i) = cartesial.values[cartesial_positions[i]];
        forward(i) = cartesial.values[cartesial_forward[i]];
        right(i) = cartesial.values[cartesial_right[i]];
    }
    forward.normalize();
    right -= forward * forward.dot(right);
    right.normalize();
    translation -= 0.2 * forward;

    //Calculating goal (rotating and translating)
    Eigen::Vector3d upright = Eigen::AngleAxisd(-M_PI / 4, forward) * right;
    Eigen::Vector3d downright = forward.cross(upright);
    Eigen::Matrix3d rotation; rotation.col(0) = upright; rotation.col(1) = downright; rotation.col(2) = forward;
    const pinocchio::SE3 goal(rotation, translation);

    //Constants
    const double tolerance   = 1e-4;
    const int max_iteration  = 1000;
    const double step        = 1e-1;
    const double damp        = 1e-6;

    //Preallocation
    pinocchio::Data::Matrix6x jacobian(6, CartesialStates::model_.nv);
    jacobian.setZero();
    Eigen::Matrix<double, 6, 1> error;
    Eigen::VectorXd gradient(CartesialStates::model_.nv);
    pinocchio::Data::Matrix6 jacobian2;
    pinocchio::SE3 wtf; //What is it?

    //Loop
    for (size_t i = 0; i < max_iteration; i++)
    {
        pinocchio::forwardKinematics(CartesialStates::model_, CartesialStates::data_, result);
        wtf = goal.actInv(CartesialStates::data_.oMi[CartesialStates::robot_joint_ids_[6]]);
        error = pinocchio::log6(wtf).toVector();
        if (error.norm() < tolerance)
        {
            //Returning result
            States sresult;
            for (size_t j = 0; j < 7; j++) sresult.values[j] = result(j);
            for (size_t j = 0; j < 7; j++)
            {
                sresult.values[robot_positions[j]] = result(j);
                sresult.values[robot_velocities[j]] = 0.0;
                sresult.values[robot_torques[j]] = 0.0;
            }
            return sresult;
        }
        pinocchio::computeJointJacobian(CartesialStates::model_, CartesialStates::data_, result, CartesialStates::robot_joint_ids_[6], jacobian);
        jacobian2.noalias() = jacobian * jacobian.transpose();
        jacobian2.diagonal().array() += damp;
        gradient.noalias() = -jacobian.transpose() * jacobian2.ldlt().solve(error);
        result = pinocchio::integrate(CartesialStates::model_, result, gradient * step);
    }
    throw std::runtime_error("franka_o80::to_joint: Number of iterations exeeded");
}