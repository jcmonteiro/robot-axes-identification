#include "Identification.hpp"
#include <iostream>

using namespace axes_ident;

Identification::Identification(int n_joints) :
    n_joints(n_joints)
{
    this->_resizeAxes(n_joints);
}

void Identification::_resizeAxes(int n_joints)
{
    this->n_joints = n_joints;
    axes = Eigen::MatrixXd(3, n_joints);
}

bool Identification::_checkNJoints()
{
    return this->n_joints >= 1;
}

bool Identification::setData(const DataParser::Data &data)
{
    if (!this->_checkNJoints())
    {
        std::cerr << "[Warn] The number of joints was not correctly set. Changing it to " <<
            this->n_joints << " to match the data." << std::endl;
    }
    if (data.cols() != n_joints + 4)
    {
        std::cerr << "[Error] Data columns = " << data.cols() << " , but " <<
            n_joints + 4 << " were expected." << std::endl;
        return false;
    }
    if (data.rows() < 2)
    {
        std::cerr << "[Error] Data should contain at least 2 rows." << std::endl;
        return false;
    }
    else if (data.rows() < n_joints + 1)
    {
        std::cerr << "[Warn] Not enough rows, received " << data.rows() << " when at least " <<
            n_joints + 1 << " were expected. Not all axes will be identified." << std::endl;
    }
    
    this->data = data;
    this->data(0, n_joints - 1) = DataParser::INDEX_INVALID;
    return true;
}

void Identification::identifyAxes(bool)
{
    // Housekeeping before main algorithm
    auto I = Eigen::Matrix3d::Identity();
    //
    unsigned int ind_data_roll = n_joints;
    unsigned int ind_data_pitch = ind_data_roll + 1;
    unsigned int ind_data_yaw = ind_data_pitch + 1;
    const int ind_data_joint = data.cols() - 1;
    //
    unsigned int ind_data = 0;
    Eigen::RowVectorXd row_last = data.row(ind_data), row;
    Eigen::Matrix3d Rwe_last = HelperFunctions::rotRPY<double>(row_last(ind_data_roll), row_last(ind_data_pitch), row_last(ind_data_yaw), false);
    //
    int n_joint_curr = 0, n_joint;
    //
    Eigen::Vector3d axes[n_joints];
    Eigen::Matrix<double, 3, Eigen::Dynamic> axes_measurements[n_joints];
    for (unsigned int k = 0; k < n_joints; ++k)
    {
        int n_joint_k_experiments = (data.col(data.cols() - 1).array() == k).count();
        axes_measurements[k].resize(3, n_joint_k_experiments);
    }

    unsigned int ind_joint = 0;
    while (ind_joint < n_joints)
    {
        n_joint = n_joint_curr;
        int ind_experiment = 0;
        while (++ind_data < data.rows())
        {
            row = data.row(ind_data);
            n_joint_curr = row(ind_data_joint);
            if (n_joint_curr != n_joint)
                break;
            //
            Eigen::Matrix3d R = I;
            for (unsigned int ind_other_joint = 0 ; ind_other_joint < ind_joint ; ++ind_other_joint)
            {
                R = R * HelperFunctions::rotAngleAxis<double>(row(ind_other_joint), axes[ind_other_joint]);
            }
            auto Rwe = HelperFunctions::rotRPY<double>(row(ind_data_roll), row(ind_data_pitch), row(ind_data_yaw), false);
            R = R.transpose() * Rwe * Rwe_last.transpose() * R;
            //
            double delta_angle_ji = row(ind_joint) - row_last(ind_joint);
            //
            axes_measurements[ind_joint].col(ind_experiment++) = HelperFunctions::axisFromRot(R, delta_angle_ji);
            //
            row_last = row;
            Rwe_last = Rwe;
        }
        axes[ind_joint] = axes_measurements[ind_joint].rowwise().mean().normalized();
        std::cout << axes[ind_joint].transpose() << std::endl;
        ++ind_joint;
    }
}