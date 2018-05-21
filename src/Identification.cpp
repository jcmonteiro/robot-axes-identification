#include "Identification.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>

using namespace axes_ident;

Identification::Identification(unsigned int n_joints) :
    n_joints(n_joints)
{
    this->_resizeAxes(n_joints);
}

void Identification::_resizeAxes(unsigned int n_joints)
{
    this->n_joints = n_joints;
    axes = Eigen::MatrixXd(3, n_joints);
}

bool Identification::_checkNJoints()
{
    return this->n_joints >= 1;
}

bool Identification::setData(const DataParser &parser)
{
    if (!this->_checkNJoints())
    {
        std::cerr << "[Warn] The number of joints was not correctly set. Changing it to " <<
            this->n_joints << " to match the data." << std::endl;
    }
    if (!parser.check())
    {
        std::cerr << "[Error] Parser contains errors. Identification algorithm was not configured." << std::endl;
        return false;
    }
    const DataParser::Data &data = parser.getDataByJoint()[0];
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
    
    this->data = parser.getDataByJoint();
    return true;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Identification::identifyAxes(bool start_from_last)
{
    // Housekeeping before main algorithm
    auto I = Eigen::Matrix3d::Identity();
    //
    unsigned int ind_data_roll = n_joints;
    unsigned int ind_data_pitch = ind_data_roll + 1;
    unsigned int ind_data_yaw = ind_data_pitch + 1;
    //
    auto rotRPY = [ind_data_roll, ind_data_pitch, ind_data_yaw] (const Eigen::RowVectorXd &row) -> Eigen::Matrix3d
    {
        return HelperFunctions::rotRPY<double>(
            row(ind_data_roll),
            row(ind_data_pitch),
            row(ind_data_yaw),
            false
        );
    };
    //
    Eigen::Matrix<double, 3, Eigen::Dynamic> axes(3, n_joints);
    Eigen::Matrix<double, 3, Eigen::Dynamic> axes_measurements[n_joints];
    for (unsigned int k = 0; k < n_joints; ++k)
    {
        axes_measurements[k].resize(3, data[k].rows() / 2);
    }
    //
    std::vector<unsigned int> ind_joint_order(n_joints);
    std::iota(ind_joint_order.begin(), ind_joint_order.end(), 0);
    if (start_from_last)
        std::reverse(ind_joint_order.begin(), ind_joint_order.end());
    //
    unsigned int counter = 0;
    while (counter < n_joints)
    {
        unsigned int ind_joint = ind_joint_order[counter];
        const DataParser::Data &experiments = data[ind_joint];
        for (unsigned int ind_exp = 0; ind_exp < experiments.rows(); ind_exp += 2)
        {
            const Eigen::RowVectorXd &row_last = experiments.row(ind_exp);
            const Eigen::RowVectorXd &row_curr = experiments.row(ind_exp+1);
            //
            auto Rwe_last = rotRPY(row_last);
            auto Rwe_curr = rotRPY(row_curr);
            //
            Eigen::Matrix3d R = I;
            for (auto iter_other = ind_joint_order.begin() ; iter_other < ind_joint_order.begin() + counter ; ++iter_other)
            {
                if (start_from_last)
                    R = HelperFunctions::rotAngleAxis<double>(row_curr(*iter_other), axes.col(*iter_other)) * R;
                else
                    R = R * HelperFunctions::rotAngleAxis<double>(row_curr(*iter_other), axes.col(*iter_other));
            }
            //
            if (start_from_last)
                R = R * Rwe_last.transpose() * Rwe_curr * R.transpose();
            else
                R = R.transpose() * Rwe_curr * Rwe_last.transpose() * R;
            //
            double delta_angle_ji = row_curr(ind_joint) - row_last(ind_joint);
            //
            axes_measurements[ind_joint].col(ind_exp / 2) = HelperFunctions::axisFromRot<double>(R, delta_angle_ji);
        }
        axes.col(ind_joint) = axes_measurements[ind_joint].rowwise().mean().normalized();
        //
        ++counter;
    }
    return axes;
}