#pragma once

#include "HelperFunctions.hpp"
#include "DataParser.hpp"
#include <Eigen/Dense>

namespace axes_ident
{

class Identification
{
private:
    DataParser::Data data;
    Eigen::Matrix<double, 3, Eigen::Dynamic> axes;

    unsigned int n_joints;

    void _resizeAxes(unsigned int n_joints);
    bool _checkNJoints();

public:
    Identification(unsigned int n_joints);

    /**
     * @brief Set the Data object
     * 
     * @param data M x (robot.getNJoints() + 4) matrix containing experimental data,
     * where each row is a single measurement [theta, rpy_angle, n_joint]:
     *      theta: 1 x N vector with encoder measurements
     *      rpy_angle: [roll, pitch, yaw] acquired from the IMU
     *      n_joint: indicates which joint moved to arrive at this joint
     *               configuration starting from the one described by the
     *               last row
     * 
     * @return true Data successfully stored
     * @return false Data did not meet the required standards
     */
    bool setData(const DataParser::Data &data);

    void identifyAxes(bool start_from_last = false);
};
}