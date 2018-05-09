#pragma once

#include <Eigen/Dense>
#include <iostream>

namespace axes_ident
{

class HelperFunctions
{
public:
    template <class type>
    inline static Eigen::Matrix<type, 3, 3> rotX(double ang)
    {
        double c = cos(ang);
        double s = sin(ang);
        Eigen::Matrix<type, 3, 3> ret;
        ret << 1, 0,  0,
               0, c, -s,
               0, s,  c;
        return ret;
    }

    template <class type>
    inline static Eigen::Matrix<type, 3, 3> rotY(double ang)
    {
        double c = cos(ang);
        double s = sin(ang);
        Eigen::Matrix<type, 3, 3> ret;
        ret << c, 0, s,
               0, 1, 0,
              -s, 0, c;
        return ret;
    }

    template <class type>
    inline static Eigen::Matrix<type, 3, 3> rotZ(double ang)
    {
        double c = cos(ang);
        double s = sin(ang);
        Eigen::Matrix<type, 3, 3> ret;
        ret << c, -s, 0,
               s,  c, 0,
               0,  0, 1;
        return ret;
    }

    template <class type>
    inline static Eigen::Matrix<type, 3, 3> rotRPY(double roll, double pitch, double yaw, bool rpy = true)
    {
        if (rpy)
            return rotX<type>(roll) * rotY<type>(pitch) * rotZ<type>(yaw);
        else
            return rotZ<type>(yaw) * rotY<type>(pitch) * rotX<type>(roll);
    }

    template <class type>
    inline static Eigen::Matrix<type, 3, 3> rotAngleAxis(double ang, const Eigen::Matrix<type, 3, 1> &h)
    {
        double c = cos(ang);
        double s = sin(ang);
        double v = 1 - c;
        Eigen::Matrix<type, 3, 3> ret;
        ret << h(0)*h(0)*v + c,      h(0)*h(1)*v - h(2)*s, h(0)*h(2)*v + h(1)*s,
               h(0)*h(1)*v + h(2)*s, h(1)*h(1)*v + c,      h(1)*h(2)*v - h(0)*s,
               h(0)*h(2)*v - h(1)*s, h(1)*h(2)*v + h(0)*s, h(2)*h(2)*v + c;
        return ret;
    }

    template <class type>
    inline static Eigen::Matrix<type, 3, 1> axisFromRot(const Eigen::Matrix<type, 3, 3> &rot, type delta_theta)
    {
        Eigen::Matrix<type, 3, 1> axis;
        axis << rot(2,1) - rot(1,2) , rot(0,2) - rot(2,0) , rot(1,0) - rot(0,1);
        axis = axis / 2 / sin(delta_theta);
        return axis;
    }
};

}