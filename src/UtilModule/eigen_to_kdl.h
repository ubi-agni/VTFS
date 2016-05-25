#ifndef EIGEN_TO_KDL_H
#define EIGEN_TO_KDL_H

#include <kdl/frames.hpp>
#include <Eigen/Dense>

namespace conversions
{

inline void EigenVectorTokdlVector(const Eigen::Vector3d& eigen_vector,
                                   KDL::Vector& kdl_vector)
{
    for (int i=0; i<3; ++i)
    {
        kdl_vector[i] = eigen_vector[i];
    }
}

inline void EigenMatrix3dTokdlRotation(const Eigen::Matrix3d& eigen_matrix,
                                       KDL::Rotation& kdl_rotation)
{
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j)
            kdl_rotation(i,j) = eigen_matrix(i,j);
}

inline void convert(const Eigen::Vector3d& eigen_vector,
                    KDL::Vector& kdl_vector)
{
    EigenVectorTokdlVector(eigen_vector,kdl_vector);
}

inline void convert(const Eigen::Matrix3d& eigen_matrix,
                    KDL::Rotation& kdl_rotation)
{
    EigenMatrix3dTokdlRotation(eigen_matrix,kdl_rotation);
}

}


#endif // EIGEN_TO_KDL_H
