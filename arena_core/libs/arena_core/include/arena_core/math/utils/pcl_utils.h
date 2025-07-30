#pragma once

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>


namespace pcl
{

template <typename PointT, typename Scalar> inline unsigned int
computeCentroidAndOBB (const pcl::PointCloud<PointT> &cloud,
Eigen::Matrix<Scalar, 3, 1> &centroid,
Eigen::Matrix<Scalar, 3, 1> &obb_center,
Eigen::Matrix<Scalar, 3, 1> &obb_dimensions,
Eigen::Matrix<Scalar, 3, 3> &obb_rotational_matrix)
{
    Eigen::Matrix<Scalar, 3, 3> covariance_matrix;
    Eigen::Matrix<Scalar, 4, 1> centroid4;
    const auto point_count = computeMeanAndCovarianceMatrix(cloud, covariance_matrix, centroid4);
    if (!point_count)
        return (0);
    centroid(0) = centroid4(0);
    centroid(1) = centroid4(1);
    centroid(2) = centroid4(2);

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 3, 3>> evd(covariance_matrix);
    const Eigen::Matrix<Scalar, 3, 3> eigenvectors_ = evd.eigenvectors();
    const Eigen::Matrix<Scalar, 3, 1> minor_axis = eigenvectors_.col(0);//the eigenvectors do not need to be normalized (they are already)
    const Eigen::Matrix<Scalar, 3, 1> middle_axis = eigenvectors_.col(1);
    // Enforce right hand rule:
    const Eigen::Matrix<Scalar, 3, 1> major_axis = middle_axis.cross(minor_axis);

    obb_rotational_matrix <<
        major_axis(0), middle_axis(0), minor_axis(0),
        major_axis(1), middle_axis(1), minor_axis(1),
        major_axis(2), middle_axis(2), minor_axis(2);
    //obb_rotational_matrix.col(0)==major_axis
    //obb_rotational_matrix.col(1)==middle_axis
    //obb_rotational_matrix.col(2)==minor_axis

    //Transforming the point cloud in the (Centroid, ma-mi-mi_axis) reference
    //with homogeneous matrix
    //[R^t  , -R^t*Centroid ]
    //[0    , 1             ]
    Eigen::Matrix<Scalar, 4, 4> transform = Eigen::Matrix<Scalar, 4, 4>::Identity();
    transform.template topLeftCorner<3, 3>() = obb_rotational_matrix.transpose();
    transform.template topRightCorner<3, 1>() =-transform.template topLeftCorner<3, 3>()*centroid;

    //when Scalar==double on a Windows 10 machine and MSVS:
    //if you substitute the following Scalars with floats you get a 20% worse processing time, if with 2 PointT 55% worse
    Scalar obb_min_pointx, obb_min_pointy, obb_min_pointz;
    Scalar obb_max_pointx, obb_max_pointy, obb_max_pointz;
    obb_min_pointx = obb_min_pointy = obb_min_pointz = std::numeric_limits<Scalar>::max();
    obb_max_pointx = obb_max_pointy = obb_max_pointz = std::numeric_limits<Scalar>::min();

    if (cloud.is_dense)
    {
        const auto& point = cloud[0];
        Eigen::Matrix<Scalar, 4, 1> P0(static_cast<Scalar>(point.x), static_cast<Scalar>(point.y) , static_cast<Scalar>(point.z), 1.0);
        Eigen::Matrix<Scalar, 4, 1> P = transform * P0;

        obb_min_pointx = obb_max_pointx = P(0);
        obb_min_pointy = obb_max_pointy = P(1);
        obb_min_pointz = obb_max_pointz = P(2);

        for (size_t i=1; i<cloud.size();++i)
        {
        const auto&  point = cloud[i];
        Eigen::Matrix<Scalar, 4, 1> P0(static_cast<Scalar>(point.x), static_cast<Scalar>(point.y) , static_cast<Scalar>(point.z), 1.0);
        Eigen::Matrix<Scalar, 4, 1> P = transform * P0;

        if (P(0) < obb_min_pointx)
            obb_min_pointx = P(0);
        else if (P(0) > obb_max_pointx)
            obb_max_pointx = P(0);
        if (P(1) < obb_min_pointy)
            obb_min_pointy = P(1);
        else if (P(1) > obb_max_pointy)
            obb_max_pointy = P(1);
        if (P(2) < obb_min_pointz)
            obb_min_pointz = P(2);
        else if (P(2) > obb_max_pointz)
            obb_max_pointz = P(2);
        }
    }
    else
    {
        size_t i = 0;
        for (; i < cloud.size(); ++i)
        {
        const auto&  point = cloud[i];
        if (!isFinite(point))
            continue;
        Eigen::Matrix<Scalar, 4, 1> P0(static_cast<Scalar>(point.x), static_cast<Scalar>(point.y) , static_cast<Scalar>(point.z), 1.0);
        Eigen::Matrix<Scalar, 4, 1> P = transform * P0;

        obb_min_pointx = obb_max_pointx = P(0);
        obb_min_pointy = obb_max_pointy = P(1);
        obb_min_pointz = obb_max_pointz = P(2);
        ++i;
        break;
        }

        for (; i<cloud.size();++i)
        {
        const auto&  point = cloud[i];
        if (!isFinite(point))
            continue;
        Eigen::Matrix<Scalar, 4, 1> P0(static_cast<Scalar>(point.x), static_cast<Scalar>(point.y) , static_cast<Scalar>(point.z), 1.0);
        Eigen::Matrix<Scalar, 4, 1> P = transform * P0;

        if (P(0) < obb_min_pointx)
            obb_min_pointx = P(0);
        else if (P(0) > obb_max_pointx)
            obb_max_pointx = P(0);
        if (P(1) < obb_min_pointy)
            obb_min_pointy = P(1);
        else if (P(1) > obb_max_pointy)
            obb_max_pointy = P(1);
        if (P(2) < obb_min_pointz)
            obb_min_pointz = P(2);
        else if (P(2) > obb_max_pointz)
            obb_max_pointz = P(2);
        }

    }

    const Eigen::Matrix<Scalar, 3, 1>  //shift between point cloud centroid and OBB center (position of the OBB center relative to (p.c.centroid, major_axis, middle_axis, minor_axis))
        shift((obb_max_pointx + obb_min_pointx) / 2.0f,
        (obb_max_pointy + obb_min_pointy) / 2.0f,
        (obb_max_pointz + obb_min_pointz) / 2.0f);

    obb_dimensions(0) = obb_max_pointx - obb_min_pointx;
    obb_dimensions(1) = obb_max_pointy - obb_min_pointy;
    obb_dimensions(2) = obb_max_pointz - obb_min_pointz;

    obb_center = centroid + obb_rotational_matrix * shift;//position of the OBB center in the same reference Oxyz of the point cloud

    return (point_count);
}

}; // namespace pcl
