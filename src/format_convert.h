#ifndef __FORMAT_CONVERT_H__
#define __FORMAT_CONVERT_H__

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <svo/common/frame.h>

inline Eigen::MatrixXd svo_trans_to_eigen_mat(const vk::Transformation& trans)
{
    return trans.getTransformationMatrix();
}

inline cv::Mat eigen_mat_to_cv_mat(const Eigen::MatrixXd& eigen_mat)
{
    cv::Mat cv_mat;
    eigen2cv(eigen_mat, cv_mat);
    return cv_mat;
}

inline cv::Mat svo_trans_to_cv_mat(const vk::Transformation& trans)
{
    return eigen_mat_to_cv_mat(svo_trans_to_eigen_mat(trans));
}

inline cv::Affine3f cv_mat_to_affine3f(const cv::Mat& mat)
{
    cv::Mat m;
    mat.convertTo(m, CV_32FC1);
    return cv::Affine3f(m);
}

inline cv::Affine3f svo_trans_to_cv_affine(const vk::Transformation& trans)
{
    cv::Mat m = svo_trans_to_cv_mat(trans);
    m.convertTo(m, CV_32FC1);
    return cv::Affine3f(m);
}


#endif//__FORMAT_CONVERT_H__