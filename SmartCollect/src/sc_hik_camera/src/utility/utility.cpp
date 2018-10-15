#include "utility.hpp"

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}

cv::Mat Utility::toCvMatd(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_64F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<double>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Utility::toCvMatd(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_64F);
    for(int i=0;i<3;i++)
        cvMat.at<double>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Utility::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Utility::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
        cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}