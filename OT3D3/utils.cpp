#include "utils.h"


bool readCameraParameters(const std::string filename, int& imgWidth, int& imgHeight,
                          cv::Mat& camera_matrix, cv::Mat& distortion_coefficients,
                          cv::Matx33f& K, cv::Matx14f& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;

    // height and widht
    fs["image_width"] >> imgWidth;

    fs["image_height"] >> imgHeight;

    // camera matrix
    fs["camera_matrix"] >> camera_matrix;

    float fx = camera_matrix.at<double>(0, 0);
    float cx = camera_matrix.at<double>(0, 2);
    float fy = camera_matrix.at<double>(1, 1);
    float cy = camera_matrix.at<double>(1, 2);
    K = cv::Matx33f(fx, 0, cx, 0, fy, cy, 0, 0, 1);


    // distortion coefficients
    fs["distortion_coefficients"] >> distortion_coefficients;

    float k1 = distortion_coefficients.at<double>(0, 0);
    float k2 = distortion_coefficients.at<double>(0, 1);
    float p1 = distortion_coefficients.at<double>(0, 2);
    float p2 = distortion_coefficients.at<double>(0, 3);
    float k3 = distortion_coefficients.at<double>(0, 4);
    distCoeffs = cv::Matx14f(k1, k2, p1, p2);

    return true;

    fs.release();
}


bool readInitialPose(const std::string filename, cv::Mat& initial_pose)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;

    fs["initial_pose"] >> initial_pose;

    fs.release();

    return true;
}

bool readInitialPose(const std::string filename, cv::Matx44f& initial_pose)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;

    cv::Mat pose;
    fs["initial_pose"] >> pose;

    initial_pose = cv::Matx44f(pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), pose.at<double>(0, 3),
                               pose.at<double>(1, 0), pose.at<double>(1, 1), pose.at<double>(1, 2), pose.at<double>(1, 3),
                               pose.at<double>(2, 0), pose.at<double>(2, 1), pose.at<double>(2, 2), pose.at<double>(2, 3),
                               pose.at<double>(3, 0), pose.at<double>(3, 1), pose.at<double>(3, 2), pose.at<double>(3, 3));

    fs.release();

    return true;
}

bool IsFileExist(const std::string& name)
{
    std::ifstream f(name.c_str());
    return f.good();
}
