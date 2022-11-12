#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


bool readCameraParameters(const std::string filename, int& imgWidth, int& imgHeight,
						  cv::Mat& camera_matrix, cv::Mat& distortion_coefficients,
						  cv::Matx33f& K, cv::Matx14f& distCoeffs);

bool readInitialPose(const std::string filename, cv::Mat& initial_pose);
bool readInitialPose(const std::string filename, cv::Matx44f& initial_pose);

bool IsFileExist(const std::string& name);