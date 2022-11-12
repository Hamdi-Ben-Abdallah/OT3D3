#include <cmath>
#include <sstream>
#include <fstream>
#include <iostream>

#include <QThread>
#include <QApplication>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <spdlog/spdlog.h>

#include "view.h"
#include "utils.h" 
#include "tracker.h"
#include "object3d.h"
#include "global_params.h"



cv::Mat drawContourOverlay(View* view, const std::vector<Model*>& objects, const cv::Mat& frame) 
{
	view->setLevel(0);
	view->RenderSilhouette(std::vector<Model*>(objects.begin(), objects.end()), GL_FILL);

	cv::Mat depth_map = view->DownloadFrame(View::DEPTH);
	cv::Mat masks_map;
	if (objects.size() > 1) 
	{
		masks_map = view->DownloadFrame(View::MASK);
	}
	else 
	{
		masks_map = depth_map;
	}

	cv::Mat result = frame.clone();

	for (int oid = 0; oid < objects.size(); oid++) 
	{
		cv::Mat mask_map;
		view->ConvertMask(masks_map, mask_map, objects[oid]->getModelID());

		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(mask_map, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		cv::Vec3b color;
		if (0 == oid)
			color = cv::Vec3b(0, 255, 0);
		if (1 == oid)
			color = cv::Vec3b(0, 0, 255);

		for (auto contour : contours)
			for (auto pt : contour) 
			{
				result.at<cv::Vec3b>(pt) = color;
			}
	}

	return result;
}

cv::Mat drawMeshOverlay(View* view, const std::vector<Model*>& objects, const cv::Mat& frame) 
{
	view->setLevel(0);

	std::vector<cv::Point3f> colors;
	colors.push_back(cv::Point3f(1.0, 0.5, 0.0));
	colors.push_back(cv::Point3f(0.2, 0.3, 1.0));
	//RenderShaded(std::vector<Model*>(objects.begin(), objects.end()), GL_FILL, colors, true);
	//RenderNormals(std::vector<Model*>(objects.begin(), objects.end()), GL_FILL);
	cv::Mat result = frame.clone();
	view->RenderCV(objects[0], result);
	//RenderCV(objects[0], result, cv::Scalar(1,255,1));
	//RenderCV(objects[1], result, cv::Scalar(1,1,255));
	return result;
}

cv::Mat drawFragmentOverlay(View* view, const std::vector<Model*>& objects, const cv::Mat& frame) 
{
	// render the models with phong shading
	view->setLevel(0);

	std::vector<cv::Point3f> colors;
	colors.push_back(cv::Point3f(1.0, 0.5, 0.0));
	//colors.push_back(cv::Point3f(0.0, 1.0, 0.0));
	//colors.push_back(Point3f(0.2, 0.3, 0.0));
	view->RenderShaded(std::vector<Model*>(objects.begin(), objects.end()), GL_FILL, colors, true);
	//RenderNormals(std::vector<Model*>(objects.begin(), objects.end()), GL_FILL);

	// download the rendering to the CPU
	cv::Mat rendering = view->DownloadFrame(View::RGB);

	// download the depth buffer to the CPU
	cv::Mat depth = view->DownloadFrame(View::DEPTH);

	// compose the rendering with the current camera image for demo purposes (can be done more efficiently directly in OpenGL)
	cv::Mat result = frame.clone();
	for (int y = 0; y < frame.rows; y++)
		for (int x = 0; x < frame.cols; x++) 
		{
			cv::Vec3b color = rendering.at<cv::Vec3b>(y, x);
			if (depth.at<float>(y, x) != 0.0f) 
			{
				result.at<cv::Vec3b>(y, x)[0] = color[2];
				result.at<cv::Vec3b>(y, x)[1] = color[1];
				result.at<cv::Vec3b>(y, x)[2] = color[0];
			}
		}
	return result;
}


int main(int argc, char* argv[]) 
{
	QCoreApplication::addLibraryPath("plugins");
	QApplication a(argc, argv);

	spdlog::set_level(spdlog::level::debug); // Set global log level to debug
	spdlog::info("OT3D-3.0.0");

	OT3D::GlobalParam* gp = OT3D::GlobalParam::Instance();
	gp->ParseConfig(argv[1]);

	//////////////////////////////////////////////// Read camera parameters ////////////////////////////////////////////////

	spdlog::debug("Read camera parameters...");
	cv::Matx33f K;
	cv::Matx14f D;
	int image_width, image_height;
	cv::Mat camera_matrix, distortion_coefficients;

	std::string filename = gp->projDir + "/camera_calibration.yaml";
	bool readOk = readCameraParameters(filename, image_width, image_height, camera_matrix, distortion_coefficients, K, D);
	if (!readOk)
	{
		spdlog::critical("Invalid camera file");
		exit(-1);
	}

	//////////////////////////////////////////////// Read initial pose ////////////////////////////////////////////////

	spdlog::debug("Read initial pose...");
	cv::Matx44f initial_pose;

	filename = gp->projDir + "/Resources/initial_pose.yaml";
	readOk = readInitialPose(filename, initial_pose);
	if (!readOk)
	{
		spdlog::critical("Invalid initial pose");
		exit(-1);
	}

	//////////////////////////////////////////////// Load 3D objects ////////////////////////////////////////////////

	spdlog::debug("Load 3D objects...");

	// distances for the pose detection template generation
	std::vector<float> distances = { 200.0f, 400.0f, 600.0f };

	// load 3D objects
	std::vector<Object3D*> objects;

	filename = gp->projDir + "/Resources/Model.obj";
	objects.push_back(new Object3D(filename, initial_pose, gp->scale, gp->qualityThreshold, distances));


	//////////////////////////////////////////////// Initialize view ////////////////////////////////////////////////

	spdlog::debug("Initialize view...");

	View* view = View::Instance();
	view->init(K, image_width, image_height, gp->zn, gp->zf, 4);

	//////////////////////////////////////////////// Initialize tracker ////////////////////////////////////////////////

	spdlog::debug("Initialize tracker...");

	std::shared_ptr<Tracker> tracker_ptr(Tracker::GetTracker(K, D, objects));


	//////////////////////////////////////////////// Open video ////////////////////////////////////////////////

	// Open video
	spdlog::debug("Open video...");
	filename = gp->projDir + "/OT3DVideoData/Video.mp4";
	cv::VideoCapture cap(filename);
	if (!cap.isOpened())
	{
		spdlog::critical("Cannot open video");
		exit(-1);
	}

	spdlog::debug("Localization algorithm is ready. Click on 'SPACE' to start.");

	// Create a display window
	cv::namedWindow("OT3D", 1);

	cv::Mat frame;
	int timeout = 0;
	
	while (true) 
	{
		cap >> frame;

		// if the frame is empty, break immediately
		if (frame.empty())
		{
			spdlog::error("Frame is empty.");
			break;
		}

		auto e1 = cv::getTickCount();

		// the main pose uodate call
		tracker_ptr->EstimatePoses(frame, true);
		tracker_ptr->PostProcess(frame);

		auto e2 = cv::getTickCount();
		auto time = 1000 * ((e2 - e1) / cv::getTickFrequency());

		cv::Mat result = drawFragmentOverlay(view, std::vector<Model*>(objects.begin(), objects.end()), frame);

		cv::putText(result, cv::format("Time: %3.2f ms", time), cv::Point(5, 35), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

		if (objects[0]->isTrackingLost())
			cv::putText(result, "Tracking is lost: relocation...", cv::Point(5, 70), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);


		cv::imshow("OT3D", result);

		int key = cv::waitKey(timeout);

		if (key == 32) // Space: start/stop tracking
		{
			timeout = 1;
			tracker_ptr->ToggleTracking(0, false);
			tracker_ptr->EstimatePoses(frame, false);
		}

		if (27 == key)
			break;

		if ('r' == key) {
			for (int i = 0; i < objects.size(); ++i) 
			{
				objects[i]->setPose(initial_pose);
			}
		}
	}

	// Closes all the frames
	cv::destroyAllWindows();
	// When everything done, release the video capture
	cap.release();
	
	// clean up
	View::Instance()->destroy();

	for (int i = 0; i < objects.size(); i++) 
	{
		delete objects[i];
	}
	objects.clear();

	spdlog::debug("Program is closed.");
}
