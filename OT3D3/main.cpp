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
#include "viewer.h"
#include "tracker.h"
#include "object3d.h"
#include "global_params.h"


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


	auto viewer_ptr = std::make_shared<FragmentViewer>();
	viewer_ptr->Init(view, std::vector<Model*>(objects.begin(), objects.end()));
	spdlog::debug("Initialize viewer...");

	spdlog::debug("Localization algorithm is ready. Click on 'SPACE' to start.");

	// Create a display window
	cv::namedWindow("OT3D", 1);

	cv::Mat frame;
	int timeout = 0;
	bool stopTracking = false;
	GLenum polygonMode = GL_FILL;
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
		tracker_ptr->EstimatePoses(frame, false);
		tracker_ptr->PostProcess(frame);

		auto e2 = cv::getTickCount();
		auto time = 1000 * ((e2 - e1) / cv::getTickFrequency());

		cv::Mat result = viewer_ptr->DrawOverlay(frame, polygonMode, gp->color);

		cv::putText(result, cv::format("Time: %3.2f ms", time), cv::Point(5, 35), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

		if (objects[0]->isTrackingLost())
			cv::putText(result, "Tracking is lost: relocation...", cv::Point(5, 70), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);


		cv::imshow("OT3D", result);

		int key = cv::waitKey(timeout);

		if (key == 16) // Pause video (ctrl + p)
		{
			stopTracking = !stopTracking;
			if (stopTracking)
				timeout = 0;
			else
				timeout = 1;
		}

		if (key == (int)'w' || key == (int)'W') // Modify the representation of all actors so that they are wireframe.
		{
			if (polygonMode == GL_LINE)
				polygonMode = GL_FILL;
			else
				polygonMode = GL_LINE;
		}
		if (key == (int)'s' || key == (int)'S') // Modify the representation of all actors so that they are surfaces.
		{
			polygonMode = GL_FILL;
		}
		if (key == (int)'p' || key == (int)'P') // Modify the representation of all actors so that they are points.
		{
			if (polygonMode == GL_POINT)
				polygonMode = GL_FILL;
			else
				polygonMode = GL_POINT;
		}

		if (key == 32) // Space: start/stop tracking
		{
			timeout = 1;
			tracker_ptr->ToggleTracking(frame, 0, false);
			tracker_ptr->EstimatePoses(frame, false);
		}

		if (27 == key)
			break;

		if ('r' == key) 
		{
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
