#pragma once

#include <opencv2/core.hpp>

#include "view.h"
#include "model.h"

class View;
class Model;


class ContourViewer
{
public:
	void Init(View* view, const std::vector<Model*>& objects);
	ContourViewer() = default;

	cv::Mat DrawOverlay(const cv::Mat& frame, const GLenum polygonMode, const std::string color = "#72e090");

protected:
	View* m_renderer = nullptr;
	std::vector<Model*> m_objects;
};

class MeshViewer
{
public:
	void Init(View* view, const std::vector<Model*>& objects);
	MeshViewer() = default;

	cv::Mat DrawOverlay(const cv::Mat& frame, const GLenum polygonMode, const std::string color = "#72e090");

protected:
	View* m_renderer = nullptr;
	std::vector<Model*> m_objects;
};


class FragmentViewer
{
public:
	void Init(View* view, const std::vector<Model*>& objects);
	FragmentViewer() = default;

	cv::Mat DrawOverlay(const cv::Mat& frame, const GLenum polygonMode, const std::string color = "#72e090");

protected:
	View* m_renderer = nullptr;
	std::vector<Model*> m_objects;
};
