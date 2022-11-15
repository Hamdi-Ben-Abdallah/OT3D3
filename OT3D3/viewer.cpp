#include "viewer.h"


void ContourViewer::Init(View* view, const std::vector<Model*>& objects)
{
	this->m_renderer = view;
	this->m_objects = objects;
}

cv::Mat ContourViewer::DrawOverlay(const cv::Mat& _iFrame, const GLenum _iPolygonMode, const std::string _iColor)
{
	cv::Vec3b color = cv::Vec3b(0, 255, 0);
	{
		int r, g, b;
		char const* hexColor = _iColor.c_str();
		auto item = std::sscanf(hexColor, "#%02x%02x%02x", &r, &g, &b);
		cv::Vec3b color = cv::Vec3b(r, g, b);
	}
	
	this->m_renderer->setLevel(0);
	
	this->m_renderer->RenderSilhouette(std::vector<Model*>(this->m_objects.begin(), this->m_objects.end()), _iPolygonMode);

	cv::Mat depth_map = this->m_renderer->DownloadFrame(View::DEPTH);
	cv::Mat masks_map;
	if (this->m_objects.size() > 1)
	{
		masks_map = this->m_renderer->DownloadFrame(View::MASK);
	}
	else
	{
		masks_map = depth_map;
	}

	cv::Mat result = _iFrame.clone();

	for (int oid = 0; oid < this->m_objects.size(); oid++)
	{
		cv::Mat mask_map;
		this->m_renderer->ConvertMask(masks_map, mask_map, this->m_objects[oid]->getModelID());

		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(mask_map, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		color = cv::Vec3b(0, 255, 0);

		for (auto contour : contours)
			for (auto pt : contour)
			{
				result.at<cv::Vec3b>(pt) = color;
			}
	}

	return result;
}




void MeshViewer::Init(View* view, const std::vector<Model*>& objects)
{
	this->m_renderer = view;
	this->m_objects = objects;
}

cv::Mat MeshViewer::DrawOverlay(const cv::Mat& _iFrame, const GLenum _iPolygonMode, const std::string _iColor)
{
	std::vector<cv::Point3f> colors;
	{
		int r, g, b;
		char const* hexColor = _iColor.c_str();
		auto item = std::sscanf(hexColor, "#%02x%02x%02x", &r, &g, &b);
		colors.push_back(cv::Point3f(r / 255.0f, g / 255.0f, b / 255.0f));
	}

	this->m_renderer->setLevel(0);

	this->m_renderer->RenderShaded(std::vector<Model*>(this->m_objects.begin(), this->m_objects.end()), GL_FILL, colors, true);
	//this->m_renderer->RenderNormals(std::vector<Model*>(this->m_objects.begin(), this->m_objects.end()), GL_FILL);
	
	cv::Mat result = _iFrame.clone();
	this->m_renderer->RenderCV(this->m_objects[0], result);
	
	this->m_renderer->RenderCV(this->m_objects[0], result, cv::Scalar(1,255,1));
	
	return result;
}



void FragmentViewer::Init(View* view, const std::vector<Model*>& objects)
{
	this->m_renderer = view;
	this->m_objects = objects;
}

cv::Mat FragmentViewer::DrawOverlay(const cv::Mat& _iFrame, const GLenum _iPolygonMode, const std::string _iColor)
{
	std::vector<cv::Point3f> colors;
	{
		int r, g, b;
		char const* hexColor = _iColor.c_str();
		auto item = std::sscanf(hexColor, "#%02x%02x%02x", &r, &g, &b);
		colors.push_back(cv::Point3f(r / 255.0f, g / 255.0f, b / 255.0f));
	}
	
	// render the models with phong shading
	this->m_renderer->setLevel(0);
	
	m_renderer->RenderShaded(std::vector<Model*>(this->m_objects.begin(), this->m_objects.end()), _iPolygonMode, colors, true);
	//m_renderer->RenderNormals(std::vector<Model*>(objects.begin(), objects.end()), GL_FILL);

	// download the rendering to the CPU
	cv::Mat rendering = m_renderer->DownloadFrame(View::RGB);

	// download the depth buffer to the CPU
	cv::Mat depth = m_renderer->DownloadFrame(View::DEPTH);

	// compose the rendering with the current camera image for demo purposes (can be done more efficiently directly in OpenGL)
	cv::Mat result = _iFrame.clone();
	for (int y = 0; y < _iFrame.rows; y++)
		for (int x = 0; x < _iFrame.cols; x++)
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
