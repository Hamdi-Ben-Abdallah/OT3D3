#include <iostream>
#include <fstream>
#include <limits>
#include "global_params.h"
#include <spdlog/spdlog.h>
#include <opencv2/core.hpp>

namespace OT3D 
{

	GlobalParam* GlobalParam::instance = NULL;

	GlobalParam::GlobalParam() 
	{

	}

	GlobalParam* GlobalParam::Instance() 
	{
		if (instance == NULL) 
		{
			instance = new GlobalParam();
		}
		return instance;
	}

	template <typename T>
	void ReadValue(cv::FileStorage& fs, const std::string& idx, T& field) 
	{
		cv::FileNode node = fs[idx];
		if (node.empty())
			spdlog::warn("Check {0}{1}{2} {3}", '<', idx, '>', "in config file");
		node >> field;
	}

	template <typename T>
	void ReadArray(cv::FileStorage& fs, const std::string& idx, std::vector<T>& field) 
	{
		cv::FileNode node = fs[idx];
		if (node.empty())
			spdlog::warn("Check {0}{1}{2} {3}", '<', idx, '>', "in config file");
		for (cv::FileNodeIterator it = node.begin(); it != node.end(); it++) 
		{
			field.push_back(T(*it));
		}
	}

	void GlobalParam::ParseConfig(const std::string& config_file) 
	{
		spdlog::info("Parsing config file: {0}", config_file);

		cv::FileStorage fs(config_file, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			spdlog::critical("failed to read config: {0}", config_file);
			exit(-1);
		}

		cv::FileNode node;

		ReadValue(fs, "zn", zn);
		ReadValue(fs, "zf", zf);

		ReadValue(fs, "scale", scale);
		ReadValue(fs, "qualityThreshold", qualityThreshold);

		ReadValue(fs, "projDir", projDir);

		ReadValue(fs, "color", color);

	}

} // namespace tk