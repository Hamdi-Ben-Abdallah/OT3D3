#ifndef GLOBAL_PARAMS
#define GLOBAL_PARAMS

#include <map>
#include <string>
#include <vector>

namespace OT3D
{
	class GlobalParam 
	{
	public:
		static GlobalParam* Instance();
		void ParseConfig(const std::string& config_file);

		std::string projDir;

		float zn = 10.0f;
		float zf = 10000.0f;

		float scale = 1.0f;
		float qualityThreshold = 0.55f;

		std::string color;
		
	protected:
		GlobalParam();

	private:
		static GlobalParam* instance;
	};

} // namespace OT3D

#endif /* GLOBAL_PARAMS */