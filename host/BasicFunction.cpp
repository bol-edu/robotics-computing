//#include <unistd.h>
#include <limits.h>
#include <sys/stat.h>
#include <sstream>
#include <cstring>
#include <string>
//#include <dirent.h>
#include "BasicFunction.hpp"
namespace BasicFunction {
std::vector<cl::Device> get_devices_of_platform(const std::string& Target_Platform_Vendor) {
    size_t i;
    std::vector<cl::Platform> platforms;
    std::vector<cl::Device> devices;
    cl::Platform::get(&platforms);
    for (i=0; i<platforms.size(); i++) {
        std::string platformName = platforms[i].getInfo<CL_PLATFORM_NAME>();
        if (platformName==Target_Platform_Vendor) {
            std::cout << "Found Platform Name: " << platformName << std::endl;
            // Get ALL devices of the specific platform
            platforms[i].getDevices(CL_DEVICE_TYPE_ALL, &devices);
            return devices;
        }
    }

    // No corresponding platform, select manually or exit
    if (i==platforms.size()) {
        std::cout << "Error: Failed to find " << Target_Platform_Vendor << " platform" << std::endl;
        std::cout << "Belows are all platforms on this system" << std::endl;
        std::cout << "Please select the platform by entering number, or enter 0 to exit" << std::endl;
        std::cout << "===============Detected Platforms===============" << std::endl;
        for(int j=0; j<platforms.size(); j++){
            std::cout << "[" << j+1 << "] " << platforms[j].getInfo<CL_PLATFORM_NAME>() << std::endl;
        }
        int selected_platform;
        std::cin >> selected_platform;
        if(selected_platform==0) exit(EXIT_FAILURE);        
        else{
            std::vector<cl::Device> devices;
            platforms[selected_platform-1].getDevices(CL_DEVICE_TYPE_ALL, &devices);
            return devices;
        }
    }
}

cl::Device get_specific_device(const std::string& Target_Device_Name, const std::vector<cl::Device>& devices){
    size_t i;
    for(i=0; i<devices.size(); i++){
        std::string deviceName = devices[i].getInfo<CL_DEVICE_NAME>();
        if(deviceName==Target_Device_Name){
            std::cout << "Found Device Name: " << deviceName << std::endl;
            // Get device of the given devices
            return devices[i];
        }
    }
    // No corresponding device, select manually or exit
    if(i==devices.size()){
        std::cout << "Error: Failed to find " << Target_Device_Name << " device" << std::endl;
        std::cout << "Belows are all devices of the given device list" << std::endl;
        std::cout << "Please select the device by entering number, or enter 0 to exit" << std::endl;
        std::cout << "===============Device List===============" << std::endl;
        for(int j=0; j<devices.size(); j++){
            std::cout << "[" << j+1 << "] " << devices[j].getInfo<CL_DEVICE_NAME>() << std::endl;
        }
        int selected_device;
        std::cin >> selected_device;
        if(selected_device==0) exit(EXIT_FAILURE);        
        else{
            return devices[selected_device-1];
        }
    }
}





// Need to build Linux version of read_image_folder
std::vector<std::string> read_image_folder(const int FRAME_NUM, const std::string& folder_path){
    std::vector<std::string> images;

    for(int i=0; i<FRAME_NUM; i++){
        std::string index = std::to_string(i);
        index.insert(0, 6-index.size(), '0');
        std::string name = folder_path + index +".png";
        images.push_back(name);
    }

    return images;
}
void read_calibration(const std::string& file_path, cv::Mat* P0, cv::Mat* P1){
    FILE* fp;
	fopen_s(&fp, file_path.c_str(), "r");
	char* next_token1 = NULL;
	char* next_token2 = NULL;

	*P0 = cv::Mat(3, 4, CV_32F);
	*P1 = cv::Mat(3, 4, CV_32F);

	if (!fp)
	{
		printf("Could not open the calibration file\n");
	}

	int count = 0;
	bool p;
	char content[1024];
	while (fgets(content, 1024, fp))
	{
		char* v = strtok_s(content, " ,", &next_token1);
		while (v)
		{
			if (--count > 0) 
			{
				std::istringstream os(v);
				float d;
				os >> d;
				if(p)
					P1->at<float>((12 - count) / 4, (12 - count) % 4) = d;
				else
					P0->at<float>((12 - count) / 4, (12 - count) % 4) = d;
			}
			if (!strcmp(v, "P0:"))
			{
				count = 13;
				p = 0;
			}
			else if (!strcmp(v, "P1:"))
			{
				count = 13;
				p = 1;
			}
			 v = strtok_s(NULL, " ,", &next_token1);
		}
	}

	fclose(fp);
}


/*
bool is_emulation() {
    bool ret = false;
    char* xcl_mode = ("XCL_EMULATION_MODE");
    if (xcl_mode != NULL) {
        ret = true;
    }
    return ret;
}

bool is_hw_emulation() {
    bool ret = false;
    char* xcl_mode = _dupenv_s("XCL_EMULATION_MODE");
    if ((xcl_mode != NULL) && !strcmp(xcl_mode, "hw_emu")) {
        ret = true;
    }
    return ret;
}

bool is_xpr_device(const char* device_name) {
    const char* output = strstr(device_name, "xpr");

    if (output == NULL) {
        return false;
    } else {
        return true;
    }
}
*/
};
