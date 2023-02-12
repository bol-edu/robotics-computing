#include <unistd.h>
#include <limits.h>
#include <sys/stat.h>
#include <string>
#include <dirent.h>
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
        std::cout << "Belows are all platforms on this system" << std::endl
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

std::string find_binary_file(const std::string& _device_name, const std::string& xclbin_name) {
    std::cout << "XCLBIN File Name: " << xclbin_name.c_str() << std::endl;
    char* xcl_mode = getenv("XCL_EMULATION_MODE");
    char* xcl_target = getenv("XCL_TARGET");
    std::string mode;

    /* Fall back mode if XCL_EMULATION_MODE is not set is "hw" */
    if (xcl_mode == NULL) {
        mode = "hw";
    } else {
        /* if xcl_mode is set then check if it's equal to true*/
        if (strcmp(xcl_mode, "true") == 0) {
            /* if it's true, then check if xcl_target is set */
            if (xcl_target == NULL) {
                /* default if emulation but not specified is software emulation */
                mode = "sw_emu";
            } else {
                /* otherwise, it's what ever is specified in XCL_TARGET */
                mode = xcl_target;
            }
        } else {
            /* if it's not equal to true then it should be whatever
             * XCL_EMULATION_MODE is set to */
            mode = xcl_mode;
        }
    }
    char* xcl_bindir = getenv("XCL_BINDIR");

    // typical locations of directory containing xclbin files
    const char* dirs[] = {xcl_bindir, // $XCL_BINDIR-specified
                          "xclbin",   // command line build
                          "..",       // gui build + run
                          ".",        // gui build, run in build directory
                          NULL};
    const char** search_dirs = dirs;
    if (xcl_bindir == NULL) {
        search_dirs++;
    }

    char* device_name = strdup(_device_name.c_str());
    if (device_name == NULL) {
        fprintf(stderr, "Error: Out of Memory\n");
        exit(EXIT_FAILURE);
    }

    // fix up device name to avoid colons and dots.
    // xilinx:xil-accel-rd-ku115:4ddr-xpr:3.2 -> xilinx_xil-accel-rd-ku115_4ddr-xpr_3_2
    for (char* c = device_name; *c != 0; c++) {
        if (*c == ':' || *c == '.') {
            *c = '_';
        }
    }

    char* device_name_versionless = strdup(_device_name.c_str());
    if (device_name_versionless == NULL) {
        fprintf(stderr, "Error: Out of Memory\n");
        exit(EXIT_FAILURE);
    }

    unsigned short colons = 0;
    bool colon_exist = false;
    for (char* c = device_name_versionless; *c != 0; c++) {
        if (*c == ':') {
            colons++;
            *c = '_';
            colon_exist = true;
        }
        /* Zero out version area */
        if (colons == 3) {
            *c = '\0';
        }
    }

    // versionless support if colon doesn't exist in device_name
    if (!colon_exist) {
        int len = strlen(device_name_versionless);
        device_name_versionless[len - 4] = '\0';
    }

    const char* aws_file_patterns[] = {
        "%1$s/%2$s.%3$s.%4$s.awsxclbin",       // <kernel>.<target>.<device>.awsxclbin
        "%1$s/%2$s.%3$s.%4$.0s%5$s.awsxclbin", // <kernel>.<target>.<device_versionless>.awsxclbin
        "%1$s/binary_container_1.awsxclbin",   // default for gui projects
        "%1$s/%2$s.awsxclbin",                 // <kernel>.awsxclbin
        NULL};

    const char* file_patterns[] = {"%1$s/%2$s.%3$s.%4$s.xclbin",       // <kernel>.<target>.<device>.xclbin
                                   "%1$s/%2$s.%3$s.%4$.0s%5$s.xclbin", // <kernel>.<target>.<device_versionless>.xclbin
                                   "%1$s/binary_container_1.xclbin",   // default for gui projects
                                   "%1$s/%2$s.xclbin",                 // <kernel>.xclbin
                                   NULL};
    char xclbin_file_name[PATH_MAX];
    memset(xclbin_file_name, 0, PATH_MAX);
    ino_t aws_ino = 0; // used to avoid errors if an xclbin found via multiple/repeated paths
    for (const char** dir = search_dirs; *dir != NULL; dir++) {
        struct stat sb;
        if (stat(*dir, &sb) == 0 && S_ISDIR(sb.st_mode)) {
            for (const char** pattern = aws_file_patterns; *pattern != NULL; pattern++) {
                char file_name[PATH_MAX];
                memset(file_name, 0, PATH_MAX);
                snprintf(file_name, PATH_MAX, *pattern, *dir, xclbin_name.c_str(), mode.c_str(), device_name,
                         device_name_versionless);
                if (stat(file_name, &sb) == 0 && S_ISREG(sb.st_mode)) {
                    char* bindir = strdup(*dir);
                    if (bindir == NULL) {
                        fprintf(stderr, "Error: Out of Memory\n");
                        exit(EXIT_FAILURE);
                    }
                    if (*xclbin_file_name && sb.st_ino != aws_ino) {
                        fprintf(stderr, "Error: multiple xclbin files discovered:\n %s\n %s\n", file_name,
                                xclbin_file_name);
                        exit(EXIT_FAILURE);
                    }
                    aws_ino = sb.st_ino;
                    strncpy(xclbin_file_name, file_name, PATH_MAX);
                }
            }
        }
    }
    ino_t ino = 0; // used to avoid errors if an xclbin found via multiple/repeated paths
    // if no awsxclbin found, check for xclbin
    if (*xclbin_file_name == '\0') {
        for (const char** dir = search_dirs; *dir != NULL; dir++) {
            struct stat sb;
            if (stat(*dir, &sb) == 0 && S_ISDIR(sb.st_mode)) {
                for (const char** pattern = file_patterns; *pattern != NULL; pattern++) {
                    char file_name[PATH_MAX];
                    memset(file_name, 0, PATH_MAX);
                    snprintf(file_name, PATH_MAX, *pattern, *dir, xclbin_name.c_str(), mode.c_str(), device_name,
                             device_name_versionless);
                    if (stat(file_name, &sb) == 0 && S_ISREG(sb.st_mode)) {
                        char* bindir = strdup(*dir);
                        if (bindir == NULL) {
                            fprintf(stderr, "Error: Out of Memory\n");
                            exit(EXIT_FAILURE);
                        }
                        if (*xclbin_file_name && sb.st_ino != ino) {
                            fprintf(stderr, "Error: multiple xclbin files discovered:\n %s\n %s\n", file_name,
                                    xclbin_file_name);
                            exit(EXIT_FAILURE);
                        }
                        ino = sb.st_ino;
                        strncpy(xclbin_file_name, file_name, PATH_MAX);
                    }
                }
            }
        }
    }
    // if no xclbin found, preferred path for error message from xcl_import_binary_file()
    if (*xclbin_file_name == '\0') {
        snprintf(xclbin_file_name, PATH_MAX, file_patterns[0], *search_dirs, xclbin_name.c_str(), mode.c_str(),
                 device_name);
    }
    free(device_name);
    return (xclbin_file_name);
}

cl::Program::Binaries import_binary_file(std::string xclbin_file_name) {
    std::cout << "INFO: Importing " << xclbin_file_name << std::endl;

    if (access(xclbin_file_name.c_str(), R_OK) != 0) {
        fprintf(stderr, "ERROR: %s xclbin not available please build\n", xclbin_file_name.c_str());
        exit(EXIT_FAILURE);
    }
    // Loading XCL Bin into char buffer
    std::cout << "Loading: '" << xclbin_file_name.c_str() << "'\n";
    std::ifstream bin_file(xclbin_file_name.c_str(), std::ifstream::binary);
    bin_file.seekg(0, bin_file.end);
    unsigned nb = bin_file.tellg();
    bin_file.seekg(0, bin_file.beg);
    char* buf = new char[nb];
    bin_file.read(buf, nb);
    bin_file.close();

    cl::Program::Binaries bins;
    bins.push_back({buf, nb});
    return bins;
}

char* read_binary_file(const std::string& xclbin_file_name, unsigned& nb){
    std::cout << "INFO: Reading " << xclbin_file_name << std::endl;

    if (access(xclbin_file_name.c_str(), R_OK) != 0) {
        fprintf(stderr, "ERROR: %s xclbin not available please build\n", xclbin_file_name.c_str());
        exit(EXIT_FAILURE);
    }
    // Loading XCL Bin into char buffer
    std::cout << "Loading: '" << xclbin_file_name.c_str() << "'\n";
    std::ifstream bin_file(xclbin_file_name.c_str(), std::ifstream::binary);
    bin_file.seekg(0, bin_file.end);
    nb = bin_file.tellg();
    bin_file.seekg(0, bin_file.beg);
    char* buf = new char[nb];
    bin_file.read(buf, nb);
    return buf;
}


// Need to build Linux version of read_image_folder
std::vector<string> read_image_folder(const int FRAME_NUM, const std::string& folder_path){
    std::vector<std::string> images;

    for(int i=0; i<4541; i++){
        std::string index = stoi(i);
        index.insert(0, 6-index.size(), '0');
        std::string name = folder_path + "/" + index +".png";
        images.push_back(name);
    }

    return images;
}
void read_calibration(const std::string& file_path, cv::Mat* P0, cv::Mat* P1){
    FILE* fp;
	fopen_s(&fp, filePath, "r");
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
				istringstream os(v);
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



void decompose_Projection_Matrix(Mat p, Mat* k, Mat* r, Mat* t){
	Mat rotMatrixX;
	Mat rotMatrixY;
	Mat rotMatrixZ;
	Mat eulerAngles;
	
	*t = *t / (t->at<float>(3));
}


bool is_emulation() {
    bool ret = false;
    char* xcl_mode = getenv("XCL_EMULATION_MODE");
    if (xcl_mode != NULL) {
        ret = true;
    }
    return ret;
}

bool is_hw_emulation() {
    bool ret = false;
    char* xcl_mode = getenv("XCL_EMULATION_MODE");
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
};
