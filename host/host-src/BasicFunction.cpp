#include <unistd.h>
#include <limits.h>
#include <sys/stat.h>
#include <sstream>
#include <cstring>
#include <string>
#include <vector>
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
            std::cout << "[HOST-Info] Found Platform Name: '" << platformName << "'" << std::endl;
            // Get ALL devices of the specific platform
            platforms[i].getDevices(CL_DEVICE_TYPE_ALL, &devices);
            return devices;
        }
    }

    // No corresponding platform, select manually or exit
    if (i==platforms.size()) {
        std::cout << "[HOST-Error] Failed to find '" << Target_Platform_Vendor << "' platform" << std::endl << std::endl;
        std::cout << "[HOST-Info] Belows are all platforms on this system" << std::endl;
        std::cout << "[HOST-Info] Please select the platform by entering number, or enter 0 to exit" << std::endl;
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
            std::cout << "[HOST-Info] Found Device Name: " << deviceName << std::endl;
            // Get device of the given devices
            return devices[i];
        }
    }
    // No corresponding device, select manually or exit
    if(i==devices.size()){
        std::cout << "[HOST-Error] Failed to find '" << Target_Device_Name << "' device" << std::endl << std::endl;
        std::cout << "[HOST-Info] Belows are all devices of the given device list" << std::endl;
        std::cout << "[HOST-Info] Please select the device by entering number, or enter 0 to exit" << std::endl;
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
void read_calibration(const std::string& File_Path, cv::Mat* P0, cv::Mat* P1){
    FILE* fp;
	fp = fopen(File_Path.c_str(), "r");
	char* next_token1 = NULL;
	char* next_token2 = NULL;

	*P0 = cv::Mat(3, 4, CV_32F);
	*P1 = cv::Mat(3, 4, CV_32F);

	if (!fp)
	{
		printf("[HOST-Error] Could not open the calibration file\n");
	}

	int count = 0;
	bool p;
	char content[1024];
	while (fgets(content, 1024, fp))
	{
		char* v = strtok_r(content, " ,", &next_token1);
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
			 v = strtok_r(NULL, " ,", &next_token1);
		}
	}

	fclose(fp);
}

void myimread(const std::string& File_Path, uchar* Image_Pointer, cv::Mat* Std_Image){

    FILE *fimg = fopen(File_Path.c_str(), "rb");
    if(!fimg){
        std::cout << "[HOST-Error] Could not open the path: " << "'" << File_Path << "'\n";
		exit(EXIT_FAILURE);
    }
    // Initialize the PNG structure
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr) {
        std::cout << "[HOST-Error] libpng failed to create PNG structure\n";
        fclose(fimg);
		exit(EXIT_FAILURE);
    }
    // Initialize the PNG info structure
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        std::cout << "[HOST-Error] libpng failed to create PNG info structure\n";
        fclose(fimg);
		exit(EXIT_FAILURE);
    }

    // Set up error handling
    if (setjmp(png_jmpbuf(png_ptr))) {
        std::cout << "[HOST-Error] libpng failed to set setjmp\n";
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fimg);
		exit(EXIT_FAILURE);
    }
    // Set up the input code
    png_init_io(png_ptr, fimg);
    png_read_info(png_ptr, info_ptr);

    int width = png_get_image_width(png_ptr, info_ptr);
    int height = png_get_image_height(png_ptr, info_ptr);
    int row_byte = png_get_rowbytes(png_ptr, info_ptr); 
    png_byte color_type = png_get_color_type(png_ptr, info_ptr);
    png_byte bit_depth = png_get_bit_depth(png_ptr, info_ptr);

    // Make sure the image is grayscale with 8-bit depth
    if (color_type != PNG_COLOR_TYPE_GRAY || bit_depth != 8) {
        std::cout << "[HOST-Error] libpng load a not alike image\n";
        std::cout << "color_type: " << color_type << "\n";
        std::cout << "bit_depth: " << bit_depth << "\n";

        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fimg);
		exit(EXIT_FAILURE);
    }

    png_bytep *row_pointers = new png_bytep[height];
    for(int i=0; i<height; i++) row_pointers[i] = Image_Pointer + i*row_byte;


    png_read_image(png_ptr, row_pointers);

    free(row_pointers);
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    fclose(fimg);
}

void print_content(const std::string& File_Path, cv::Mat* Matrix) {
    std::ofstream fout;
    fout.open(File_Path.c_str(), std::ios::out);
    if(!fout){
        std::cout << "[HOST-Error] Could not open the path: " << "'" << File_Path << "'\n";
		exit(EXIT_FAILURE);
	}
    fout << *Matrix;
    fout.close();
}

cl::Program::Binaries import_binary_file(std::string xclbin_file_name) {
    std::cout << "[HOST-Info] Importing: '" << xclbin_file_name << "'" << std::endl;

    if (access(xclbin_file_name.c_str(), R_OK) != 0) {
        fprintf(stderr, "[HOST-Error] %s xclbin not available please build\n", xclbin_file_name.c_str());
        exit(EXIT_FAILURE);
    }
    // Loading XCL Bin into char buffer
    std::cout << "[HOST-Info] Loading: '" << xclbin_file_name.c_str() << "'\n";
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
std::string find_binary_file(const std::string& _device_name, const std::string& xclbin_name) {
    std::cout << "[HOST-Info] XCLBIN File Name: '" << xclbin_name.c_str() << "'" << std::endl;
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
        fprintf(stderr, "[HOST-Error] Out of Memory\n");
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
        fprintf(stderr, "[HOST-Error] Out of Memory\n");
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
                        fprintf(stderr, "[HOST-Error] Out of Memory\n");
                        exit(EXIT_FAILURE);
                    }
                    if (*xclbin_file_name && sb.st_ino != aws_ino) {
                        fprintf(stderr, "[HOST-Error] multiple xclbin files discovered:\n %s\n %s\n", file_name,
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
                            fprintf(stderr, "[HOST-Error] Out of Memory\n");
                            exit(EXIT_FAILURE);
                        }
                        if (*xclbin_file_name && sb.st_ino != ino) {
                            fprintf(stderr, "[HOST-Error] multiple xclbin files discovered:\n %s\n %s\n", file_name,
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


void matwrite(const std::string& filename, cv::Mat* mat)
{
    std::ofstream fs(filename, std::fstream::binary);

    // Header
    int type = mat->type();
    int channels = mat->channels();
    fs.write((char*)&(mat->rows), sizeof(int));    // rows
    fs.write((char*)&(mat->cols), sizeof(int));    // cols
    fs.write((char*)&type, sizeof(int));        // type
    fs.write((char*)&channels, sizeof(int));    // channels

    // Data
    if (mat->isContinuous())
    {
        fs.write(mat->ptr<char>(0), (mat->dataend - mat->datastart));
    }
    else
    {
        int rowsz = CV_ELEM_SIZE(type) * mat->cols;
        for (int r = 0; r < mat->rows; ++r)
        {
            fs.write(mat->ptr<char>(r), rowsz);
        }
    }
    
}

void matread(const std::string& filename, cv::Mat* mat)
{
    std::ifstream fs(filename, std::fstream::binary);

    // Get length of file
    fs.seekg(0, fs.end);
    int length = fs.tellg();
    fs.seekg(0, fs.beg);

    //while (fs.tellg() < length)
    //{
        // Header
        int rows, cols, type, channels;
        fs.read((char*)&rows, sizeof(int));         // rows
        fs.read((char*)&cols, sizeof(int));         // cols
        fs.read((char*)&type, sizeof(int));         // type
        fs.read((char*)&channels, sizeof(int));     // channels

        // Data
        //cv::Mat mat(rows, cols, type);
        memset((char*)(mat->data), 0, CV_ELEM_SIZE(type) * rows * cols);
        fs.read((char*)(mat->data), CV_ELEM_SIZE(type) * rows * cols);
    // }
    // return mat;
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
