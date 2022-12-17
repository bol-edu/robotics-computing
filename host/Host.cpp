#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <CL/cl.h>
#include "ErrHandler.cpp" 
#include "help_functions.h"
//#include "kernel.h"

using namespace std;

#define INFO

int main(int argc, const char** argv) {

//	Check Command Line Arguments
	#ifdef INFO
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info: 				Check Command Line Arguments                  " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

	if (argc != 4)
		return ErrHandler(err0, argv[0], EXIT_FAILURE);

	const char *Target_Platform_Vendor = argv[1];
	const char *Target_Device_Name = argv[2];
	const char *xclbinFilename = argv[3];
	cout << "HOST-Info: Platform_Vendor   : " << Target_Platform_Vendor << endl;
	cout << "HOST-Info: Device_Name       : " << Target_Device_Name << endl;
	cout << "HOST-Info: XCLBIN_file       : " << xclbinFilename << endl;
	cout << endl;

// 	   Detect Target Platform and Target Device in a system.
//         Create Context and Command Queue.
//         Get All PLATFORMS, then search for (CL_PLATFORM_VENDOR)
	
	// Get the number of platforms
	#ifdef INFO
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info:     Detect Target Platform and Target Device in a system 	  " << endl;
		cout << "HOST-Info:          	Create Context and Command Queue                  " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

	cl_uint ui;                        

	cl_platform_id *Platform_IDs;          
	cl_uint Num_Platforms;
	cl_platform_id Target_Platform_ID;
	bool Platform_Detected;
	char *platform_info;

	cl_device_id *Device_IDs;
	cl_uint Num_Devices;
	cl_device_id Target_Device_ID;
	bool Device_Detected;
	char *device_info;

	cl_context Context;
	cl_command_queue Command_Queue;

	cl_int errCode;
	size_t size;

	// Get the number of platforms
	errCode = clGetPlatformIDs(0, NULL, &Num_Platforms);
	if(errCode != CL_SUCCESS || Num_Platforms <= 0) return ErrHandler(err1, nullptr, EXIT_FAILURE);

	#ifdef INFO
		cout << "HOST-Info: Number of detected platforms : " << Num_Platforms << endl;
	#endif

 	// Allocate memory to store platforms
	Platform_IDs = new cl_platform_id[Num_Platforms];
	if(!Platform_IDs) return ErrHandler(err2, nullptr, EXIT_FAILURE);

	#ifdef INFO
		cout << "HOST-Info: Number of detected platforms : " << Num_Platforms << endl;
	#endif

	// Get and store all PLATFORMS
	errCode = clGetPlatformIDs(Num_Platforms, Platform_IDs, NULL);
	if(errCode != CL_SUCCESS) return ErrHandler(err3, nullptr, EXIT_FAILURE);

	// Search for Platform using: CL_PLATFORM_VENDOR = Target_Platform_Vendor
	Platform_Detected = false;
	for (ui = 0; ui < Num_Platforms; ui++){

		errCode = clGetPlatformInfo(Platform_IDs[ui], CL_PLATFORM_VENDOR, 0, NULL, &size);
		if (errCode != CL_SUCCESS) return ErrHandler(err4, nullptr, EXIT_FAILURE);

		platform_info = new char[size];
		if (!platform_info) return ErrHandler(err5, nullptr, EXIT_FAILURE);

		errCode = clGetPlatformInfo(Platform_IDs[ui], CL_PLATFORM_VENDOR, size, platform_info, NULL);
		if (errCode != CL_SUCCESS) return ErrHandler(err6, nullptr, EXIT_FAILURE);

		// Check if the current platform matches Target_Platform_Vendor
		if (strcmp(platform_info, Target_Platform_Vendor) == 0){
			Platform_Detected = true;
			Target_Platform_ID = Platform_IDs[ui];
			#ifdef INFO
						cout << "HOST-Info: Selected platform: " << Target_Platform_Vendor << endl
							<< endl;
			#endif
		}
	}
	if (Platform_Detected == false) return ErrHandler(err7, Target_Platform_Vendor, EXIT_FAILURE);

//	      Get All Devices for selected platform Target_Platform_ID
//            then search for Xilinx platform (CL_DEVICE_NAME = Target_Device_Name)

	// Get the Number of Devices
	errCode = clGetDeviceIDs(Target_Platform_ID, CL_DEVICE_TYPE_ALL, 0, NULL, &Num_Devices);
	if (errCode != CL_SUCCESS)  return ErrHandler(err8, nullptr, EXIT_FAILURE);
	
	#ifdef INFO
		cout << "HOST-Info: Number of available devices  : " << Num_Devices << endl;
	#endif

	// Allocate memory and store devices
	Device_IDs = new cl_device_id[Num_Devices];
	if (!Device_IDs) return ErrHandler(err9, nullptr, EXIT_FAILURE);

	errCode = clGetDeviceIDs(Target_Platform_ID, CL_DEVICE_TYPE_ALL, Num_Devices, Device_IDs, NULL);
	if (errCode != CL_SUCCESS) return ErrHandler(err10, nullptr, EXIT_FAILURE);
	
	// Search for CL_DEVICE_NAME = Target_Device_Name
	Device_Detected = false;
	for (ui = 0; ui < Num_Devices; ui++){
		errCode = clGetDeviceInfo(Device_IDs[ui], CL_DEVICE_NAME, 0, NULL, &size);
		if (errCode != CL_SUCCESS) return ErrHandler(err11, nullptr, EXIT_FAILURE);

		device_info = new char[size];
		if (!device_info) return ErrHandler(err12, nullptr, EXIT_FAILURE);
	
		errCode = clGetDeviceInfo(Device_IDs[ui], CL_DEVICE_NAME, size, device_info, NULL);
		if (errCode != CL_SUCCESS) return ErrHandler(err13, nullptr, EXIT_FAILURE);

		// Check if the current device matches Target_Device_Name
		if (strcmp(device_info, Target_Device_Name) == 0){
			Device_Detected = true;
			Target_Device_ID = Device_IDs[ui];
		}
	}

	if (Device_Detected == false) return ErrHandler(err14, Target_Device_Name, EXIT_FAILURE);
	else{
		#ifdef INFO
			cout << "HOST-Info: Selected device: " << Target_Device_Name << endl
		 		 << endl;
		#endif
	}

// Create Context

	#ifdef INFO
		cout << "HOST-Info: Creating Context ... " << endl;
	#endif
	Context = clCreateContext(0, 1, &Target_Device_ID, NULL, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err15, nullptr, EXIT_FAILURE);

// Create Command Queue (commands are executed in-order)
	#ifdef INFO
		cout << "HOST-Info: Creating Command Queue ... " << endl;
	#endif
	Command_Queue = clCreateCommandQueue(Context, Target_Device_ID, CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE | CL_QUEUE_PROFILING_ENABLE, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err16, nullptr, EXIT_FAILURE);


// Create Program and Kernel
	#ifdef INFO
		cout << endl;
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info:                  Create Program and Kernels                   " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

// Load Binary File from a disk to Memory
	unsigned char *xclbin_Memory;
	int program_length;

	#ifdef INFO
		cout << "HOST-Info: Loading " << xclbinFilename << " binary file to memory ..." << endl;
	#endif

	program_length = loadFile2Memory(xclbinFilename, (char **)&xclbin_Memory);
	if (program_length < 0) return ErrHandler(err17, xclbinFilename, EXIT_FAILURE);

// Create a program using a Binary File
	size_t Program_Length_in_Bytes;
	cl_program Program;
	cl_int Binary_Status;

	#ifdef INFO
		cout << "HOST-Info: Creating Program with Binary ..." << endl;
	#endif
	Program_Length_in_Bytes = program_length;
	Program = clCreateProgramWithBinary(Context, 1, &Target_Device_ID, &Program_Length_in_Bytes,
										(const unsigned char **)&xclbin_Memory, &Binary_Status, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err18, nullptr, EXIT_FAILURE);

// Build (compiles and links) a program executable from binary
	#ifdef INFO
		cout << "HOST-Info: Building the Program ..." << endl;
	#endif

	errCode = clBuildProgram(Program, 1, &Target_Device_ID, NULL, NULL, NULL);
	if (errCode != CL_SUCCESS) return ErrHandler(err19, nullptr, EXIT_FAILURE);
	
// Create a Kernels
	cl_kernel K_StereoMatching, K_FeatureExtract, K_FeatureTracker, K_MotionEstimation;

	#ifdef INFO
		cout << "HOST-Info: Creating a Kernel: K_StereoMatching ..." << endl;
	#endif
	K_StereoMatching = clCreateKernel(Program, "K_StereoMatching", &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err20, nullptr, EXIT_FAILURE);

	#ifdef INFO
		cout << "HOST-Info: Creating a Kernel: K_FeatureExtract ..." << endl;
	#endif
	K_FeatureExtract = clCreateKernel(Program, "K_FeatureExtract", &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err21, nullptr, EXIT_FAILURE);

	#ifdef INFO
		cout << "HOST-Info: Creating a Kernel: K_FeatureTracker ..." << endl;
	#endif
	K_FeatureTracker = clCreateKernel(Program, "K_FeatureTracker", &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err22, nullptr, EXIT_FAILURE);

	#ifdef INFO
		cout << "HOST-Info: Creating a Kernel: K_MotionEstimation ..." << endl;
	#endif
	K_MotionEstimation = clCreateKernel(Program, "K_MotionEstimation", &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err23, nullptr, EXIT_FAILURE);

// Prepare Data to Run Kernel
	int *ImgLeft_0, *ImgRight_0, *ImgLeft_1, *T_Mat;

	#ifdef INFO
		cout << endl;
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info:                 Prepare Data to Run Kernels                   " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

//	     Generate data for ImgLeft_0 array
//           Generate data for ImgRight_0 array
//           Generate data for ImgLeft_1 array
//           Allocate Memory to store the results: T_Mat array
	cl_uint CONST_arg = 5;

	void *ptr = nullptr;
	int Values_Period = 3;

	cout << "HOST-Info: Generating data for SIZE_ImgLeft_0 ... ";
	if (posix_memalign(&ptr, 4096, SIZE_ImgLeft_0 * sizeof(int)))
	{
		cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for SIZE_ImgLeft_0 array" << endl
			 << endl;
		return EXIT_FAILURE;
	}
	ImgLeft_0 = reinterpret_cast<int *>(ptr);
	gen_int_values(ImgLeft_0, SIZE_ImgLeft_0, Values_Period);
	cout << "Generated " << SIZE_ImgLeft_0 << " values" << endl;

	cout << "HOST-Info: Generating data for ImgRight_0 ... ";
	if (posix_memalign(&ptr, 4096, SIZE_ImgRight_0 * sizeof(int)))
	{
		cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for ImgRight_0 array" << endl
			 << endl;
		return EXIT_FAILURE;
	}
	ImgRight_0 = reinterpret_cast<int *>(ptr);
	gen_int_values(ImgRight_0, SIZE_ImgRight_0, Values_Period);
	cout << "Generated " << SIZE_ImgRight_0 << " values" << endl;

	cout << "HOST-Info: Generating data for DataIn_3 ... ";
	ImgLeft_1 = new int[SIZE_ImgLeft_1];
	gen_int_values(ImgLeft_1, SIZE_ImgLeft_1, Values_Period);
	cout << "Generated " << SIZE_ImgLeft_1 << " values" << endl;

	cout << "Allocated" << endl;

// Create Buffers in Global Memory to store data

}
