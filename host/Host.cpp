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
#include "Host.h"
#include "ErrHandler.cpp" 
#include "help_functions.h"
//#include "kernel.h"

using namespace std;

#define INFO

int main(int argc, const char** argv) {

//	Step 1: Check Command Line Arguments
	#ifdef INFO
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info: 				Check Command Line Arguments                  " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

	if (argc != 4) return ErrHandler(err0, argv[0], EXIT_FAILURE);

	const char *Target_Platform_Vendor = argv[1];
	const char *Target_Device_Name = argv[2];
	const char *xclbinFilename = argv[3];
	cout << "HOST-Info: Platform_Vendor   : " << Target_Platform_Vendor << endl;
	cout << "HOST-Info: Device_Name       : " << Target_Device_Name << endl;
	cout << "HOST-Info: XCLBIN_file       : " << xclbinFilename << endl;
	cout << endl;

// Step 2: Detect Target Platform and Target Device in a system.
//         Create Context and Command Queue.
// Step 2.1: Get All PLATFORMS, then search for (CL_PLATFORM_VENDOR)
	
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

//			  Get All Devices for selected platform Target_Platform_ID
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
	cl_kernel K_StereoMatching, K_FeatureExtraction, K_FeatureTracking, K_MotionEstimation;

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
	int *ImgLeft_0, *ImgRight_0, *ImgLeft_1, *Mask, *K_Left, *T_Left, *T_Right, *Filter, *T_Mat;

	#ifdef INFO
		cout << endl;
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info:                 Prepare Data to Run Kernels                   " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

//			 Generate data for ImgLeft_0, ImgRight_0, ImgLeft_1, Mask, K_Left, T_Left, K_Left, Filter array
//           Allocate Memory to store the results: T_Mat array
	cl_uint ConstArg_MaxDepth = MaxDepth;
	cl_uint ConstArg_K = K;
	void *ptr = nullptr;

	// SIZE_XXXX are in byte
	cout << "HOST-Info: Preparing data for ImgLeft_0 ... ";
	if (posix_memalign(&ptr, 4096, SIZE_ImgLeft_0)) return ErrHandler(err24, nullptr, EXIT_FAILURE);
	ImgLeft_0 = reinterpret_cast<Img_Type *>(ptr);
	cout << "Generated " << SIZE_ImgLeft_0 << " bytes value" << endl;

	cout << "HOST-Info: Preparing data for ImgRight_0 ... ";
	if (posix_memalign(&ptr, 4096, SIZE_ImgRight_0)) return ErrHandler(err25, nullptr, EXIT_FAILURE);
	ImgRight_0 = reinterpret_cast<Img_Type *>(ptr);
	cout << "Generated " << SIZE_ImgRight_0 << " values" << endl;

	cout << "HOST-Info: Preparing data for ImgLeft_1 ... ";
	if (posix_memalign(&ptr, 4096, SIZE_ImgLeft_1)) return ErrHandler(err26, nullptr, EXIT_FAILURE);
	ImgLeft_1 = reinterpret_cast<Img_Type *>(ptr);
	cout << "Generated " << SIZE_ImgLeft_1 << " values" << endl;

	cout << "HOST-Info: Preparing data for Mask ... ";
	if (posix_memalign(&ptr, 4096, SIZE_Mask)) return ErrHandler(err27, nullptr, EXIT_FAILURE);
	Mask = reinterpret_cast<Mask_Type *>(ptr);
	cout << "Generated " << SIZE_Mask << " values" << endl;

	cout << "HOST-Info: Preparing data for K_Left ... ";
	if (posix_memalign(&ptr, 4096, SIZE_K_Left)) return ErrHandler(err28, nullptr, EXIT_FAILURE);
	K_Left = reinterpret_cast<Mat_P_Type *>(ptr);
	cout << "Generated " << SIZE_K_Left << " values" << endl;

	cout << "HOST-Info: Preparing data for T_Left ... ";
	if (posix_memalign(&ptr, 4096, SIZE_T_Left)) return ErrHandler(err29, nullptr, EXIT_FAILURE);
	T_Left = reinterpret_cast<Mat_P_Type *>(ptr);
	cout << "Generated " << SIZE_T_Left << " values" << endl;

	cout << "HOST-Info: Preparing data for T_Right ... ";
	if (posix_memalign(&ptr, 4096, SIZE_T_Right)) return ErrHandler(err30, nullptr, EXIT_FAILURE);
	T_Right = reinterpret_cast<Mat_P_Type *>(ptr);
	cout << "Generated " << SIZE_T_Right << " values" << endl;

	cout << "HOST-Info: Preparing data for Filter ... ";
	if (posix_memalign(&ptr, 4096, SIZE_Filter)) return ErrHandler(err31, nullptr, EXIT_FAILURE);
	Filter = reinterpret_cast<Filter_Type *>(ptr);
	cout << "Generated " << SIZE_Filter << " values" << endl;

	cout << "HOST-Info: Allocating memory for T_Mat ... ";
	if (posix_memalign(&ptr, 4096, SIZE_T_Mat)) return ErrHandler(err32, nullptr, EXIT_FAILURE);
	T_Mat = reinterpret_cast<T_Mat_Type *>(ptr);
	cout << "Generated " << SIZE_T_Mat << " values" << endl;


	cout << "Allocated" << endl;

// Create Buffers in Global Memory to store data
// 			GlobMem_ImgLeft_0				
// 			GlobMem_ImgRight_0
// 			GlobMem_ImgLeft_1
// 			GlobMem_Mask
// 			GlobMem_K_Left
// 			GlobMem_T_Left
// 			GlobMem_T_Right
// 			GlobMem_Filter
// 			GlobMem_T_Mat
// 			GlobMem_BUF_Depth			
//			GlobMem_BUF_KP0
//			GlobMem_BUF_KP1
//			GlobMem_BUF_Des0
//			GlobMem_BUF_Des1
//			GlobMem_BUF_Detected_Points
//			GlobMem_BUF_Matches
//			GlobMem_BUF_Detected_Matches

#ifdef INFO
	cout << "HOST-Info: Allocating buffers in Global Memory to store Input and Output Data ..." << endl;
#endif
	cl_mem GlobMem_ImgLeft_0, GlobMem_ImgRight_0, GlobMem_ImgLeft_1, GlobMem_Mask,
			GlobMem_K_Left, GlobMem_T_Left, GlobMem_T_Right, GlobMem_Filter, GlobMem_T_Mat,
			GlobMem_BUF_Depth, GlobMem_BUF_KP0, GlobMem_BUF_KP1, GlobMem_BUF_Des0, GlobMem_BUF_Des1,
			GlobMem_BUF_Detected_Points, GlobMem_BUF_Matches, GlobMem_BUF_Detected_Matches, GlobMem_BUF_rmat, GlobMem_BUF_tvec;
	
	// reference for clCreateBuffer 
	// https://stackoverflow.com/questions/61578885/whats-the-purpose-of-host-ptr-parameter-in-clcreatebuffer
	// https://www.twblogs.net/a/5b89bb3d2b71775d1ce384bf

	// Allocate Global Memory for GlobMem_ImgLeft_0
	GlobMem_ImgLeft_0 = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_0, ImgLeft_0, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err33, nullptr, EXIT_FAILURE);

	// Allocate Global Memory for GlobMem_ImgRight_0
	GlobMem_ImgRight_0 = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgRight_0, ImgRight_0, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err34, nullptr, EXIT_FAILURE);

	// Allocate Global Memory for GlobMem_ImgLeft_1
	GlobMem_ImgLeft_1 = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_1, ImgLeft_1, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err35, nullptr, EXIT_FAILURE);

	// Allocate Global Memory for GlobMem_Mask
	GlobMem_Mask = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_Mask, Mask, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err36, nullptr, EXIT_FAILURE);

	// Allocate Global Memory for GlobMem_K_Left
	GlobMem_K_Left = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_K_Left, K_Left, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err37, nullptr, EXIT_FAILURE);

	// Allocate Global Memory for GlobMem_T_Left
	GlobMem_T_Left = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_T_Left, T_Left, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err38, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_T_Right
	GlobMem_T_Right = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_T_Right, T_Right, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err39, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_Filter
	GlobMem_Filter = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_Filter, Filter, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err40, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_T_Mat
	GlobMem_T_Mat = clCreateBuffer(Context, CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR, SIZE_T_Mat, T_Mat, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err41, nullptr, EXIT_FAILURE);
	

	
	// Allocate Global Memory for GlobMem_BUF_Depth
	GlobMem_BUF_Depth = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_Depth, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err42, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_BUF_KP0
	GlobMem_BUF_KP0 = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_KP0, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err43, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_BUF_KP1
	GlobMem_BUF_KP1 = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_KP1, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err44, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_BUF_Des0
	GlobMem_BUF_Des0 = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_Des0, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err45, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_BUF_Des1
	GlobMem_BUF_Des1 = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_Des1, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err46, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_BUF_Detected_Points
	GlobMem_BUF_Detected_Points = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_Detected_Points, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err47, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_BUF_Matches
	GlobMem_BUF_Matches = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_Matches, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err48, nullptr, EXIT_FAILURE);
	
	// Allocate Global Memory for GlobMem_BUF_Detected_Matches
	GlobMem_BUF_Detected_Matches = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_Detected_Matches, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err49, nullptr, EXIT_FAILURE);

	// Allocate Global Memory for GlobMem_BUF_rmat
	GlobMem_BUF_rmat = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_rmat, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err50, nullptr, EXIT_FAILURE);

	// Allocate Global Memory for GlobMem_BUF_tvec
	GlobMem_BUF_tvec = clCreateBuffer(Context, CL_MEM_READ_WRITE, SIZE_BUF_tvec, NULL, &errCode);
	if (errCode != CL_SUCCESS) return ErrHandler(err51, nullptr, EXIT_FAILURE);
	
	// =======================================================================================================================================
	// Set Kernel Arguments and Run the Application
	//				----------------------------------------------------------------
	// 				 		Kernel	  		Argument Nb			Description
	// 				----------------------------------------------------------------
	//  			 	K_StereoMatching		0			GlobMem_ImgLeft_0
	//				 	K_StereoMatching		1			GlobMem_ImgRight_0
	//				 	K_StereoMatching		2			GlobMem_K_Left
	//				 	K_StereoMatching		3			GlobMem_T_Left
	//				 	K_StereoMatching		4			GlobMem_T_Right
	//				 	K_StereoMatching		5			GlobMem_BUF_Depth
	//					I didn't add matcher(bool), rgb(bool), recticified(bool) to args, if you need other args please tell me.
	//					You also need to load K_Left, T_Left, T_Right directly instead of P0, P1,
	//					and write GlobMem_BUF_Depth by address, not returning value.
	//
	//  			 	K_FeatureExtraction		0			GlobMem_ImgLeft_0
	//  			 	K_FeatureExtraction		1			GlobMem_ImgLeft_1
	//					K_FeatureExtraction		2			GlobMem_Mask
	//					K_FeatureExtraction		3			GlobMem_BUF_KP0	
	//					K_FeatureExtraction		4			GlobMem_BUF_KP1
	//					K_FeatureExtraction		5			GlobMem_BUF_Des0
	//					K_FeatureExtraction		6			GlobMem_BUF_Des1
	//					K_FeatureExtraction		7			GlobMem_BUF_Detected_Points
	//					I didn't add detector(bool) to args, if you need other args please tell me.
	//					You need to load ImgLeft_0, ImgLeft_1, KP0, KP1 at once, 
	//					and write GlobMem_BUF_Des0, GlobMem_BUF_Des1, GlobMem_BUF_Detected_Points by address, not returning value.
	//
	//  			 	K_FeatureTracking		0			GlobMem_Filter
	//  			 	K_FeatureTracking		1			GlobMem_BUF_Des0
	//  			 	K_FeatureTracking		2			GlobMem_BUF_Des1
	//  			 	K_FeatureTracking		3			GlobMem_BUF_Detected_Points
	//  			 	K_FeatureTracking		4			GlobMem_BUF_Matches
	//  			 	K_FeatureTracking		5			GlobMem_BUF_Detected_Matches
	//					K_FeatureTracking		6			ConstArg_K
	//					I didn't add matching(bool), detector(bool), sorting(bool) to args, if you need other args please tell me.
	//					You need to write GlobMem_BUF_Matches, GlobMem_BUF_Detected_Matches by address, not returning value.
	//
	//  			 	K_MotionEstimation		0			GlobMem_K_Left
	//  			 	K_MotionEstimation		1			GlobMem_BUF_Depth
	//  			 	K_MotionEstimation		2			GlobMem_BUF_Matches
	//  			 	K_MotionEstimation		3			GlobMem_BUF_Detected_Matches
	//  			 	K_MotionEstimation		4			GlobMem_BUF_KP0
	//  			 	K_MotionEstimation		5			GlobMem_BUF_KP1
	//  			 	K_MotionEstimation		6			GlobMem_BUF_rmat
	//  			 	K_MotionEstimation		7			GlobMem_BUF_tvec
	//  			 	K_MotionEstimation		8			ConstArg_MaxDepth
	//					I didn't add img1_points, img2_points to args, if you need other args please tell me.
	//					You need to write GlobMem_BUF_rmat, GlobMem_BUF_tvec by address, not returning value.
	// 				----------------------------------------------------------------	
	// =======================================================================================================================================
	int Num_Mem_Events = 20, Num_Exe_Events = 4;
	cl_event Mem_op_event[Num_Mem_Events],K_exe_event[Num_Exe_Events];

#ifdef INFO
	cout << endl;
	cout << "HOST-Info: ============================================================= " << endl;
	cout << "HOST-Info:  					  Run Application                         " << endl;
	cout << "HOST-Info: ============================================================= " << endl;
#endif

// Set Kernel Arguments
#ifdef INFO
	cout << "HOST-Info: Setting Kernel arguments ..." << endl;
#endif

	errCode = false;

	errCode |= clSetKernelArg(K_StereoMatching, 0, sizeof(cl_mem), &GlobMem_ImgLeft_0);
	errCode |= clSetKernelArg(K_StereoMatching, 1, sizeof(cl_mem), &GlobMem_ImgRight_0);
	errCode |= clSetKernelArg(K_StereoMatching, 2, sizeof(cl_mem), &GlobMem_K_Left);
	errCode |= clSetKernelArg(K_StereoMatching, 3, sizeof(cl_mem), &GlobMem_T_Left);
	errCode |= clSetKernelArg(K_StereoMatching, 4, sizeof(cl_mem), &GlobMem_T_Right);
	errCode |= clSetKernelArg(K_StereoMatching, 5, sizeof(cl_mem), &GlobMem_BUF_Depth);

	errCode |= clSetKernelArg(K_FeatureExtraction, 0, sizeof(cl_mem), &GlobMem_ImgLeft_0);
	errCode |= clSetKernelArg(K_FeatureExtraction, 1, sizeof(cl_mem), &GlobMem_ImgLeft_1);
	errCode |= clSetKernelArg(K_FeatureExtraction, 2, sizeof(cl_mem), &GlobMem_Mask);
	errCode |= clSetKernelArg(K_FeatureExtraction, 3, sizeof(cl_mem), &GlobMem_BUF_KP0);
	errCode |= clSetKernelArg(K_FeatureExtraction, 4, sizeof(cl_mem), &GlobMem_BUF_KP1);
	errCode |= clSetKernelArg(K_FeatureExtraction, 5, sizeof(cl_mem), &GlobMem_BUF_Des0);
	errCode |= clSetKernelArg(K_FeatureExtraction, 6, sizeof(cl_mem), &GlobMem_BUF_Des1);
	errCode |= clSetKernelArg(K_FeatureExtraction, 7, sizeof(cl_mem), &GlobMem_BUF_Detected_Points);

	errCode |= clSetKernelArg(K_FeatureTracking, 0, sizeof(cl_mem), &GlobMem_Filter);
	errCode |= clSetKernelArg(K_FeatureTracking, 1, sizeof(cl_mem), &GlobMem_BUF_Des0);
	errCode |= clSetKernelArg(K_FeatureTracking, 2, sizeof(cl_mem), &GlobMem_BUF_Des1);
	errCode |= clSetKernelArg(K_FeatureTracking, 3, sizeof(cl_mem), &GlobMem_BUF_Detected_Points);
	errCode |= clSetKernelArg(K_FeatureTracking, 4, sizeof(cl_mem), &GlobMem_BUF_Matches);
	errCode |= clSetKernelArg(K_FeatureTracking, 5, sizeof(cl_mem), &GlobMem_BUF_Detected_Matches);
	errCode |= clSetKernelArg(K_FeatureTracking, 6, sizeof(cl_uint), &ConstArg_K);

	errCode |= clSetKernelArg(K_MotionEstimation, 0, sizeof(cl_mem), &GlobMem_K_Left);
	errCode |= clSetKernelArg(K_MotionEstimation, 1, sizeof(cl_mem), &GlobMem_BUF_Depth);
	errCode |= clSetKernelArg(K_MotionEstimation, 2, sizeof(cl_mem), &GlobMem_BUF_Matches);
	errCode |= clSetKernelArg(K_MotionEstimation, 3, sizeof(cl_mem), &GlobMem_BUF_Detected_Matches);
	errCode |= clSetKernelArg(K_MotionEstimation, 4, sizeof(cl_mem), &GlobMem_BUF_KP0);
	errCode |= clSetKernelArg(K_MotionEstimation, 5, sizeof(cl_mem), &GlobMem_BUF_KP1);
	errCode |= clSetKernelArg(K_MotionEstimation, 6, sizeof(cl_mem), &GlobMem_BUF_rmat);
	errCode |= clSetKernelArg(K_MotionEstimation, 7, sizeof(cl_mem), &GlobMem_BUF_tvec);
	errCode |= clSetKernelArg(K_MotionEstimation, 8, sizeof(cl_uint), &ConstArg_MaxDepth);

	if (errCode != CL_SUCCESS) return ErrHandler(err52, nullptr, EXIT_FAILURE)

#ifdef INFO
	cout << "HOST_Info: Copy Input data to Global Memory ..." << endl;
#endif

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_ImgLeft_0, 0, 0, NULL, &Mem_op_event[0]);
	if (errCode != CL_SUCCESS) return ErrHandler(err53, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_ImgRight_0, 0, 0, NULL, &Mem_op_event[1]);
	if (errCode != CL_SUCCESS) return ErrHandler(err54, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_ImgLeft_1, 0, 0, NULL, &Mem_op_event[2]);
	if (errCode != CL_SUCCESS) return ErrHandler(err55, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_Mask, 0, 0, NULL, &Mem_op_event[3]);
	if (errCode != CL_SUCCESS) return ErrHandler(err56, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_K_Left, 0, 0, NULL, &Mem_op_event[4]);
	if (errCode != CL_SUCCESS) return ErrHandler(err57, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_T_Left, 0, 0, NULL, &Mem_op_event[5]);
	if (errCode != CL_SUCCESS) return ErrHandler(err58, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_T_Right, 0, 0, NULL, &Mem_op_event[6]);
	if (errCode != CL_SUCCESS) return ErrHandler(err59, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_Filter, 0, 0, NULL, &Mem_op_event[7]);
	if (errCode != CL_SUCCESS) return ErrHandler(err60, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_T_Mat, 0, 0, NULL, &Mem_op_event[8]);
	if (errCode != CL_SUCCESS) return ErrHandler(err61, nullptr, EXIT_FAILURE);

	// --------------------------------------------------------

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_Depth, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[9]);
	if (errCode != CL_SUCCESS) return ErrHandler(err62, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_KP0, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[10]);
	if (errCode != CL_SUCCESS) return ErrHandler(err63, nullptr, EXIT_FAILURE);
	
	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_KP1, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[11]);
	if (errCode != CL_SUCCESS) return ErrHandler(err64, nullptr, EXIT_FAILURE);
	
	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_Des0, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[12]);
	if (errCode != CL_SUCCESS) return ErrHandler(err65, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_Des1, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[13]);
	if (errCode != CL_SUCCESS) return ErrHandler(err66, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_Detected_Points, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[14]);
	if (errCode != CL_SUCCESS) return ErrHandler(err67, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_Matches, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[15]);
	if (errCode != CL_SUCCESS) return ErrHandler(err68, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_Detected_Matches, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[16]);
	if (errCode != CL_SUCCESS) return ErrHandler(err69, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_rmat, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[17]);
	if (errCode != CL_SUCCESS) return ErrHandler(err70, nullptr, EXIT_FAILURE);

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_tvec, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, 0, NULL, &Mem_op_event[18]);
	if (errCode != CL_SUCCESS) return ErrHandler(err71, nullptr, EXIT_FAILURE);

	// --------------------------------------------------------

	errCode = clEnqueueBarrierWithWaitList(Command_Queue, 0, NULL, NULL);
	if (errCode != CL_SUCCESS) return ErrHandler(err72, nullptr, EXIT_FAILURE);


// Submit Kernels for Execution
#ifdef INFO
	cout << "HOST-Info: Submitting Kernel StereoMatching ..." << endl;
#endif
	errCode = clEnqueueTask(Command_Queue, K_StereoMatching, 0, NULL, &K_exe_event[0]);
	if (errCode != CL_SUCCESS) return ErrHandler(err73, nullptr, EXIT_FAILURE);

#ifdef INFO
	cout << "HOST-Info: Submitting Kernel FeatureExtraction ..." << endl;
#endif
	errCode = clEnqueueTask(Command_Queue, K_FeatureExtraction, 1, &K_exe_event[0], &K_exe_event[1]);
	if (errCode != CL_SUCCESS) return ErrHandler(err74, nullptr, EXIT_FAILURE);

#ifdef INFO
	cout << "HOST-Info: Submitting Kernel FeatureTracking ..." << endl;
#endif
	errCode = clEnqueueTask(Command_Queue, K_FeatureTracking, 1, &K_exe_event[1], &K_exe_event[2]);
	if (errCode != CL_SUCCESS) return ErrHandler(err75, nullptr, EXIT_FAILURE);

#ifdef INFO
	cout << "HOST-Info: Submitting Kernel MotionEstimation ..." << endl;
#endif
	errCode = clEnqueueTask(Command_Queue, K_MotionEstimation, 1, &K_exe_event[2], &K_exe_event[3]);
	if (errCode != CL_SUCCESS) return ErrHandler(err76, nullptr, EXIT_FAILURE);
	

// Submit Copy Results from Global Memory to Host

#ifdef INFO
	cout << "HOST_Info: Submitting Copy Results data from Global Memory to Host ..." << endl;
#endif

	errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_T_Mat, CL_MIGRATE_MEM_OBJECT_HOST, 1, &K_exe_event[4], &Mem_op_event[7]);
	if (errCode != CL_SUCCESS) return ErrHandler(err77, nullptr, EXIT_FAILURE);

	cout << endl
		 << "HOST_Info: Waiting for application to be completed ..." << endl;
	clFinish(Command_Queue);









}