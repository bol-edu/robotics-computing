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
#include <vector>
#include <string>

#include <CL/opencl.hpp>
#include <opencv2/opencv.hpp>
#include <common/xf_headers.hpp>
#include "Host.h"
#include "BasicFunction.hpp"
#include "help_functions.h"

using namespace std;

#define INFO

#define OCL_CHECK(error, call)  																			\
	call;                                                                                           		\
    if (error != CL_SUCCESS) {                                                                            	\
        fprintf(stderr, "%s:%d Error calling " #call ", error code is: %d\n", __FILE__, __LINE__, error); 	\
        exit(EXIT_FAILURE);                                                                               	\
    }


int main(int argc, const char** argv) {

	//	Check command line arguments, read images and set up parameters
	#ifdef INFO
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info: 				Check Command Line Arguments                  " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

	if (argc != 7){
		cout << "HOST-Error: Incorrect command line syntax " << endl << endl;
		return EXIT_FAILURE;
	}
	string Target_Platform_Vendor(argv[1]);
	string Target_Device_Name(argv[2]);
	string xclbinFilename(argv[3]);
	string Input_Image_Path_1(argv[4]);
	string Input_Image_Path_2(argv[5]);
	string Dataset_Frames_Num(argv[6]);
	cout << "HOST-Info: Platform_Vendor   	: " << Target_Platform_Vendor << endl;
	cout << "HOST-Info: Device_Name       	: " << Target_Device_Name << endl;
	cout << "HOST-Info: XCLBIN_file       	: " << xclbinFilename << endl;
	cout << "HOST-Info: Input_Image_Path_1  : " << Input_Image_Path_1 << endl;
	cout << "HOST-Info: Input_Image_Path_2  : " << Input_Image_Path_2 << endl;
	cout << "HOST-Info: Dataset_Frames_Num  : " << Dataset_Frames_Num << endl;
	cout << endl;

	vector<cv::Mat> Img_Left_Set, Img_Right_Set;
	int FRAME_NUM = stoi(Dataset_Frames_Num);
	for(int i=1; i<=FRAME_NUM; i++){
		string IMG_NAME("/");
		IMG_NAME = IMG_NAME + to_string(i) + ".jpg";
		Img_Left_Set.push_back(cv::imread(Input_Image_Path_1 + IMG_NAME, 0));
		if(Img_Left_Set[i-1].data==NULL){
			cout << "HOST-Error: Failed to open image " << Input_Image_Path_1 << IMG_NAME << endl << endl;
			return EXIT_FAILURE;
		}
		if(!Img_Left_Set[i-1].isContinuous()){
			cout << "HOST-Error: Discontinuous Image " << Input_Image_Path_1 << IMG_NAME << "" << endl << endl;
			return EXIT_FAILURE;
		}
		Img_Right_Set.push_back(cv::imread(Input_Image_Path_2 + IMG_NAME, 0));
		if(Img_Right_Set[i-1].data==NULL){
			cout << "HOST-Error: Failed to open image " << Input_Image_Path_2 << IMG_NAME << endl << endl;
			return EXIT_FAILURE;
		}
		if(!Img_Right_Set[i-1].isContinuous()){
			cout << "HOST-Error: Discontinuous Image " << Input_Image_Path_2 << IMG_NAME << "" << endl << endl;
			return EXIT_FAILURE;
		}
	}
	// Copy the data of images to a consecutive dynamic allocated array
	uchar* Img_Left_Arr = new uchar[FRAME_NUM * (Img_Left_Set[0].total())];
	uchar* Img_Right_Arr = new uchar[FRAME_NUM * (Img_Right_Set[0].total())];
	for(int i=0; i<FRAME_NUM; i++){
		uchar* psrc_l = Img_Left_Set[i].data;
		uchar* psrc_r = Img_Right_Set[i].data;
		uchar* pdst_l = Img_Left_Arr + Img_Left_Set[0].total() * i;
		uchar* pdst_r = Img_Left_Arr + Img_Right_Set[0].total() * i;
		memcpy(pdst_l, psrc_l, Img_Left_Set[0].total() * sizeof(uchar));
		memcpy(pdst_r, psrc_r, Img_Right_Set[0].total() * sizeof(uchar));
	}

	

	size_t image_in_size_bytes = Img_Left_Set[0].rows * Img_Left_Set[0].cols * sizeof(uchar);

    int height = in_imgL.rows;
    int width = in_imgL.cols;
    cout << "HOST-Info: Input image height : " << height << endl;
    cout << "HOST-Info: Input image width  : " << width << endl;


	// OpenCL section
	#ifdef INFO
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info:     				Run OpenCL Section 	  					  " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

	cl_int errCode;

	// Get the list of Devices of the given Platform 
	vector<cl::Device> Devices = BasicFunction::get_devices_of_platform(Target_Platform_Vendor);
	// Get the Specific Device 
	cl::Device Device = BasicFunction::get_specific_device(Target_Device_Name, Devices);


	// Create context, command queue and device name
	OCL_CHECK(errCode, cl::Context Context(Device, NULL, NULL, NULL, &errCode));								
	OCL_CHECK(errCode, cl::CommandQueue Queue(Context, Device, CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE | CL_QUEUE_PROFILING_ENABLE, &errCode));

	string Device_Name = Device.getInfo<CL_DEVICE_NAME>(&errCode); 
	OCL_CHECK(errCode, Device.getInfo<CL_DEVICE_NAME>(&errCode));

	cout << "INFO: Device found - " << Device_Name << endl;
    cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(IN_TYPE, NPC1) << endl;
    cout << "Input Image Channels:" << XF_CHANNELS(IN_TYPE, NPC1) << endl;
    cout << "NPPC:" << NPC1 << endl;

// Create Program and Kernel
	#ifdef INFO
		cout << endl;
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info:                  Create Program and Kernels                   " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

// Load binary file to memory and create program
	cl::Program::Binaries xcl_Binaries = BasicFunction::import_binary_file(xclbinFilename);
	Devices.clear();
	Devices.push_back(Device);
	Devices.resize(1);
	OCL_CHECK(errCode, cl::Program Program(Context, Devices, xcl_Binaries, NULL, &errCode));

// Create a kernels
	OCL_CHECK(errCode, cl::Kernel K_StereoMatching(Program, "K_StereoMatching", &errCode));
	OCL_CHECK(errCode, cl::Kernel K_FeatureExtraction(Program, "K_FeatureExtraction", &errCode));
	OCL_CHECK(errCode, cl::Kernel K_FeatureTracking(Program, "K_FeatureTracking", &errCode));
	OCL_CHECK(errCode, cl::Kernel K_MotionEstimation(Program, "K_MotionEstimation", &errCode));

/*
1 CL_MEM_READ_WRITE：在device上開闢一段kernal可讀可寫的內存，這是默認
2 CL_MEM_WRITE_ONLY：在device上開闢一段kernal只可以寫的內存
3 CL_MEM_READ_ONLY：在device上開闢一段kernal只可以讀的內存

4 CL_MEM_USE_HOST_PTR：直接使用host上一段已經分配的mem供device使用，注意：這裡雖然是用了host上已經存在的內存，但是這個內存的值不一定會和經過kernal函數計算後的實際的值，即使用clEnqueueReadBuffer函數拷貝回的內存和原本的內存是不一樣的，或者可以認為opencl雖然借用了這塊內存作為cl_mem，但是並不保證同步的，不過初始的值是一樣的，（可以使用mapmem等方式來同步）
5 CL_MEM_ALLOC_HOST_PTR：在host上新開闢一段內存供device使用
6 CL_MEM_COPY_HOST_PTR：在device上開闢一段內存供device使用，並賦值為host上一段已經存在的mem
 
7 CL_MEM_HOST_WRITE_ONLY:這塊內存是host只可寫的
8 CL_MEM_HOST_READ_ONLY:這塊內存是host只可讀的
9 CL_MEM_HOST_NO_ACCESS:這塊內存是host可讀可寫的
*/

uchar *ImgLeft_0=Img_Left_Arr, *ImgRight_0=Img_Right_Arr, *ImgLeft_1=Img_Left_Arr;
uchar *Mask, *K_Left, *T_Left, *T_Right, *Filter, *T_Mat;
int ConstArg_K, ConstArg_MaxDepth
	#ifdef INFO
		cout << endl;
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info:              Create Buffer and Set Kernel Args                " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif
// Allocate the buffers
	// does it need enqueuebuffer to sync?
	OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_0 (Context, /*CL_MEM_READ_ONLY |*/ CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_0, 	*ImgLeft_0, 	&errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_ImgRight_0(Context, /*CL_MEM_READ_ONLY |*/ CL_MEM_USE_HOST_PTR, SIZE_ImgRight_0, 	*ImgRight_0, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_1	(Context, /*CL_MEM_READ_ONLY |*/ CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_1, 	*ImgLeft_1, 	&errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_Mask		(Context, /*CL_MEM_READ_ONLY |*/ CL_MEM_USE_HOST_PTR, SIZE_Mask, 		*Mask, 		&errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_K_Left	(Context, /*CL_MEM_READ_ONLY |*/ CL_MEM_USE_HOST_PTR, SIZE_K_Left, 		*K_Left, 	&errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_T_Left	(Context, /*CL_MEM_READ_ONLY |*/ CL_MEM_USE_HOST_PTR, SIZE_T_Left, 		*T_Left, 	&errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_T_Right	(Context, /*CL_MEM_READ_ONLY |*/ CL_MEM_USE_HOST_PTR, SIZE_T_Right, 	*T_Right, 	&errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_Filter	(Context, /*CL_MEM_READ_ONLY |*/ CL_MEM_USE_HOST_PTR, SIZE_Filter, 		*Filter, 	&errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_T_Mat		(Context, /*CL_MEM_READ_ONLY |*/ CL_MEM_USE_HOST_PTR, SIZE_T_Mat, 		*T_Mat, 		&errCode));

	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Depth				(Context, /*CL_MEM_READ_WRITE?*/CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_Depth, 			NULL, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_KP0				(Context, CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_KP0, 				NULL, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_KP1				(Context, CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_KP1, 				NULL, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Des0				(Context, CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_Des0, 			NULL, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Des1				(Context, CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_Des1, 			NULL, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Detected_Points	(Context, CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_Detected_Points, 	NULL, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Matches			(Context, CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_Matches, 			NULL, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Detected_Matches	(Context, CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_Detected_Matches, NULL, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_rmat				(Context, CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_rmat, 			NULL, &errCode));
	OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_tvec				(Context, CL_MEM_ALLOC_HOST_PTR, SIZE_BUF_tvec, 			NULL, &errCode));

// Set kernel arguments
/*
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
*/
	OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(0, GlobMem_ImgLeft_0));
	OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(1, GlobMem_ImgRight_0));
	OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(2, GlobMem_K_Left));
	OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(3, GlobMem_T_Left));
	OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(4, GlobMem_T_Right));
	OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(5, GlobMem_BUF_Depth));

	OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(0, GlobMem_ImgLeft_0));
	OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(1, GlobMem_ImgLeft_1));
	OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(2, GlobMem_Mask));
	OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(3, GlobMem_BUF_KP0));
	OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(4, GlobMem_BUF_KP1));
	OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(5, GlobMem_BUF_Des0));
	OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(6, GlobMem_BUF_Des1));
	OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(7, GlobMem_BUF_Detected_Points));

	OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(0, GlobMem_Filter));
	OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(1, GlobMem_BUF_Des0));
	OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(2, GlobMem_BUF_Des1));
	OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(3, GlobMem_BUF_Detected_Points));
	OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(4, GlobMem_BUF_Matches));
	OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(5, GlobMem_BUF_Detected_Matches));
	OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(6, ConstArg_K));

	OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(0, GlobMem_K_Left));
	OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(1, GlobMem_BUF_Depth));
	OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(2, GlobMem_BUF_Matches));
	OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(3, GlobMem_BUF_Detected_Matches));
	OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(4, GlobMem_BUF_KP0));
	OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(5, GlobMem_BUF_KP1));
	OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(6, GlobMem_BUF_rmat));
	OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(7, GlobMem_BUF_tvec));
	OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(8, ConstArg_MaxDepth));

	
	// reference for clCreateBuffer 
	// https://stackoverflow.com/questions/61578885/whats-the-purpose-of-host-ptr-parameter-in-clcreatebuffer
	// https://www.twblogs.net/a/5b89bb3d2b71775d1ce384bf

	
	int Num_Mem_Events = 20, Num_Exe_Events = 4;
	vector<cl::Event> Krnl_Event_List(Num_Exe_Events);
	cl::Event K_StereoMatching_Event, K_FeatureExtraction_Event, K_FeatureTracking_Event, K_MotionEstimation_Event;


#ifdef INFO
	cout << endl;
	cout << "HOST-Info: ============================================================= " << endl;
	cout << "HOST-Info:  					  Run Application                         " << endl;
	cout << "HOST-Info: ============================================================= " << endl;
#endif

	// Submit Kernels for Execution
	for(int i=0; i<FRAME_NUM; i++){
		
		OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_StereoMatching, NULL, &K_StereoMatching_Event));
		Krnl_Event_List[0] = K_StereoMatching_Event;
		OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_FeatureExtraction, &Krnl_Event_List, &K_FeatureExtraction_Event));
		Krnl_Event_List[0] = K_FeatureExtraction_Event;
		OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_FeatureTracking, &Krnl_Event_List, &K_FeatureTracking_Event));
		Krnl_Event_List[0] = K_FeatureTracking_Event;
		OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_MotionEstimation, &Krnl_Event_List, &K_MotionEstimation_Event));
		Krnl_Event_List[0] = K_MotionEstimation_Event;
		Queue.finish();
	}

	








}