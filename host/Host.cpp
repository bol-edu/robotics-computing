#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//#include <unistd.h>
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
//#include <common/xf_headers.hpp>
#include "Host.hpp"
#include "BasicFunction.hpp"
#include "C_StereoMatching/Methods.h"			//stereo_2_depth(unsigned char* image_left_test_data, unsigned char* image_right_test_data, unsigned char* k_left_test_data, unsigned char* t_left_test_data, unsigned char* t_right_test_data, bool matcher, bool rgb, bool rectified, unsigned char* depth_map);
#include "C_FeatureExtraction/Methods.h"		//extract_features(unsigned char* image_data, unsigned char* mask_data, float* kp_xy, unsigned char* des_data);
#include "C_FeatureTracking/FeatureMatching.h"	//match_feature(uchar *des0, uchar *des1, FLOAT dist_threshold, int32 *match);
#include "C_MotionEstimation/EstimateMotion.h"	//EstimateMotion::estimate(Matrix &match, Matrix &kp0, Matrix &kp1, Matrix &k, Matrix &depth, Matrix &rmat, Matrix &tvec);
#include "C_MotionEstimation/Matrix.h"
using namespace std;

int main(int argc, const char** argv) {

	//	Check command line arguments, read images and set up parameters
	#ifdef _INFO_
		#ifdef _PURE_C
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info: 				     Pure C Compilation                       " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#ifdef _ALL_KERNELS_
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info: 		    	   	Compile All Kernels                       " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
		#else
		#ifdef _ONLY_K_StereoMatching_
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info: 		   		ONLY Compile K_StereoMatching                 " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#ifdef _ONLY_K_FeatureExtraction_
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info: 		   		ONLY Compile K_FeatureExtraction              " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#ifdef _ONLY_K_FeatureTracking_
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info: 		   		ONLY Compile K_FeatureTracking                " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#ifdef _ONLY_K_MotionEstimation_
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info: 		   		ONLY Compile K_MotionEstimation               " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#endif  
			std::cout << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info: 				Check Command Line Arguments                  " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
	#endif


	#ifdef _PURE_C_
		if (argc != 5){
			std::cout << "HOST-Error: Incorrect command line syntax " << endl << endl;
			return EXIT_FAILURE;
		}
	#else 
		if (argc != 8){
			std::cout << "HOST-Error: Incorrect command line syntax " << endl << endl;
			return EXIT_FAILURE;
		}
	#endif
		string Input_Image_Path_1(argv[1]);
		string Input_Image_Path_2(argv[2]);
		string Calibration_Path(argv[3]);
		string Dataset_Frames_Num(argv[4]);
	#ifndef _PURE_C_
		string Target_Platform_Vendor(argv[5]);
		string Target_Device_Name(argv[6]);
		string xclbinFilename(argv[7]);
	#endif
		std::cout << endl;
		std::cout << "HOST-Info: Input_Image_Path_1  : " << Input_Image_Path_1 << endl;
		std::cout << "HOST-Info: Input_Image_Path_2  : " << Input_Image_Path_2 << endl;
		std::cout << "HOST-Info: Calibration_Path  	: " << Calibration_Path << endl;
		std::cout << "HOST-Info: Dataset_Frames_Num  : " << Dataset_Frames_Num << endl;
	#ifndef _PURE_C_
		std::cout << "HOST-Info: Platform_Vendor   	: " << Target_Platform_Vendor << endl;
		std::cout << "HOST-Info: Device_Name       	: " << Target_Device_Name << endl;
		std::cout << "HOST-Info: XCLBIN_file       	: " << xclbinFilename << endl;
	#endif
		int FRAME_NUM = stoi(Dataset_Frames_Num);

		cv::Mat K_Left, R_Left, T_Left;
		cv::Mat K_Right, R_Right, T_Right;
		cv::Mat P0, P1;
		cv::Mat ImgLeft_0, ImgRight_0, ImgLeft_1;
		cv::Mat Depth, Mask;
		int height, width;
	#ifndef _PURE_C_
		size_t Image_Bytes;
	#endif
	
// Read calibration file
	BasicFunction::read_calibration((const char*)Calibration_Path.c_str(), &P0, &P1);
	#ifdef _INFO_
		std::cout << endl << "HOST-Info: Read calibration successfully " << endl;
	#endif
// Collect all images in the given directory
	vector<string> Img_Left_Set = BasicFunction::read_image_folder(FRAME_NUM, Input_Image_Path_1);
	vector<string> Img_Right_Set = BasicFunction::read_image_folder(FRAME_NUM, Input_Image_Path_2);
	#ifdef _INFO_
		std::cout << endl << "HOST-Info: Read image set successfully " << endl;
	#endif
// Read the first image
	ImgLeft_1 = cv::imread(Img_Left_Set[0], cv::IMREAD_GRAYSCALE);


// Set parameters
	height = ImgLeft_1.rows;
	width = ImgLeft_1.cols;
	
	Mask = cv::Mat::zeros(height, width, CV_8U);
	Mask(cv::Rect(96, 0, width - 96, height)) = 255;

	cv::Mat T_tot = cv::Mat::eye(4, 4, CV_64F);
	vector<cv::Mat> trajectory(FRAME_NUM);
	cv::Rect rect(0, 0, 4, 3);
	T_tot(rect).copyTo(trajectory[0]);

// Decompose Projection Matrix
	cv::decomposeProjectionMatrix(P0, K_Left, R_Left, T_Left);
	cv::decomposeProjectionMatrix(P1, K_Right, R_Right, T_Right);
	T_Left = T_Left / (T_Left.at<float>(3));
	T_Right = T_Right / (T_Right.at<float>(3));

	size_t Image_Bytes = height * width * sizeof(uchar);
	#ifdef _INFO_
		std::cout  << endl;
		std::cout << "HOST-Info: Input image height : " << height << endl;
		std::cout << "HOST-Info: Input image width  : " << width << endl;
	#endif

// Set up OpenCL
	#ifndef _PURE_C_
		#ifdef _INFO_
			std::cout << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info:     				Run OpenCL Section 	  					  " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << endl;
			#ifdef _ALL_KERNELS_
				std::cout << "HOST-Info: ============================================================= " << endl;
				std::cout << "HOST-Info: 		    	 	Set Up All Kernels                        " << endl;
				std::cout << "HOST-Info: ============================================================= " << endl;
			#else
				#ifdef _ONLY_K_StereoMatching_
					std::cout << "HOST-Info: ============================================================= " << endl;
					std::cout << "HOST-Info: 		   			Set Up K_StereoMatching                   " << endl;
					std::cout << "HOST-Info: ============================================================= " << endl;
				#endif
				#ifdef _ONLY_K_FeatureExtraction_
					std::cout << "HOST-Info: ============================================================= " << endl;
					std::cout << "HOST-Info: 		   			Set Up K_FeatureExtraction                " << endl;
					std::cout << "HOST-Info: ============================================================= " << endl;
				#endif
				#ifdef _ONLY_K_FeatureTracking_
					std::cout << "HOST-Info: ============================================================= " << endl;
					std::cout << "HOST-Info: 		   			Set Up K_FeatureTracking                  " << endl;
					std::cout << "HOST-Info: ============================================================= " << endl;
				#endif
				#ifdef _ONLY_K_MotionEstimation_
					std::cout << "HOST-Info: ============================================================= " << endl;
					std::cout << "HOST-Info: 		   			Set Up K_MotionEstimation                 " << endl;
					std::cout << "HOST-Info: ============================================================= " << endl;
				#endif
			#endif  	
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

		std::cout << "INFO: Device found - " << Device_Name << endl;
		std::cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(IN_TYPE, NPC1) << endl;
		std::cout << "Input Image Channels:" << XF_CHANNELS(IN_TYPE, NPC1) << endl;
		std::cout << "NPPC:" << NPC1 << endl;

		// Create Program and Kernel
		#ifdef _INFO_
			std::cout << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info:                  Create Program and Kernel                    " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
		#endif

		// Load binary file to memory and create program
		cl::Program::Binaries xcl_Binaries = BasicFunction::import_binary_file(xclbinFilename);
		Devices.clear();
		Devices.push_back(Device);
		Devices.resize(1);
		OCL_CHECK(errCode, cl::Program Program(Context, Devices, xcl_Binaries, NULL, &errCode));

		// Create one or more kernels
		#ifdef _ONLY_K_StereoMatching_
			OCL_CHECK(errCode, cl::Kernel K_StereoMatching(Program, "K_StereoMatching", &errCode));
		#endif
		#ifdef _ONLY_K_FeatureExtraction_
			OCL_CHECK(errCode, cl::Kernel K_FeatureExtraction(Program, "K_FeatureExtraction", &errCode));
		#endif
		#ifdef _ONLY_K_FeatureTracking_
			OCL_CHECK(errCode, cl::Kernel K_FeatureTracking(Program, "K_FeatureTracking", &errCode));
		#endif
		#ifdef _ONLY_K_MotionEstimation_
			OCL_CHECK(errCode, cl::Kernel K_MotionEstimation(Program, "K_MotionEstimation", &errCode));
		#endif

		// Create Buffer and Set Kernel Args 
		#ifdef _INFO_
			std::cout << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
			std::cout << "HOST-Info:              Create Buffer and Set Kernel Args                " << endl;
			std::cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#ifdef _ALL_KERNELS_
			uchar *ImgLeft_0, *ImgRight_0, *ImgLeft_1, *Mask, *K_Left, *T_Left, *T_Right, *Filter, *T_Mat;
			int ConstArg_K, ConstArg_MaxDepth;
			OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_0 (Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_0, 	*ImgLeft_0, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_ImgRight_0(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgRight_0, 	*ImgRight_0,&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_1	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_1, 	*ImgLeft_1, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_Mask		(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_Mask, 		*Mask, 		&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_K_Left	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_K_Left, 		*K_Left, 	&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_T_Left	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_T_Left, 		*T_Left, 	&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_T_Right	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_T_Right, 	*T_Right, 	&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_Filter	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_Filter, 		*Filter, 	&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_T_Mat		(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_T_Mat, 		*T_Mat, 	&errCode));

			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Depth				(Context, CL_MEM_READ_WRITE , SIZE_BUF_Depth, 			NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_KP0				(Context, CL_MEM_READ_WRITE , SIZE_BUF_KP0, 			NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_KP1				(Context, CL_MEM_READ_WRITE , SIZE_BUF_KP1, 			NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Des0				(Context, CL_MEM_READ_WRITE , SIZE_BUF_Des0, 			NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Des1				(Context, CL_MEM_READ_WRITE , SIZE_BUF_Des1, 			NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Detected_Points	(Context, CL_MEM_READ_WRITE , SIZE_BUF_Detected_Points, NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Matches			(Context, CL_MEM_READ_WRITE , SIZE_BUF_Matches, 		NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Detected_Matches	(Context, CL_MEM_READ_WRITE , SIZE_BUF_Detected_Matches, NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_rmat				(Context, CL_MEM_READ_WRITE , SIZE_BUF_rmat, 			NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_tvec				(Context, CL_MEM_READ_WRITE , SIZE_BUF_tvec, 			NULL, &errCode));

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
		#else
			#ifdef _ONLY_K_StereoMatching_
				uchar *ImgLeft_0, *ImgRight_0, *K_Left, *T_Left, *T_Right, *Depth;
				OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_0 (Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_0, 	*ImgLeft_0, &errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_ImgRight_0(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgRight_0, 	*ImgRight_0,&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_K_Left	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_K_Left, 		*K_Left, 	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_T_Left	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_T_Left, 		*T_Left, 	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_T_Right	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_T_Right, 	*T_Right, 	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Depth	(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_T_Mat, 		*Depth, 	&errCode));

				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(0, GlobMem_ImgLeft_0));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(1, GlobMem_ImgRight_0));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(2, GlobMem_K_Left));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(3, GlobMem_T_Left));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(4, GlobMem_T_Right));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(5, GlobMem_Depth));
			#endif
			#ifdef _ONLY_K_FeatureExtraction_
				uchar *ImgLeft_0, *ImgLeft_1, *Mask, *KP0, *KP1, *Des0, *Des1, *Detected_Points;
				OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_0 		(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_0, 	*ImgLeft_0, &errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_1			(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_1, 	*ImgLeft_1, &errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Mask				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_Mask, 		*Mask, 		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_KP0				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_K_Left, 		*KP0, 		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_KP1				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_T_Left, 		*KP1, 		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Des0				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_T_Right, 	*Des0, 		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Des1				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_Filter, 		*Des1, 		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Detected_Points	(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_T_Mat, 		*T_Mat, 	&errCode));

				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(0, GlobMem_ImgLeft_0));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(1, GlobMem_ImgLeft_1));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(2, GlobMem_Mask));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(3, GlobMem_KP0));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(4, GlobMem_KP1));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(5, GlobMem_Des0));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(6, GlobMem_Des1));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(7, GlobMem_Detected_Points));
			#endif
			#ifdef _ONLY_K_FeatureTracking_
				uchar *Filter, *Des0, *Des1, *Detected_Points, *Matches, *Detected_Matches;
				int ConstArg_K;
				OCL_CHECK(errCode, cl::Buffer GlobMem_Filter 			(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_0, 	*Filter,			&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Des0				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgRight_0, 	*Des0,				&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Des1				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_1, 	*Des1, 				&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Detected_Points	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_Mask, 		*Detected_Points, 	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Matches			(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_K_Left, 		*Matches, 			&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Detected_Matches	(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_T_Left, 		*Detected_Matches, 	&errCode));

				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(0, GlobMem_Filter));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(1, GlobMem_Des0));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(2, GlobMem_Des1));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(3, GlobMem_Detected_Points));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(4, GlobMem_Matches));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(5, GlobMem_Detected_Matches));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(6, ConstArg_K));
			#endif
			#ifdef _ONLY_K_MotionEstimation_
				uchar *K_Left, *Depth, *Matches, *Detected_Matches, *KP0, *KP1, *rmat, *tvec, *T_Mat;
				int ConstArg_MaxDepth;
				OCL_CHECK(errCode, cl::Buffer GlobMem_K_Left 			(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_0, 	*K_Left, 			&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Depth				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgRight_0, 	*Depth,				&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Matches			(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_ImgLeft_1, 	*Matches, 			&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Detected_Matches	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_Mask, 		*Detected_Matches, 	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_KP0				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_K_Left, 		*KP0, 				&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_KP1				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, SIZE_T_Left, 		*KP1, 				&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_rmat				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_T_Right, 	*rmat, 				&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_tvec				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, SIZE_Filter, 		*tvec, 				&errCode));

				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(0, GlobMem_K_Left));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(1, GlobMem_Depth));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(2, GlobMem_Matches));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(3, GlobMem_Detected_Matches));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(4, GlobMem_KP0));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(5, GlobMem_KP1));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(6, GlobMem_rmat));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(7, GlobMem_tvec));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(8, ConstArg_MaxDepth));
			#endif
		#endif
	#endif

	
	int Num_Mem_Events = 20, Num_Exe_Events = 4;
	vector<cl::Event> Krnl_Event_List(Num_Exe_Events);
	cl::Event K_StereoMatching_Event, K_FeatureExtraction_Event, K_FeatureTracking_Event, K_MotionEstimation_Event;


	
	#ifdef _INFO_
	std::cout << endl;
	std::cout << "HOST-Info: ============================================================= " << endl;
	std::cout << "HOST-Info:  					  Run Application                         " << endl;
	std::cout << "HOST-Info: ============================================================= " << endl;
	#endif

	ofstream fout_KP0, fout_KP1, fout_Des0, fout_Des1, fout;
	//fout.open("Mask.txt", ios::out);
	//fout << Mask;

	for(int i=0; i<10; i++){

		//ImgLeft_0 = ImgLeft_1;
		ImgLeft_0 = cv::imread(Img_Left_Set[i], cv::IMREAD_GRAYSCALE);
		cout << Img_Left_Set[i] << "\n";
		fout.close();
		ImgRight_0 = cv::imread(Img_Right_Set[i], cv::IMREAD_GRAYSCALE);
		ImgLeft_1 = cv::imread(Img_Left_Set[i+1], cv::IMREAD_GRAYSCALE);

		cout << "iteration: " << i << endl;

//stereo_2_depth(Mat img_left, Mat img_right, Mat k_left, Mat t_left, Mat t_right, bool matcher, bool rgb, bool rectified)
//extract_features(unsigned char* image_data, unsigned char* mask_data, float* kp_xy, unsigned char* des_data);
//match_feature(uchar *des0, uchar *des1, FLOAT dist_threshold, int32 *match);
//EstimateMotion::estimate(Matrix &match, Matrix &kp0, Matrix &kp1, Matrix &k, Matrix &depth, Matrix &rmat, Matrix &tvec);
		
		#ifdef _ONLY_K_StereoMatching_ 
			/**
			 * 	
			 * Extract the Mat.data pointer
			 * Migrate the memory object to FPGA GlobalMem
			 * Submit kernel function
			 * Enqueue the task
			 * Transfer the result to host
			 * 
			 */
		#else
			cv::Mat Depth(height, width, CV_8U);

			//stereo_2_depth((unsigned char*)ImgLeft_0.data, (unsigned char*)ImgRight_0.data, (unsigned char*)K_Left.data, (unsigned char*)T_Left.data, (unsigned char*)T_Right.data, (bool)1, (bool)0, (bool)1, (unsigned char*)Depth.data);
			
			
		#endif




		#ifdef _ONLY_K_FeatureExtraction_ 
			/**
			 * 	
			 * Extract the Mat.data pointer
			 * Migrate the memory object to FPGA GlobalMem
			 * Submit kernel function
			 * Enqueue the task
			 * Transfer the result to host
			 * 
			 */
		#else
			float *KP0 = new float[1000];
			float *KP1 = new float[1000];
			cv::Mat Des0(500, 32, CV_8U);
			cv::Mat Des1(500, 32, CV_8U);
			extract_features(ImgLeft_0.data, Mask.data, KP0, Des0.data);
			extract_features(ImgLeft_1.data, Mask.data, KP1, Des1.data);
			/*
			fout_KP0.open("KP0_iteration0.txt", ios::out);
			for (int i = 0; i < 499; i++) {
				fout_KP0 << KP0[i*2] << " " << KP0[i*2 + 1] << endl;
			}
			fout_KP0.close();

			
			fout_KP1.open("KP1_iteration0.txt", ios::out);
			for (int i = 0; i < 499; i++) {
				fout_KP1 << KP1[i*2] << " " << KP1[i*2 + 1] << endl;
			}
			fout_KP1.close();


			fout_Des0.open("Des0_iteration0.txt", ios::out);
			fout_Des0 << Des0;
			fout_Des0.close();



			fout_Des1.open("Des1_iteration0.txt", ios::out);
			fout_Des1 << Des1;
			fout_Des1.close();
			*/

			/*
			cout << "KP0: ";
			for (int i = 0; i < 1000; i++) {
				cout << KP0[i] << " ";
			}
			cout << "\n\n";
			cout << "KP1: ";
			for (int i = 0; i < 1000; i++) {
				cout << KP1[i] << " ";
			}
			cout << "\n\n";
			cout << "Des0: " << "\n";
			cout << Des0;
			cout << "\n\n";
			cout << "Des1: " << "\n";
			cout << Des1;
			cout << "\n\n";
			*/

		#endif




		#ifdef _ONLY_K_FeatureTracking_ 
			/**
			 * 	
			 * Extract the Mat.data pointer
			 * Migrate the memory object to FPGA GlobalMem
			 * Submit kernel function
			 * Enqueue the task
			 * Transfer the result to host
			 * 
			 */
		#else
			float *Matches = new float[1000];
			match_feature(Des0.data, Des1.data, Filter, (int32*)Matches);
		#endif




		#ifdef _ONLY_K_MotionEstimation_ 
			/**
			 * 	
			 * Extract the Mat.data pointer
			 * Migrate the memory object to FPGA GlobalMem
			 * Submit kernel function
			 * Enqueue the task
			 * Transfer the result to host
			 * 
			 */
		#else

			/*
			cv::Mat rmat(3, 3, CV_8U);
			cv::Mat tvec(3, 1, CV_8U);
			EstimateMotion motion;

			Matrix matches, kp0, kp1, kleft, depth, Rmat, Tvec;
			for (int i = 0; i < 1; i++) {
				for (int j = 0; j < 1000; j++) {
					matches.val[i][j] = Matches[i];
					kp0.val[i][j] = KP0[i];
					kp1.val[i][j] = KP1[i];
				}
			}

			for (int i = 0; i < K_Left.rows; i++) {
				for (int j = 0;j < K_Left.cols; j++) {
					kleft.val[i][j] = K_Left.data[i* K_Left.cols + j];
				}
			}

			for (int i = 0; i < Depth.rows; i++) {
				for (int j = 0;j < Depth.cols; j++) {
					depth.val[i][j] = Depth.data[i * Depth.cols + j];
				}
			}

			motion.estimate(matches, kp0, kp1, kleft, depth, Rmat, Tvec);

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					rmat.data[i*3 + j] = Rmat.val[i][j];
				}
				tvec.data[i] = Tvec.val[i][0];
			}
			*/
		#endif
		

		/*
		cv::Mat T_mat;
		cv::Mat I4 = cv::Mat::eye(4, 4, CV_64F);
		cv::hconcat(rmat, tvec, T_mat);
		cv::vconcat(T_mat, I4.row(3), T_mat);
		T_tot = T_tot * T_mat.inv();
		T_tot(rect).copyTo(trajectory[i+1]);
		*/

/*
		FILE* fp;
		fopen_s(&fp, "../output/trajectory.txt", "w");

		for (int i = 0; i < trajectory.size(); i++)
		{
			fprintf_s(fp, "%lf\t%lf\t%lf\n", 
				trajectory.at(i).at<double>(0, 3), 
				trajectory.at(i).at<double>(1, 3), 
				trajectory.at(i).at<double>(2, 3));
		}

		fclose(fp);


		Depth.release();
		Des0.release();
		Des1.release();
		//rmat.release();
		//tvec.release();
		delete []KP0;
		delete []KP1;
		delete []Matches;
	*/
	}



/**
 * // Submit Kernels for Execution
	for(int i=0; i<FRAME_NUM; i++){
		// read image 
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
 * 
 */
	

	








}

