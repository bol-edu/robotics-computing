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
#include "Host.hpp"
#include "BasicFunction.hpp"
#include "help_functions.h"
using namespace std;

#ifdef _PURE_C_
int main(int argc, const char** argv){
	//	Check command line arguments, read images and set up parameters
	#ifdef _INFO_
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info: 				     Pure C Compilation                       " << endl;
		cout << "HOST-Info: ============================================================= " << endl << endl;
		cout << "HOST-Info: ============================================================= " << endl;
		cout << "HOST-Info: 				Check Command Line Arguments                  " << endl;
		cout << "HOST-Info: ============================================================= " << endl;
	#endif

	if (argc != 5){
		cout << "HOST-Error: Incorrect command line syntax " << endl << endl;
		return EXIT_FAILURE;
	}
	string Input_Image_Path_1(argv[1]);
	string Input_Image_Path_2(argv[2]);
	string Calibration_Path(argv[3]);
	string Dataset_Frames_Num(argv[4]);
	cout << "HOST-Info: Input_Image_Path_1  : " << Input_Image_Path_1 << endl;
	cout << "HOST-Info: Input_Image_Path_2  : " << Input_Image_Path_2 << endl;
	cout << "HOST-Info: Calibration_Path  	: " << Calibration_Path << endl;
	cout << "HOST-Info: Dataset_Frames_Num  : " << Dataset_Frames_Num << endl;
	cout << endl;
	int FRAME_NUM = stoi(Dataset_Frames_Num);

	


	cv::Mat K_Left, R_Left, T_Left;
	cv::Mat K_Right, R_Right, T_Right;
	cv::Mat P0, P1;
	
	BasicFunction::read_calibration((const char*)Calibration_Path.c_str(), &P0, P1);
	vector<cv::Mat> Img_Left_Set = BasicFunction::read_image_folder(FRAME_NUM, Input_Image_Path_1);
	vector<cv::Mat> Img_Right_Set = BasicFunction::read_image_folder(FRAME_NUM, Input_Image_Path_2);

	cv::Mat ImgLeft_0, ImgRight_0, ImgLeft_1, Depth;
	ImgLeft_1 = imread(Img_Left_Set[i], IMREAD_GRAYSCALE);

	int height = ImgLeft_0.cols;
	int width = Img_Left_0.rows;
	
	cv::Mat Mask = cv::Mat::zeros(height, width, CV_8U);
	Mask(cv::Rect(96, 0, height - 96, width)) = 255;

	cv::Mat T_tot = cv::Mat::eye(4, 4, CV_64F);
	vector<cv::Mat> trajectory(FRAME_NUM);
	cv::Rect rect(0, 0, 4, 3);
	T_tot(rect).copyTo(trajectory[0]);


	cv::decompose_Projection_Matrix(P0, &K_Left, &R_Left, &T_Left);
	cv::decompose_Projection_Matrix(P1, &K_Right, &R_Right, &T_Right);


	for(int i=0; i<FRAME_NUM; i++){



		ImgLeft_0 = ImgLeft_1;
		ImgRight_0 = cv::imread(Img_Right_Set[i], IMREAD_GRAYSCALE);
		ImgLeft_1 = cv::imread(Img_left_Set[i], IMREAD_GRAYSCALE);
		Depth = C_StereoMatching(ImgLeft_0, ImgRight_0, K_Left, T_Left, T_Right);




		vector<cv::Mat> KP0, KP1;
		cv::Mat Des0, Des1;
		int Detected_Points = C_FeatureExtraction(ImgLeft_0, ImgLeft_1, Mask, &KP0, &KP1, &Des0, &Des1);



		float Filter;
		vector<vector<cv::Mat>> Matches;
		int Detected_Matches = C_FeatureTracking(Filter, &Des0, &Des1, Detected_Points, Matches);



		int Max_Depth;
		Mat rmat, tvec, Img1_Points, Img2_Points;
		C_MotionEstimation(K_Left, Depth, Matches, KP0, KP1, Max_Depth, rmat, tvec, Img1_Points, Img2_Points);




		Mat T_mat;
		Mat I4 = Mat::eye(4, 4, CV_64F);
		hconcat(rmat, tvec, T_mat);
		vconcat(T_mat, I4.row(3), T_mat);
		T_tot = T_tot * T_mat.inv();
		T_tot(rect).copyTo(trajectory[i+1]);

	}

	FILE* fp;
	fopen_s(&fp, "../output/trajectory_nolidar_bm.txt", "w");

	for (int i = 0; i < trajectory.size(); i++)
	{
		fprintf_s(fp, "%lf\t%lf\t%lf\n", 
			trajectory.at(i).at<double>(0, 3), 
			trajectory.at(i).at<double>(1, 3), 
			trajectory.at(i).at<double>(2, 3));
	}

	fclose(fp);


}

// Compile one or more Kernels
#else
int main(int argc, const char** argv) {

	//	Check command line arguments, read images and set up parameters
	#ifdef _INFO_
		#ifdef _PURE_C
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info: 				     Pure C Compilation                       " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#ifdef _ALL_KERNELS_
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info: 		    	   	Compile All Kernels                       " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
		#else
		#ifdef _ONLY_K_StereoMatching_
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info: 		   		ONLY Compile K_StereoMatching                 " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#ifdef _ONLY_K_FeatureExtraction_
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info: 		   		ONLY Compile K_FeatureExtraction              " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#ifdef _ONLY_K_FeatureTracking_
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info: 		   		ONLY Compile K_FeatureTracking                " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#ifdef _ONLY_K_MotionEstimation_
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info: 		   		ONLY Compile K_MotionEstimation               " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
		#endif
		#endif  
			cout << endl;
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info: 				Check Command Line Arguments                  " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
	#endif


	#ifdef _PURE_C_
		if (argc != 5){
			cout << "HOST-Error: Incorrect command line syntax " << endl << endl;
			return EXIT_FAILURE;
		}
	#else 
		if (argc != 8){
			cout << "HOST-Error: Incorrect command line syntax " << endl << endl;
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
		cout << endl;
		cout << "HOST-Info: Input_Image_Path_1  : " << Input_Image_Path_1 << endl;
		cout << "HOST-Info: Input_Image_Path_2  : " << Input_Image_Path_2 << endl;
		cout << "HOST-Info: Calibration_Path  	: " << Calibration_Path << endl;
		cout << "HOST-Info: Dataset_Frames_Num  : " << Dataset_Frames_Num << endl;
	#ifndef _PURE_C_
		cout << "HOST-Info: Platform_Vendor   	: " << Target_Platform_Vendor << endl;
		cout << "HOST-Info: Device_Name       	: " << Target_Device_Name << endl;
		cout << "HOST-Info: XCLBIN_file       	: " << xclbinFilename << endl;
	#endif
		int FRAME_NUM = stoi(Dataset_Frames_Num);

		cv::Mat K_Left, R_Left, T_Left;
		cv::Mat K_Right, R_Right, T_Right;
		cv::Mat P0, P1;
		cv::Mat ImgLeft_0, ImgRight_0, ImgLeft_1;
		cv::Mat Depth, Mask;
		int height, width;
	#ifndef _PURE_C_
		size_t Image_Bytes
	#endif
	
// Read calibration file
	BasicFunction::read_calibration((const char*)Calibration_Path.c_str(), &P0, P1);
	#ifdef _INFO_
		cout << endl << "HOST-Info: Read calibration successfully " << endl;
	#endif
// Collect all images in the given directory
	vector<cv::Mat> Img_Left_Set = BasicFunction::read_image_folder(FRAME_NUM, Input_Image_Path_1);
	vector<cv::Mat> Img_Right_Set = BasicFunction::read_image_folder(FRAME_NUM, Input_Image_Path_2);
	#ifdef _INFO_
		cout << endl << "HOST-Info: Read image set successfully " << endl;
	#endif
// Read the first image
	ImgLeft_1 = imread(Img_Left_Set[i], IMREAD_GRAYSCALE);
// Set parameters
	height = ImgLeft_0.cols;
	width = ImgLeft_0.rows;

	Mask = cv::Mat::zeros(height, width, CV_8U);
	Mask(cv::Rect(96, 0, height - 96, width)) = 255;

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
		cout  << endl;
		cout << "HOST-Info: Input image height : " << height << endl;
		cout << "HOST-Info: Input image width  : " << width << endl;
	#endif

// Set up OpenCL
	#ifndef _PURE_C_
		#ifdef _INFO_
			cout << endl;
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info:     				Run OpenCL Section 	  					  " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
			cout << endl;
			#ifdef _ALL_KERNELS_
				cout << "HOST-Info: ============================================================= " << endl;
				cout << "HOST-Info: 		    	 	Set Up All Kernels                        " << endl;
				cout << "HOST-Info: ============================================================= " << endl;
			#else
				#ifdef _ONLY_K_StereoMatching_
					cout << "HOST-Info: ============================================================= " << endl;
					cout << "HOST-Info: 		   			Set Up K_StereoMatching                   " << endl;
					cout << "HOST-Info: ============================================================= " << endl;
				#endif
				#ifdef _ONLY_K_FeatureExtraction_
					cout << "HOST-Info: ============================================================= " << endl;
					cout << "HOST-Info: 		   			Set Up K_FeatureExtraction                " << endl;
					cout << "HOST-Info: ============================================================= " << endl;
				#endif
				#ifdef _ONLY_K_FeatureTracking_
					cout << "HOST-Info: ============================================================= " << endl;
					cout << "HOST-Info: 		   			Set Up K_FeatureTracking                  " << endl;
					cout << "HOST-Info: ============================================================= " << endl;
				#endif
				#ifdef _ONLY_K_MotionEstimation_
					cout << "HOST-Info: ============================================================= " << endl;
					cout << "HOST-Info: 		   			Set Up K_MotionEstimation                 " << endl;
					cout << "HOST-Info: ============================================================= " << endl;
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

		cout << "INFO: Device found - " << Device_Name << endl;
		cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(IN_TYPE, NPC1) << endl;
		cout << "Input Image Channels:" << XF_CHANNELS(IN_TYPE, NPC1) << endl;
		cout << "NPPC:" << NPC1 << endl;

		// Create Program and Kernel
		#ifdef _INFO_
			cout << endl;
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info:                  Create Program and Kernel                    " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
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
			cout << endl;
			cout << "HOST-Info: ============================================================= " << endl;
			cout << "HOST-Info:              Create Buffer and Set Kernel Args                " << endl;
			cout << "HOST-Info: ============================================================= " << endl;
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
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(1, GlobMem_BUF_Depth));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(2, GlobMem_BUF_Matches));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(3, GlobMem_BUF_Detected_Matches));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(4, GlobMem_BUF_KP0));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(5, GlobMem_BUF_KP1));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(6, GlobMem_BUF_rmat));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(7, GlobMem_BUF_tvec));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(8, ConstArg_MaxDepth));
			#endif
		#endif
	#endif

	
	int Num_Mem_Events = 20, Num_Exe_Events = 4;
	vector<cl::Event> Krnl_Event_List(Num_Exe_Events);
	cl::Event K_StereoMatching_Event, K_FeatureExtraction_Event, K_FeatureTracking_Event, K_MotionEstimation_Event;


	
	#ifdef _INFO_
	cout << endl;
	cout << "HOST-Info: ============================================================= " << endl;
	cout << "HOST-Info:  					  Run Application                         " << endl;
	cout << "HOST-Info: ============================================================= " << endl;
	#endif
	for(int i=0; i<FRAME_NUM; i++){



		ImgLeft_0 = ImgLeft_1;
		ImgRight_0 = cv::imread(Img_Right_Set[i], IMREAD_GRAYSCALE);
		ImgLeft_1 = cv::imread(Img_left_Set[i], IMREAD_GRAYSCALE);
		Depth = C_StereoMatching(ImgLeft_0, ImgRight_0, K_Left, T_Left, T_Right);




		vector<cv::Mat> KP0, KP1;
		cv::Mat Des0, Des1;
		int Detected_Points = C_FeatureExtraction(ImgLeft_0, ImgLeft_1, Mask, &KP0, &KP1, &Des0, &Des1);



		float Filter;
		vector<vector<cv::Mat>> Matches;
		int Detected_Matches = C_FeatureTracking(Filter, &Des0, &Des1, Detected_Points, Matches);



		int Max_Depth;
		Mat rmat, tvec, Img1_Points, Img2_Points;
		C_MotionEstimation(K_Left, Depth, Matches, KP0, KP1, Max_Depth, rmat, tvec, Img1_Points, Img2_Points);




		Mat T_mat;
		Mat I4 = Mat::eye(4, 4, CV_64F);
		hconcat(rmat, tvec, T_mat);
		vconcat(T_mat, I4.row(3), T_mat);
		T_tot = T_tot * T_mat.inv();
		T_tot(rect).copyTo(trajectory[i+1]);

	}




	// Submit Kernels for Execution
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

	








}

#endif

