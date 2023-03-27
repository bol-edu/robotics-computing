#include "Host.hpp"
#include "BasicFunction.hpp"
#include "Parameter.hpp"



using namespace std;

int main(int argc, const char** argv) {

	//	Check command line arguments, read images and set up parameters
	#ifdef _INFO_
		#ifdef _PURE_C
			std::cout << " ============================================================= " << endl;
			std::cout << " ----------------------Pure C Compilation--------------------- " << endl;
			std::cout << " ============================================================= " << endl;
		#endif
		#ifdef _ALL_KERNELS_
			std::cout << " ============================================================= " << endl;
			std::cout << " ---------------------Compile All Kernels--------------------- " << endl;
			std::cout << " ============================================================= " << endl;
		#else
		#ifdef _ONLY_K_StereoMatching_
			std::cout << " ============================================================= " << endl;
			std::cout << " ----------------ONLY Compile K_StereoMatching---------------- " << endl;
			std::cout << " ============================================================= " << endl;
		#endif
		#ifdef _ONLY_K_FeatureExtraction_
			std::cout << " ============================================================= " << endl;
			std::cout << " ---------------ONLY Compile K_FeatureExtraction-------------- " << endl;
			std::cout << " ============================================================= " << endl;
		#endif
		#ifdef _ONLY_K_FeatureTracking_
			std::cout << " ============================================================= " << endl;
			std::cout << " ----------------ONLY Compile K_FeatureTracking--------------- " << endl;
			std::cout << " ============================================================= " << endl;
		#endif
		#ifdef _ONLY_K_MotionEstimation_
			std::cout << " ============================================================= " << endl;
			std::cout << " ---------------ONLY Compile K_MotionEstimation--------------- " << endl;
			std::cout << " ============================================================= " << endl;
		#endif
		#endif
			std::cout << endl;
			std::cout << " ============================================================= " << endl;
			std::cout << " -----------------Check Command Line Arguments---------------- " << endl;
			std::cout << " ============================================================= " << endl;
	#endif


	#ifdef _PURE_C_
		if (argc != 5){
			std::cout << "HOST-Error: Incorrect command line syntax " << endl << endl;
			return EXIT_FAILURE;
		}
	#else
		if (argc != 8){
			std::cout << "[HOST-Error]  Incorrect command line syntax " << endl << endl;
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
		std::cout << "[HOST-Info] argv[1] Input_Image_Path_1 : " << Input_Image_Path_1 << endl;
		std::cout << "[HOST-Info] argv[2] Input_Image_Path_2 : " << Input_Image_Path_2 << endl;
		std::cout << "[HOST-Info] argv[3] Calibration_Path   : " << Calibration_Path << endl;
		std::cout << "[HOST-Info] argv[4] Dataset_Frames_Num : " << Dataset_Frames_Num << endl;
	#ifndef _PURE_C_
		std::cout << "[HOST-Info] argv[5] Platform_Vendor    : " << Target_Platform_Vendor << endl;
		std::cout << "[HOST-Info] argv[6] Device_Name        : " << Target_Device_Name << endl;
		std::cout << "[HOST-Info] argv[7] XCLBIN_file        : " << xclbinFilename << endl << endl;
	#endif

	// Initial the paras and datastructures
		int FRAME_NUM = stoi(Dataset_Frames_Num);
		string OUTPUT_PATH(OUTPUT_FOLDER_PATH);

		cv::Mat K_Left, R_Left, T_Left;
		cv::Mat K_Right, R_Right, T_Right;
		cv::Mat P0, P1;
		cv::Mat ImgLeft_0, ImgRight_0, ImgLeft_1;
		cv::Mat Mask;
		int height, width;


		float* KP0 = new float[1000];
		float* KP1 = new float[1000];
		cv::Mat Des0(500, 32, CV_8U);
		cv::Mat Des1(500, 32, CV_8U);
		int* Matches = new int[1000];
		int Detected_Matches;

		cv::Mat rmat(3, 3, CV_32F);
		cv::Mat tvec(3, 1, CV_32F);



// Read calibration file
	BasicFunction::read_calibration((const char*)Calibration_Path.c_str(), &P0, &P1);
	#ifdef _INFO_
		std::cout << "[HOST-Info] Read calibration successfully " << endl;
	#endif
// Collect all images in the given directory
	vector<string> Img_Left_Set = BasicFunction::read_image_folder(FRAME_NUM, Input_Image_Path_1);
	vector<string> Img_Right_Set = BasicFunction::read_image_folder(FRAME_NUM, Input_Image_Path_2);
	#ifdef _INFO_
		std::cout << "[HOST-Info] Read image set successfully " << endl << endl ;
	#endif
// Read the first image
	cv::Mat Img = cv::imread(Img_Left_Set[0], cv::IMREAD_GRAYSCALE);

// Set parameters
	height = Img.rows;
	width = Img.cols;
	const size_t Img_Size = Img.rows*Img.cols*Img.channels();
	uchar* ImgLeft_0_BUF = new uchar[Img_Size];
	uchar* ImgRight_0_BUF = new uchar[Img_Size];
	uchar* ImgLeft_1_BUF = new uchar[Img_Size];
	
	cv::Mat Depth(height, width, CV_32F);

	Matrix Depth_Matrix(height, width);
	Mask = cv::Mat::zeros(height, width, CV_8U);
	Mask(cv::Rect(96, 0, width - 96, height)) = 255;

	cv::Mat T_tot = cv::Mat::eye(4, 4, CV_32F);
	vector<cv::Mat> trajectory(FRAME_NUM);
	cv::Rect rect(0, 0, 4, 3);
	T_tot(rect).copyTo(trajectory[0]);

// Decompose Projection Matrix
	cv::decomposeProjectionMatrix(P0, K_Left, R_Left, T_Left);
	cv::decomposeProjectionMatrix(P1, K_Right, R_Right, T_Right);
	T_Left = T_Left / (T_Left.at<float>(3));
	T_Right = T_Right / (T_Right.at<float>(3));

	#ifdef _INFO_
		std::cout << "[HOST-Info] Input image height: " << height << endl;
		std::cout << "[HOST-Info] Input image width: " << width << endl;
		std::cout << "[HOST-Info] Input image size: " << Img_Size * sizeof(uchar) << " Bytes" << endl;
	#endif

// Set up OpenCL
	#ifndef _PURE_C_
		#ifdef _INFO_
			std::cout << endl;
			std::cout << " ============================================================= " << endl;
			std::cout << " ----------------------Run OpenCL Section--------------------- " << endl;
			std::cout << " ============================================================= " << endl;
			std::cout << endl;
			#ifdef _ALL_KERNELS_
				std::cout << " ============================================================= " << endl;
				std::cout << " ----------------------Set Up All Kernels--------------------- " << endl;
				std::cout << " ============================================================= " << endl;
			#else
				#ifdef _ONLY_K_StereoMatching_
					std::cout << " ============================================================= " << endl;
					std::cout << " -------------------Set Up K_StereoMatching------------------- " << endl;
					std::cout << " ============================================================= " << endl;
				#endif
				#ifdef _ONLY_K_FeatureExtraction_
					std::cout << " ============================================================= " << endl;
					std::cout << " ------------------Set Up K_FeatureExtraction----------------- " << endl;
					std::cout << " ============================================================= " << endl;
				#endif
				#ifdef _ONLY_K_FeatureTracking_
					std::cout << " ============================================================= " << endl;
					std::cout << " -------------------Set Up K_FeatureTracking------------------ " << endl;
					std::cout << " ============================================================= " << endl;
				#endif
				#ifdef _ONLY_K_MotionEstimation_
					std::cout << " ============================================================= " << endl;
					std::cout << " ------------------Set Up K_MotionEstimation------------------ " << endl;
					std::cout << " ============================================================= " << endl;
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
		OCL_CHECK(errCode, std::string Device_Name = Device.getInfo<CL_DEVICE_NAME>(&errCode));

		std::cout << "[HOST-Info] Device found - " << Device_Name << endl;
		//std::cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(IN_TYPE, NPC1) << endl;
		//std::cout << "Input Image Channels:" << XF_CHANNELS(IN_TYPE, NPC1) << endl;
		//std::cout << "NPPC:" << NPC1 << endl;

		// Create Program and Kernel
		#ifdef _INFO_
			std::cout << endl;
			std::cout << " ============================================================= " << endl;
			std::cout << " ------------------Create Program and Kernel------------------ " << endl;
			std::cout << " ============================================================= " << endl;
		#endif

		// Load binary file to memory and create program
		//std::string xclbinFilename = BasicFunction::find_binary_file(Device_Name, "krnl");
		cl::Program::Binaries xcl_Binaries = BasicFunction::import_binary_file(xclbinFilename);
		Devices.clear();
		Devices.push_back(Device);
		Devices.resize(1);
		OCL_CHECK(errCode, cl::Program Program(Context, Devices, xcl_Binaries, NULL, &errCode));

		// Create one or more kernels
		#ifdef _ONLY_K_StereoMatching_
			OCL_CHECK(errCode, cl::Kernel K_StereoMatching(Program, K_StereoMatching_NAME, &errCode));
		#endif
		#ifdef _ONLY_K_FeatureExtraction_
			OCL_CHECK(errCode, cl::Kernel K_FeatureExtraction(Program, K_FeatureExtraction_NAME, &errCode));
		#endif
		#ifdef _ONLY_K_FeatureTracking_
			OCL_CHECK(errCode, cl::Kernel K_FeatureTracking(Program, "K_FeatureTracking_NAME", &errCode));
		#endif
		#ifdef _ONLY_K_MotionEstimation_
			OCL_CHECK(errCode, cl::Kernel K_MotionEstimation(Program, "K_MotionEstimation_NAME", &errCode));
		#endif

		// Create Buffer and Set Kernel Args
		#ifdef _INFO_
			std::cout << endl;
			std::cout << " ============================================================= " << endl;
			std::cout << " --------------Create Buffer and Set Kernel Args-------------- " << endl;
			std::cout << " ============================================================= " << endl;
		#endif
		#ifdef _ALL_KERNELS
			OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_0	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)ImgLeft_0_BUF,	&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_ImgRight_0(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)ImgRight_0_BUF,  &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_1	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)ImgLeft_1_BUF,	&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_Mask		(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)Mask.data,		&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_K_Left	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar) * 3 * 3,		(void*)K_Left.data,		&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_T_Left	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar) * 4 * 1,		(void*)T_Left.data,		&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_T_Right	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar) * 4 * 1,		(void*)T_Right.data,	&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_rmat		(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(uchar) * 3 * 3,		(void*)rmat.data,		&errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_tvec		(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(uchar) * 3 * 1,		(void*)tvec.data,		&errCode));

			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Depth				(Context, CL_MEM_READ_WRITE, sizeof(uchar)* height* width,	NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_KP0				(Context, CL_MEM_READ_WRITE, sizeof(float) * 2 * 500,		NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_KP1				(Context, CL_MEM_READ_WRITE, sizeof(float) * 2 * 500,		NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Des0				(Context, CL_MEM_READ_WRITE, sizeof(uchar) * 32 * 500,		NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Des1				(Context, CL_MEM_READ_WRITE, sizeof(uchar) * 32 * 500,		NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Detected_Matches	(Context, CL_MEM_READ_WRITE, sizeof(int),					NULL, &errCode));
			OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Matches			(Context, CL_MEM_READ_WRITE, sizeof(int) * 2 * 500,		    NULL, &errCode));

			OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(0, GlobMem_ImgLeft_0));
			OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(1, GlobMem_ImgRight_0));
			OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(2, GlobMem_K_Left));
			OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(3, GlobMem_T_Left));
			OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(4, GlobMem_T_Right));
			OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(5, GlobMem_BUF_Depth));

			// Since K_FeatureExtraction process one Img and generate exactly one KP and Des
			// we need to setArg of K_FeatureExtraction in a loop to select which Img
			/**
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(0, GlobMem_ImgLeft_0));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(1, GlobMem_ImgLeft_1));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(2, GlobMem_Mask));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(3, GlobMem_BUF_KP0));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(4, GlobMem_BUF_KP1));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(5, GlobMem_BUF_Des0));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(6, GlobMem_BUF_Des1));
			*/

			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(0, GlobMem_ImgLeft_0));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(1, GlobMem_ImgLeft_1));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(2, GlobMem_Mask));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(3, GlobMem_BUF_KP0));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(4, GlobMem_BUF_KP1));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(5, GlobMem_BUF_Des0));
			OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(6, GlobMem_BUF_Des1));

			OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(0, Filter));
			OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(1, GlobMem_BUF_Des0));
			OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(2, GlobMem_BUF_Des1));
			OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(4, GlobMem_BUF_Matches));
			OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(5, GlobMem_BUF_Detected_Matches));

			OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(0, GlobMem_K_Left));
			OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(1, GlobMem_BUF_Depth));
			OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(2, GlobMem_BUF_Matches));
			OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(3, GlobMem_BUF_Detected_Matches));
			OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(4, GlobMem_BUF_KP0));
			OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(5, GlobMem_BUF_KP1));
			OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(6, GlobMem_rmat));
			OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(7, GlobMem_tvec));


			OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(7, GlobMem_tvec));
		#else
			#ifdef _ONLY_K_StereoMatching_

				OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_0 (Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)ImgLeft_0_BUF,	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_ImgRight_0(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)ImgRight_0_BUF,	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_K_Left	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * 3 * 3,		(void*)K_Left.data,		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_T_Left	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * 4 * 1,		(void*)T_Left.data,		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_T_Right	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * 4 * 1,		(void*)T_Right.data,	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Depth		(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)Depth.data, 		&errCode));

				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(0, GlobMem_ImgLeft_0));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(1, GlobMem_ImgRight_0));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(2, GlobMem_K_Left));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(3, GlobMem_T_Left));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(4, GlobMem_T_Right));
				OCL_CHECK(errCode, errCode = K_StereoMatching.setArg(5, GlobMem_Depth));
			    

			#endif
			#ifdef _ONLY_K_FeatureExtraction_


				OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_0			(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)ImgLeft_0_BUF,	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_ImgLeft_1			(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)ImgRight_0_BUF, &errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Mask				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)Mask.data,		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_KP0				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * 2 * 500,		(void*)KP0,				&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_KP1				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(float) * 2 * 500,		(void*)KP1,				&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Des0				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(uchar) * 32 * 500,		(void*)Des0.data,		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Des1				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(uchar) * 32 * 500,		(void*)Des1.data,		&errCode));

				// Since K_FeatureExtraction process one Img and generate exactly one KP and Des
				// we need to setArg of K_FeatureExtraction in a loop to select which Img
				/**
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(0, GlobMem_ImgLeft_0));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(1, GlobMem_ImgLeft_1));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(2, GlobMem_Mask));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(3, GlobMem_KP0));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(4, GlobMem_KP1));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(5, GlobMem_Des0));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(6, GlobMem_Des1));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(7, GlobMem_Deheretected_Points));
				*/
			#endif
			#ifdef _ONLY_K_FeatureTracking_
				OCL_CHECK(errCode, cl::Buffer GlobMem_Des0				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar) * 32 * 500, (void*)Des0.data,	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Des1				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar) * 32 * 500, (void*)Des1.data, 	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Matches			(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(int) * 2 * 500,	(void*)Matches, 	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Detected_Matches	(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(int),				(void*)Detected_Matches, 	&errCode));

				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(0, Filter));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(1, GlobMem_Des0));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(2, GlobMem_Des1));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(4, GlobMem_Matches));
				OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(5, GlobMem_Detected_Matches));
			#endif
			#ifdef _ONLY_K_MotionEstimation_
				OCL_CHECK(errCode, cl::Buffer GlobMem_K_Left 			(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar) * 3 * 3,		(void*)K_Left.data, 	&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Depth				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(uchar)* height* width, (void*)Depth.data,		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Matches			(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(int) * 2 * 500,		(void*)Matches, 		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_Detected_Matches	(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(int),					(void*)Detected_Matches,&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_KP0				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * 2 * 500,		(void*)KP0, 			&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_KP1				(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * 2 * 500,		(void*)KP1, 			&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_rmat				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(uchar) * 3 * 3,		(void*)rmat.data, 		&errCode));
				OCL_CHECK(errCode, cl::Buffer GlobMem_tvec				(Context, CL_MEM_WRITE_ONLY| CL_MEM_USE_HOST_PTR, sizeof(uchar) * 3 * 1,		(void*)tvec.data, 		&errCode));

				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(0, GlobMem_K_Left));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(1, GlobMem_Depth));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(2, GlobMem_Matches));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(3, GlobMem_Detected_Matches));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(4, GlobMem_KP0));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(5, GlobMem_KP1));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(6, GlobMem_rmat));
				OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(7, GlobMem_tvec));
			#endif
		#endif
	#endif





	#ifdef _INFO_
	std::cout << endl;
	std::cout << " ============================================================= " << endl;
	std::cout << " -----------------------Run Application----------------------- " << endl;
	std::cout << " ============================================================= " << endl;
	#endif


	#ifdef _ALL_KERNELS

		int Num_Mem_Events = 20, Num_Exe_Events = 4;
		vector<cl::Event> EventList(Num_Exe_Events);
		#ifdef _INFO_
			std::cout << "[HOST-Info] Run All Kernels Using HLS..." << endl;
		#endif
			for (int i = 0; i < FRAME_NUM - 1; i++) {

				//ImgLeft_0 = ImgLeft_1;
				ImgLeft_0 = cv::imread(Img_Left_Set[i], cv::IMREAD_GRAYSCALE);
				ImgRight_0 = cv::imread(Img_Right_Set[i], cv::IMREAD_GRAYSCALE);
				ImgLeft_1 = cv::imread(Img_Left_Set[i + 1], cv::IMREAD_GRAYSCALE);
				memcpy(ImgLeft_0_BUF, ImgLeft_0.data, Img_Size);
				memcpy(ImgRight_0_BUF, ImgRight_0.data, Img_Size);
				memcpy(ImgLeft_1_BUF, ImgLeft_1.data, Img_Size);




		#ifdef _INFO_
				std::cout << endl << endl << endl << "=================== Iteration: " << i << " ==================== " << endl;
				std::cout << "ImgLeft_0: " << Img_Left_Set[i] << endl << endl;
		#endif

				// Submit Kernels for Execution

				// K_StereoMatching
				OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_K_Left, GlobMem_T_Left, GlobMem_T_Right, GlobMem_ImgLeft_0, GlobMem_ImgRight_0}, 0, NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_BUF_Depth}, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_StereoMatching, NULL, NULL));


				// K_FeatureExtraction
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(0, GlobMem_ImgLeft_0));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(1, GlobMem_Mask));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(2, GlobMem_BUF_KP0));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(3, GlobMem_BUF_Des0));
				OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_ImgLeft_0, GlobMem_Mask}, 0, NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_BUF_KP0, GlobMem_BUF_Des0}, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_FeatureExtraction, NULL, &EventList[0]));

				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(0, GlobMem_ImgLeft_1));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(1, GlobMem_Mask));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(2, GlobMem_BUF_KP1));
				OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(3, GlobMem_BUF_Des1));
				OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_ImgLeft_1, GlobMem_Mask}, 0, NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_BUF_KP1, GlobMem_BUF_Des1}, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_FeatureExtraction, &EventList, &EventList[1]));


				// K_FeatureTracking
				OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_BUF_Des0, GlobMem_BUF_Des1, GlobMem_BUF_Matches, GlobMem_BUF_Detected_Matches}, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_FeatureTracking, &EventList, NULL));


				// Waits until all previous enqueued commands have completed
				OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));


				// K_MotionEstimation
				OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_K_Left, GlobMem_rmat, GlobMem_tvec}, 0, NULL, NULL));
				OCL_CHECK(errCode, cl::Buffer GlobMem_BUF_Detected_Matches(Context, CL_MEM_READ_WRITE, sizeof(int), NULL, &errCode));
				OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_BUF_Depth, GlobMem_BUF_Matches, GlobMem_BUF_Detected_Matches, GlobMem_BUF_KP0, GlobMem_BUF_KP1}, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
				OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_MotionEstimation, NULL, NULL));

				Queue.finish();
			}

	#else

		for(int i=0; i<FRAME_NUM-1; i++){

			//ImgLeft_0 = ImgLeft_1;
			ImgLeft_0 = cv::imread(Img_Left_Set[i], cv::IMREAD_GRAYSCALE);
			ImgRight_0 = cv::imread(Img_Right_Set[i], cv::IMREAD_GRAYSCALE);
			ImgLeft_1 = cv::imread(Img_Left_Set[i + 1], cv::IMREAD_GRAYSCALE);
			memcpy(ImgLeft_0_BUF, ImgLeft_0.data, Img_Size);
			memcpy(ImgRight_0_BUF, ImgRight_0.data, Img_Size);
			memcpy(ImgLeft_1_BUF, ImgLeft_1.data, Img_Size);


			#ifdef _INFO_
				std::cout << endl << endl << endl << "=================== Iteration: " << i << " ==================== " << endl;
				std::cout << "[HOST-Info] ImgLeft_0: " << Img_Left_Set[i] << endl << endl;
			#endif

			#ifdef _ONLY_K_StereoMatching_
				#ifdef _INFO_
					std::cout << "[HOST-Info] Compute Stereo Matching Using HLS..." << endl;
				#endif
					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_K_Left, GlobMem_T_Left, GlobMem_T_Right, GlobMem_ImgLeft_0, GlobMem_ImgRight_0, GlobMem_Depth}, 0, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_StereoMatching, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_Depth}, CL_MIGRATE_MEM_OBJECT_HOST, NULL, NULL));
					Queue.finish();
			#else
				#ifdef _INFO_
					std::cout << "[HOST-Info] Compute Stereo Matching Using C..." << endl;
				#endif

				stereo_2_depth(ImgLeft_0.data, ImgRight_0.data, K_Left.data, T_Left.data, T_Right.data, true, false, true, Depth.data);

				#ifdef _PRINT_
					string Depth_name = OUTPUT_PATH + "C_StereoMatching/Depth_iteration" + std::to_string(i) + ".txt";
					BasicFunction::print_content(Depth_name, &Depth);
				#endif
			#endif




			#ifdef _ONLY_K_FeatureExtraction_
				#ifdef _INFO_
					std::cout << "[HOST-Info] Compute Feature Extraction Using HLS..." << endl;
				#endif

					OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(0, GlobMem_ImgLeft_0));
					OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(1, GlobMem_Mask));
					OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(2, GlobMem_KP0));
					OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(3, GlobMem_Des0));
					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_ImgLeft_1, GlobMem_Mask, GlobMem_KP0, GlobMem_Des0}, 0, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_FeatureExtraction, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_KP0, GlobMem_Des0}, CL_MIGRATE_MEM_OBJECT_HOST, NULL, NULL));

					OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(0, GlobMem_ImgLeft_1));
					OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(1, GlobMem_Mask));
					OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(2, GlobMem_KP1));
					OCL_CHECK(errCode, errCode = K_FeatureExtraction.setArg(3, GlobMem_Des1));
					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_ImgLeft_1, GlobMem_Mask, GlobMem_KP1, GlobMem_Des1}, 0, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_FeatureExtraction, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_KP1, GlobMem_Des1}, CL_MIGRATE_MEM_OBJECT_HOST, NULL, NULL));

					Queue.finish();
					#ifdef _PRINT_
					string KP0_name = OUTPUT_PATH + "K_FeatureExtraction/KP0_iteration" + std::to_string(i) + ".txt";
					string KP1_name = OUTPUT_PATH + "K_FeatureExtraction/KP1_iteration" + std::to_string(i) + ".txt";
					string Des0_name = OUTPUT_PATH + "K_FeatureExtraction/Des0_iteration" + std::to_string(i) + ".txt";
					string Dea1_name = OUTPUT_PATH + "K_FeatureExtraction/Des1_iteration" + std::to_string(i) + ".txt";
					BasicFunction::print_content(KP0_name, KP0, 2, 500);
					BasicFunction::print_content(KP1_name, KP1, 2, 500);
					BasicFunction::print_content(Des0_name, &Des0);
					BasicFunction::print_content(Dea1_name, &Des1);
					#endif

			#else

				#ifdef _INFO_
					std::cout << "[HOST-Info] Compute Feature Extraction Using C..." << endl;
				#endif

				extract_features(ImgLeft_0.data, Mask.data, KP0, Des0.data);
				extract_features(ImgLeft_1.data, Mask.data, KP1, Des1.data);

				#ifdef _PRINT_
					string KP0_name = OUTPUT_PATH + "C_FeatureExtraction/KP0_iteration" + std::to_string(i) + ".txt";
					string KP1_name = OUTPUT_PATH + "C_FeatureExtraction/KP1_iteration" + std::to_string(i) + ".txt";
					string Des0_name = OUTPUT_PATH + "C_FeatureExtraction/Des0_iteration" + std::to_string(i) + ".txt";
					string Dea1_name = OUTPUT_PATH + "C_FeatureExtraction/Des1_iteration" + std::to_string(i) + ".txt";
					BasicFunction::print_content(KP0_name, KP0, 2, 500);
					BasicFunction::print_content(KP1_name, KP1, 2, 500);
					BasicFunction::print_content(Des0_name, &Des0);
					BasicFunction::print_content(Dea1_name, &Des1);
				#endif

			#endif




			#ifdef _ONLY_K_FeatureTracking_

					OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(0, Filter));
					OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(1, GlobMem_Des0));
					OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(2, GlobMem_Des1));
					OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(4, GlobMem_Matches));
					OCL_CHECK(errCode, errCode = K_FeatureTracking.setArg(5, GlobMem_Detected_Matches));

					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_Des0, GlobMem_Des1, GlobMem_Matches, GlobMem_Detected_Matches}, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_FeatureTracking, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_Matches, GlobMem_Detected_Matches}, CL_MIGRATE_MEM_OBJECT_CONTENT_UNDEFINED, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));

					Queue.finish();


			#else
				#ifdef _INFO_
					std::cout << "[HOST-Info] Compute Feature Tracking Using C..." << endl;
				#endif

				match_feature(Des0.data, Des1.data, Filter, (int32*)Matches);

				#ifdef _PRINT_
					string Matches_name = OUTPUT_PATH + "C_FeatureTracking/Matches_iteration" + std::to_string(i) + ".txt";
					BasicFunction::print_content(Matches_name, Matches, 2, 500);
				#endif
			#endif




			#ifdef _ONLY_K_MotionEstimation_

					OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(0, GlobMem_K_Left));
					OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(1, GlobMem_Depth));
					OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(2, GlobMem_Matches));
					OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(3, GlobMem_Detected_Matches));
					OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(4, GlobMem_KP0));
					OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(5, GlobMem_KP1));
					OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(6, GlobMem_rmat));
					OCL_CHECK(errCode, errCode = K_MotionEstimation.setArg(7, GlobMem_tvec));

					// K_MotionEstimation
					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_K_Left, GlobMem_Depth, GlobMem_Matches, GlobMem_Detected_Matches, GlobMem_KP0, GlobMem_KP1, GlobMem_rmat, GlobMem_tvec}, 0, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueMigrateMemObjects(cl::vector<cl::Memory>{GlobMem_rmat, GlobMem_tvec}, 0, NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueBarrierWithWaitList(NULL, NULL));
					OCL_CHECK(errCode, errCode = Queue.enqueueTask(K_MotionEstimation, NULL, NULL));

					Queue.finish();


			#else
				#ifdef _INFO_
					std::cout << "[HOST-Info] Compute Motion Estimation Using C..." << endl;
				#endif

				EstimateMotion motion = EstimateMotion();

				// constructor: Matrix(row, col)
				// assign value: Matrix.val[row][col] = val;
				Matrix Matches_Matrix(500, 2);
				Matrix KP0_Matrix(500, 2);
				Matrix KP1_Matrix(500, 2);
				Matrix K_Left_Matrix(3, 3);
				Matrix rmat_Matrix(3, 3);
				Matrix tvec_Matrix(3, 1);


				for (int r = 0; r < 500; r++) {
					for (int c = 0; c < 2; c++) {
						Matches_Matrix.val[r][c] = Matches[r * 2 + c];
						KP0_Matrix.val[r][c] = KP0[r * 2 + c];
						KP1_Matrix.val[r][c] = KP1[r * 2 + c];
					}
				}
				for (int r = 0; r < 3; r++) {
					for (int c = 0; c < 3; c++) {
						K_Left_Matrix.val[r][c] = K_Left.at<float>(r, c);
					}
				}
				for (int r = 0; r < height; r++) {
					for (int c = 0; c < width; c++) {
						Depth_Matrix.val[r][c] = Depth.at<float>(r, c);
					}
				}

				motion.estimate(Matches_Matrix, KP0_Matrix, KP1_Matrix, K_Left_Matrix, Depth_Matrix, rmat_Matrix, tvec_Matrix);

				for (int r = 0; r < 3; r++) {
					for (int c = 0; c < 3; c++) {
						rmat.at<float>(r, c) = rmat_Matrix.val[r][c];
					}
					tvec.at<float>(r) = tvec_Matrix.val[r][0];
				}


				#ifdef _PRINT_
					string rmat_name = OUTPUT_PATH + "C_MotionEstimation/rmat_iteration" + std::to_string(i) + ".txt";
					string tvec_name = OUTPUT_PATH + "C_MotionEstimation/tvec_iteration" + std::to_string(i) + ".txt";
					BasicFunction::print_content(rmat_name, &rmat);
					BasicFunction::print_content(tvec_name, &tvec);
				#endif

			#endif

			cv::Mat T_mat;
			cv::Mat I4 = cv::Mat::eye(4, 4, CV_32F);
			cv::hconcat(rmat, tvec, T_mat);
			cv::vconcat(T_mat, I4.row(3), T_mat);
			T_tot = T_tot * T_mat.inv();
			T_tot(rect).copyTo(trajectory[i+1]);



		}
	#endif

	ImgLeft_0.deallocate();
	ImgRight_0.deallocate();
	ImgLeft_1.deallocate();
	Depth.deallocate();
	Des0.deallocate();
	Des1.deallocate();
	rmat.deallocate();
	tvec.deallocate();
	delete[]KP0;
	delete[]KP1;
	delete[]Matches;
	/*
	ofstream fout;
	fout.open("Output/trajectory.txt", ios::out);
	for (int idx = 0; idx < FRAME_NUM - 1; idx++) {
		fout << trajectory.at(idx).at<float>(0, 3) << "\t"
			<< trajectory.at(idx).at<float>(1, 3) << "\t"
			<< trajectory.at(idx).at<float>(2, 3) << endl;
	}
	fout.close();
	*/











}

