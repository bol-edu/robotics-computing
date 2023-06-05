//=================================DO NOT MODIFY=================================
#define CL_HPP_CL_1_2_DEFAULT_BUILD
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_ENABLE_PROGRAM_CONSTRUCTION_FROM_ARRAY_COMPATIBILITY 1
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS


#include <CL/cl2.hpp>
#include <opencv2/opencv.hpp>
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


#ifdef _ALL_KERNELS_
#define _ONLY_K_StereoMatching_
#define _ONLY_K_FeatureExtraction_
#define _ONLY_K_FeatureTracking_
#define _ONLY_K_MotionEstimation_
#endif

#define OCL_CHECK(error, call)  																			\
	call;                                                                                           		\
    if (error != CL_SUCCESS) {                                                                            	\
        fprintf(stderr, "%s:%d Error calling " #call ", error code is: %d\n", __FILE__, __LINE__, error); 	\
        fprintf(stderr, "See the error code reference:\n");													\
		fprintf(stderr, "https://www.docdroid.net/Ee5RTkS/opencl-error-codes-1x-and-2x-pdf#page=6\n");		\
		fprintf(stderr, "https://registry.khronos.org/OpenCL/sdk/1.0/docs/man/xhtml/errors.html\n");		\
        exit(EXIT_FAILURE);                                                                               	\
    }
//=================================DO NOT MODIFY=================================


