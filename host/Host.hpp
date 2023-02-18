//#include "hls_stream.h"
//#include "ap_int.h"
//#include "common/xf_common.hpp"
//#include "common/xf_utility.hpp"
//#include "imgproc/xf_sgbm.hpp"
//#include "xf_config_params.h"

//=================================Macro Section=================================
/**
 * 6 compile modes:
 * `_PURE_C_`
 * `_ONLY_K_StereoMatching_`
 * `_ONLY_K_FeatureExtraction_` 
 * `_ONLY_K_FeatureTracking_`
 * `_ONLY_K_MotionEstimation_`
 * `_ALL_KERNELS_`
 * 
 * 1 message modes
 * `_INFO_`
 * 
 * The default compile mode is `_ALL_KERNELS_` + `_INFO_`
 */

#define _PURE_C_
#define _INFO_


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
        exit(EXIT_FAILURE);                                                                               	\
    }
//===============================Macro Section END===============================


//===============================Parameter Section===============================
#define BASE			1024

#define Filter 1

// Set the input and output pixel depth:
#define IN_TYPE XF_8UC1
#define PTR_IN_WIDTH 8
#define OUT_TYPE XF_8UC1
#define PTR_OUT_WIDTH 8

// Set the optimization type:
#define NPC1 XF_NPPC1

// Size of these buffers are in byte
#define SIZE_ImgLeft_0              BASE * /*datadepth*/  376*1241
#define SIZE_ImgRight_0             BASE * /*datadepth*/  376*1241  
#define SIZE_ImgLeft_1              BASE * /*datadepth*/  376*1241
#define SIZE_Mask                   BASE * /*datadepth*/  376*1241

#define SIZE_K_Left                 BASE * /*datadepth*/
#define SIZE_T_Left                 BASE * /*datadepth*/
#define SIZE_T_Right                BASE * /*datadepth*/

#define SIZE_Filter                 BASE * /*datadepth*/


#define SIZE_BUF_rmat               BASE * /*datadepth*/
#define SIZE_BUF_tvec               BASE * /*datadepth*/

#define SIZE_BUF_Depth              BASE * /*datadepth*/

#define SIZE_BUF_KP0                BASE * /*datadepth*/
#define SIZE_BUF_KP1                BASE * /*datadepth*/
#define SIZE_BUF_Des0               BASE * /*datadepth*/
#define SIZE_BUF_Des1               BASE * /*datadepth*/
#define SIZE_BUF_Detected_Points    BASE * /*datadepth*/

#define SIZE_BUF_Matches            BASE * /*datadepth*/
#define SIZE_BUF_Detected_Matches   BASE * /*datadepth*/

#define SIZE_T_Mat                  BASE * /*datadepth*/

#define K                           // input arg of K_FeatureTracking
#define MaxDepth                    // input arg of K_MotionEstimation
//=============================Parameter Section END=============================

//===============================Datatype Section================================

//=============================Datatype Section END==============================












