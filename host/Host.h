

#define BASE			1024

// Size of these buffers are in byte
#define SIZE_ImgLeft_0              BASE * /*datadepth*/
#define SIZE_ImgRight_0             BASE * /*datadepth*/
#define SIZE_ImgLeft_1              BASE * /*datadepth*/
#define SIZE_Mask                   BASE * /*datadepth*/

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


typedef /*datatype*/ Img_Type;
typedef /*datatype*/ Mask_Type;
typedef /*datatype*/ Mat_P_Type;
typedef /*datatype*/ Filter_Type;
typedef /*datatype*/ Depth_Type;
typedef /*datatype*/ KP_Type;
typedef /*datatype*/ Des_Type;
typedef /*datatype*/ Detected_Points_Type;
typedef /*datatype*/ Matches_Type;
typedef /*datatype*/ Detected_Matches_Type;
typedef /*datatype*/ rmat_Type;
typedef /*datatype*/ tvec_Type;
typedef /*datatype*/ T_Mat_Type;
