

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>
//#include "define.h"
//#include "Mat.h"
//#include "KeyPoint.h"
//#include "resize1.h"
//#include "threshold__.h"
//#include "copyMakeBorder__.h"
//#include "GaussianBlur__.h"
//#include "KeyPointsFilter__.h"
//#include "AutoBuffer__.h"
//#include "RNG__.h"
//#include "BufferArea__.h"
//#include "FAST__.h"
//#include "Point__.h"








using namespace std;
//using namespace cv;


#define BM 0
#define SGBM 1

#define Sift 0
#define Orb 1
#define Surf 2

#define BF 1
#define FLANN 0



void extract_features(unsigned char* image_data, unsigned char* mask_data, float* kp_xy, unsigned char* des_data);
