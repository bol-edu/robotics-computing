
# include <stdio.h>
# include <iostream>
# include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <algorithm>

using namespace std;

#define BM 0
#define SGBM 1

#define Sift 0
#define Orb 1
#define Surf 2

#define BF 1
#define FLANN 0

extern "C" {
    void top_function(unsigned char* image_left_test_data, unsigned char* image_right_test_data, unsigned char* k_left_test_data, unsigned char* t_left_test_data, unsigned char* t_right_test_data, unsigned char* depth_map);
}

