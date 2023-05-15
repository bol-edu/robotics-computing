#include "helper.h"

#define num_frames 495

int loadFile2Memory(const char *filename, char **result);

void read_data(MATCH* match, int &match_num,
			   IPOINT* kp0, IPOINT* kp1,
			   FLOAT &fx, FLOAT &fy, FLOAT &cx, FLOAT &cy, FLOAT* depth, int idx);

void display_output(FLOAT rmat[9], FLOAT tvec[3]);
