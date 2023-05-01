#include "kernel.h"

#define num_frames 98

int loadFile2Memory(const char *filename, char **result);

void read_data(MATCH *match, int &match_num,
			   IPOINT *kp0, IPOINT *kp1,
			   FLOAT &fx, FLOAT &fy, FLOAT &cx, FLOAT &cy, FLOAT *depth, int idx);
