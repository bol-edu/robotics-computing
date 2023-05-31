#include "helper.h"

#define num_frames 1

int loadFile2Memory(const char *filename, char **result);

void read_data(MATCH *match, int &match_num,
			   IPOINT *kp0, IPOINT *kp1,
			   FLOAT &fx, FLOAT &fy, FLOAT &cx, FLOAT &cy, 
         FLOAT *depth, char* data_dir, int idx);

void display_store_output(FLOAT rmat[9], FLOAT tvec[3], int idx, char *output_dir);
