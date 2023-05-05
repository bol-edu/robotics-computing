#include "kernel.h"

#define K 2

struct DMatch
{
	int queryIndex;
	int trainIndex;
	int distance;
};

static const uchar popCountTable2[] =
	{
		0, 1, 1, 1, 1, 2, 2, 2, 1, 2, 2, 2, 1, 2, 2, 2, 1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3,
		1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3,
		1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4};

extern "C"
{
void match_feature(uchar *des0, uchar *des1, FLOAT dist_threshold, MATCH *match, int* match_num);
}

void knnMatch(uchar queryDescriptors[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
			  uchar trainDescriptors[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
			  DMatch matches[MAX_KEYPOINT_NUM][2]);

void batchDistance(uchar src1[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
				   uchar src2[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
				   int dist[MAX_KEYPOINT_NUM][2],
				   int nidx[MAX_KEYPOINT_NUM][2]);

void batchDistHamming2(const uchar src1[DESCRIPTOR_COL],
					   const uchar src2[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
					   int dist[MAX_KEYPOINT_NUM]);

int normHamming(const uchar a[DESCRIPTOR_COL], const uchar b[DESCRIPTOR_COL]);
