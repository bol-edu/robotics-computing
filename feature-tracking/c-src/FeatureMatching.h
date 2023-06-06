
typedef unsigned char uchar;

#define MAX_KEYPOINT_NUM 500
#define DESCRIPTOR_COL 32
#define K 2

struct DMatch
{
	int queryIndex;
	int trainIndex;
	int distance;
};

const int tab[] =
	{
		0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
		1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
		1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
		2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
		1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
		2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
		2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
		3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8};

void match_feature(uchar *des0, uchar *des1, float dist_threshold, int *match, int match_num[1]);

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
