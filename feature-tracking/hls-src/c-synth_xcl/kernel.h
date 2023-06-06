typedef unsigned char uchar;
typedef float FLOAT;

struct MATCH
{
    int a;
    int b;
};

#define MAX_KEYPOINT_NUM 500
#define DESCRIPTOR_COL 32
#define DES0_SIZE MAX_KEYPOINT_NUM *DESCRIPTOR_COL
#define DES1_SIZE MAX_KEYPOINT_NUM *DESCRIPTOR_COL
