#pragma once
#define MAX_KEYPOINT_NUM 500
#define IMAGE_HEIGTH 376
#define IMAGE_WIDTH 1241

#define DESCRIPTOR_COL 32
#define DES0_SIZE MAX_KEYPOINT_NUM *DESCRIPTOR_COL
#define DES1_SIZE MAX_KEYPOINT_NUM *DESCRIPTOR_COL

typedef float FLOAT;
typedef unsigned char uchar;

struct OPOINT
{
	FLOAT x;
	FLOAT y;
	FLOAT z;
};

struct IPOINT
{
	FLOAT x;
	FLOAT y;
};

struct MATCH
{
	int a;
	int b;
};