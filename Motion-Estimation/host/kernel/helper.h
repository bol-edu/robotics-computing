#pragma once
#define MAX_KEYPOINT_NUM 500
#define IMAGE_HEIGTH 376
#define IMAGE_WIDTH 1241

typedef float FLOAT; // FLOAT precision

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
