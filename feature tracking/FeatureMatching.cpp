#include "FeatureMatching.h"

using namespace cv;

vector<DMatch> FeatureMatching::match_features(Mat des1, Mat des2, int detector, bool sorting, float dist_threshold)
{
	vector<vector<DMatch>> matches;
	clock_t start = clock();

	__knnMatch(des1, des2, matches, 2);

	clock_t end = clock();

	printf("\tTime to match keypoints using BF: %lld ms\n\n", end - start);

	vector<DMatch> filtered_match;
	for (int m = 0; m < matches.size(); m++)
	{

		if (matches[m][0].distance <= dist_threshold * matches[m][1].distance)
		{
			filtered_match.push_back(matches[m][0]);
		}
	}
	return filtered_match;
}

void FeatureMatching::__knnMatch(InputArray _queryDescriptors, InputArray _trainDescriptors,
    vector<vector<DMatch>>& matches, int knn)
{

    const int IMGIDX_SHIFT = 18;
    const int IMGIDX_ONE = (1 << IMGIDX_SHIFT);

    Mat queryDescriptors = _queryDescriptors.getMat();
    Mat trainDescriptors = _trainDescriptors.getMat();

    matches.reserve(queryDescriptors.rows);

    Mat dist, nidx;

    int dtype = CV_32S;

    __batchDistance(queryDescriptors, trainDescriptors, dist, dtype, nidx,
        NORM_HAMMING2, knn);

    Mat temp;
    dist.convertTo(temp, CV_32F);
    dist = temp;

    for (int qIdx = 0; qIdx < queryDescriptors.rows; qIdx++)
    {
        const float* distptr = dist.ptr<float>(qIdx);
        const int* nidxptr = nidx.ptr<int>(qIdx);

        matches.push_back(vector<DMatch>());
        vector<DMatch>& mq = matches.back();
        mq.reserve(knn);

        for (int k = 0; k < nidx.cols; k++)
        {
            if (nidxptr[k] < 0)
                break;
            mq.push_back(DMatch(qIdx, nidxptr[k] & (IMGIDX_ONE - 1),
                nidxptr[k] >> IMGIDX_SHIFT, distptr[k]));
        }

    }
    cout << "================== " << matches[0][0].distance << endl;
    cout << "================== " << matches[1][0].distance << endl;
    cout << "================== " << matches[2][0].distance << endl;
}


void FeatureMatching::__batchDistance(InputArray _src1, InputArray _src2,
    OutputArray _dist, int dtype, OutputArray _nidx,
    int normType, int K)
{

    Mat src1 = _src1.getMat(), src2 = _src2.getMat();
    int type = src1.type();


    K = min(K, src2.rows);

    _dist.create(src1.rows, (K > 0 ? K : src2.rows), dtype);
    Mat dist = _dist.getMat(), nidx;

    _nidx.create(dist.size(), CV_32S);
    nidx = _nidx.getMat();

    dist = Scalar::all(dtype == CV_32S ? (double)INT_MAX : (double)FLT_MAX);
    nidx = Scalar::all(-1);

    AutoBuffer<int> buf(src2.rows);
    int* bufptr = buf.data();

    for (int i = 0; i < src1.rows; i++)
    {
        __batchDistHamming2_((uchar*)src1.ptr(i), (uchar*)src2.ptr(), 
            src2.step, src2.rows, src2.cols, (int*)bufptr);


        int* nidxptr = nidx.ptr<int>(i);
        int* distptr = (int*)dist.ptr(i);

        int j, k;

        for (j = 0; j < src2.rows; j++)
        {
            int d = bufptr[j];
            if (d < distptr[K - 1])
            {
                for (k = K - 2; k >= 0 && distptr[k] > d; k--)
                {
                    nidxptr[k + 1] = nidxptr[k];
                    distptr[k + 1] = distptr[k];
                }
                nidxptr[k + 1] = j;
                distptr[k + 1] = d;
            }
        }
    }
}


void FeatureMatching::__batchDistHamming2_(const uchar* src1, const uchar* src2, size_t step2,
    int nvecs, int len, int* dist)
{
    const uchar tab[256] =
    {
        0, 1, 1, 1, 1, 2, 2, 2, 1, 2, 2, 2, 1, 2, 2, 2, 1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3,
        1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3,
        1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
        2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
        1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
        2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
        1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
        2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4
    };

    step2 /= sizeof(src2[0]);

    for (int i = 0; i < nvecs; i++) 
    {
        for (int j = 0; j < len; j++)   // len = 32
            dist[i] += tab[src1[j] ^ src2[j]];
    }
}