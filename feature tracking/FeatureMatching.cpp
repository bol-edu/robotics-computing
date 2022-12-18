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

    int dtype = CV_32F;

    __batchDistance(queryDescriptors, trainDescriptors, dist, dtype, nidx,
        NORM_L2, knn);

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
        __batchDistL2_((float*)src1.ptr(i), (float*)src2.ptr(), src2.step, src2.rows, src2.cols,
            (float*)bufptr);

        int* nidxptr = nidx.ptr<int>(i);
        // since positive float's can be compared just like int's,
        // we handle both CV_32S and CV_32F cases with a single branch
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


void FeatureMatching::__batchDistL2_(const float* src1, const float* src2, size_t step2,
    int nvecs, int len, float* dist)
{
    step2 /= sizeof(src2[0]);

    for (int i = 0; i < nvecs; i++)
        dist[i] = sqrt(__normL2Sqr(src1, src2 + step2 * i, len));
}



float FeatureMatching::__normL2Sqr(const float* a, const float* b, int n)
{
    float s = 0;
    int i = 0;
    
    for (; i < n; i++)
    {
        float v = float(a[i] - b[i]);
        s += v * v;
    }
    return s;
}