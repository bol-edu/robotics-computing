
#include <vector>
//#include "Mat.h"
#include "Rect.h"
#include "Point__.h"
#include "KeyPoint.h"
using namespace std;

class KeyPointsFilter__
{
public:
    KeyPointsFilter__() {}

    /*
     * Remove keypoints within borderPixels of an image edge.
     */
    static void runByImageBorder(std::vector<KeyPoint>& keypoints, Size imageSize, int borderSize);
    /*
     * Remove keypoints of sizes out of range.
     */
    //static void runByKeypointSize(std::vector<KeyPoint>& keypoints, float minSize,
     //   float maxSize = FLT_MAX);
    /*
     * Remove keypoints from some image by mask for pixels of this image.
     */
    static void runByPixelsMask(std::vector<KeyPoint>& keypoints, const Mat& mask);
    /*
     * Remove duplicated keypoints.
     */
    //static void removeDuplicated(std::vector<KeyPoint>& keypoints);
    /*
     * Remove duplicated keypoints and sort the remaining keypoints
     */
    static void removeDuplicatedSorted(std::vector<KeyPoint>& keypoints);

    /*
     * Retain the specified number of the best keypoints (according to the response)
     */
    static void retainBest(std::vector<KeyPoint>& keypoints, int npoints);
};

struct KeyPoint12_LessThan
{
    bool operator()(const KeyPoint& kp1, const KeyPoint& kp2) const
    {
        if (kp1.pt.x != kp2.pt.x)
            return kp1.pt.x < kp2.pt.x;
        if (kp1.pt.y != kp2.pt.y)
            return kp1.pt.y < kp2.pt.y;
        if (kp1.size != kp2.size)
            return kp1.size > kp2.size;
        if (kp1.angle != kp2.angle)
            return kp1.angle < kp2.angle;
        if (kp1.response != kp2.response)
            return kp1.response > kp2.response;
        if (kp1.octave != kp2.octave)
            return kp1.octave > kp2.octave;
        return kp1.class_id > kp2.class_id;
    }
};


void KeyPointsFilter__::removeDuplicatedSorted(std::vector<KeyPoint>& keypoints)
{
    int i, j, n = (int)keypoints.size();

    if (n < 2) return;

    std::sort(keypoints.begin(), keypoints.end(), KeyPoint12_LessThan());

    for (i = 0, j = 1; j < n; ++j)
    {
        const KeyPoint& kp1 = keypoints[i];
        const KeyPoint& kp2 = keypoints[j];
        if (kp1.pt.x != kp2.pt.x || kp1.pt.y != kp2.pt.y ||
            kp1.size != kp2.size || kp1.angle != kp2.angle) {
            keypoints[++i] = keypoints[j];
        }
    }
    keypoints.resize(i + 1);
}

struct KeypointResponseGreaterThanThreshold
{
    KeypointResponseGreaterThanThreshold(float _value) :
        value(_value)
    {
    }
    inline bool operator()(const KeyPoint& kpt) const
    {
        return kpt.response >= value;
    }
    float value;
};

struct KeypointResponseGreater
{
    inline bool operator()(const KeyPoint& kp1, const KeyPoint& kp2) const
    {
        return kp1.response > kp2.response;
    }
};



void KeyPointsFilter__::retainBest(std::vector<KeyPoint>& keypoints, int n_points)
{
    //this is only necessary if the keypoints size is greater than the number of desired points.
    if (n_points >= 0 && keypoints.size() > (size_t)n_points)
    {
        if (n_points == 0)
        {
            keypoints.clear();
            return;
        }
        //first use nth element to partition the keypoints into the best and worst.
        std::nth_element(keypoints.begin(), keypoints.begin() + n_points - 1, keypoints.end(), KeypointResponseGreater());
        //this is the boundary response, and in the case of FAST may be ambiguous
        float ambiguous_response = keypoints[n_points - 1].response;
        //use std::partition to grab all of the keypoints with the boundary response.
        std::vector<KeyPoint>::const_iterator new_end =
            std::partition(keypoints.begin() + n_points, keypoints.end(),
                KeypointResponseGreaterThanThreshold(ambiguous_response));
        //resize the keypoints, given this new end point. nth_element and partition reordered the points inplace
        keypoints.resize(new_end - keypoints.begin());
    }
}





class MaskPredicate
{
public:
    MaskPredicate(const Mat& _mask) : mask(_mask) {}
    bool operator() (const KeyPoint& key_pt) const
    {
        return mask.at<uchar>((int)(key_pt.pt.y + 0.5f), (int)(key_pt.pt.x + 0.5f)) == 0;
    }

private:
    const Mat mask;
    MaskPredicate& operator=(const MaskPredicate&);
};

void KeyPointsFilter__::runByPixelsMask(std::vector<KeyPoint>& keypoints, const Mat& mask)
{
    //CV_INSTRUMENT_REGION();

    if (mask.empty())
        return;

    keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(), MaskPredicate(mask)), keypoints.end());
}




struct RoiPredicate
{
    RoiPredicate(const Rect& _r) : r(_r)
    {}

    bool operator()(const KeyPoint& keyPt) const
    {
        //return !r.contains(keyPt.pt);
        return!(r.x <= keyPt.pt.x && keyPt.pt.x < r.x + r.width && r.y <= keyPt.pt.y && keyPt.pt.y < r.y + r.height);
    }

    Rect r;
};

void KeyPointsFilter__::runByImageBorder(std::vector<KeyPoint>& keypoints, Size imageSize, int borderSize)
{
    if (borderSize > 0)
    {
        if (imageSize.height <= borderSize * 2 || imageSize.width <= borderSize * 2)
            keypoints.clear();
        else
            keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
                RoiPredicate(Rect(Point__(borderSize, borderSize),
                    Point__(imageSize.width - borderSize, imageSize.height - borderSize)))),
                keypoints.end());
    }
}
