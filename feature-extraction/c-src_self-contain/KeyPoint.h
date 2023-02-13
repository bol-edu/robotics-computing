#pragma once
#include "Point__.h"

//using namespace std;

class  KeyPoint
{
public:
    //! the default constructor
     KeyPoint();
    /**
    @param _pt x & y coordinates of the keypoint
    @param _size keypoint diameter
    @param _angle keypoint orientation
    @param _response keypoint detector response on the keypoint (that is, strength of the keypoint)
    @param _octave pyramid octave in which the keypoint has been detected
    @param _class_id object id
     */
    KeyPoint(Point2f_ _pt, float _size, float _angle = -1, float _response = 0, int _octave = 0, int _class_id = -1);
    /**
    @param x x-coordinate of the keypoint
    @param y y-coordinate of the keypoint
    @param _size keypoint diameter
    @param _angle keypoint orientation
    @param _response keypoint detector response on the keypoint (that is, strength of the keypoint)
    @param _octave pyramid octave in which the keypoint has been detected
    @param _class_id object id
     */
     KeyPoint(float x, float y, float _size, float _angle = -1, float _response = 0, int _octave = 0, int _class_id = -1);

    size_t hash() const;

    /**
    This method converts vector of keypoints to vector of points or the reverse, where each keypoint is
    assigned the same size and the same orientation.

    @param keypoints Keypoints obtained from any feature detection algorithm like SIFT/SURF/ORB
    @param points2f Array of (x,y) coordinates of each keypoint
    @param keypointIndexes Array of indexes of keypoints to be converted to points. (Acts like a mask to
    convert only specified keypoints)
    */
     static void convert(const std::vector<KeyPoint>& keypoints,
         std::vector<Point2f_>& points2f,
        const std::vector<int>& keypointIndexes = std::vector<int>());
    /** @overload
    @param points2f Array of (x,y) coordinates of each keypoint
    @param keypoints Keypoints obtained from any feature detection algorithm like SIFT/SURF/ORB
    @param size keypoint diameter
    @param response keypoint detector response on the keypoint (that is, strength of the keypoint)
    @param octave pyramid octave in which the keypoint has been detected
    @param class_id object id
    */
     static void convert(const std::vector<Point2f_>& points2f,
         std::vector<KeyPoint>& keypoints,
        float size = 1, float response = 1, int octave = 0, int class_id = -1);

    /**
    This method computes overlap for pair of keypoints. Overlap is the ratio between area of keypoint
    regions' intersection and area of keypoint regions' union (considering keypoint region as circle).
    If they don't overlap, we get zero. If they coincide at same location with same size, we get 1.
    @param kp1 First keypoint
    @param kp2 Second keypoint
    */
     static float overlap(const KeyPoint& kp1, const KeyPoint& kp2);

     Point2f_ pt; //!< coordinates of the keypoints
     float size; //!< diameter of the meaningful keypoint neighborhood
     float angle; //!< computed orientation of the keypoint (-1 if not applicable);
    //!< it's in [0,360) degrees and measured relative to
    //!< image coordinate system, ie in clockwise.
     float response; //!< the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
     int octave; //!< octave (pyramid layer) from which the keypoint has been extracted
     int class_id; //!< object class (if the keypoints need to be clustered by an object they belong to)
};


inline
KeyPoint::KeyPoint()
    : pt(0, 0), size(0), angle(-1), response(0), octave(0), class_id(-1) {}

inline
KeyPoint::KeyPoint(Point2f_ _pt, float _size, float _angle, float _response, int _octave, int _class_id)
    : pt(_pt), size(_size), angle(_angle), response(_response), octave(_octave), class_id(_class_id) {}

inline
KeyPoint::KeyPoint(float x, float y, float _size, float _angle, float _response, int _octave, int _class_id)
    : pt(x, y), size(_size), angle(_angle), response(_response), octave(_octave), class_id(_class_id) {}