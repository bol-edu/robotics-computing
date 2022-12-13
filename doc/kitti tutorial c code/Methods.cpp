#include "Methods.h"
using namespace cv;

void Methods::read_calib(const char *filePath, cv::Mat *P0, cv::Mat *P1)
/*******************************************************************************
    Read the calibration file from 'filePath', and return projection matices
    of camera0 and camera1 as P0 and P1.

    Arguments:
        filePath -- file path of the calibration file
        P0 -- pointer to projection matrix of camera0
        P1 -- pointer to projection matrix of camera1

*******************************************************************************/
{
    FILE *fp;
    fopen_s(&fp, filePath, "r");
    char *next_token1 = NULL;
    char *next_token2 = NULL;

    *P0 = cv::Mat(3, 4, CV_32F);
    *P1 = cv::Mat(3, 4, CV_32F);

    if (!fp)
    {
        printf("Could not open the calibration file\n");
    }

    int count = 0;
    bool p;
    char content[1024];
    while (fgets(content, 1024, fp))
    {
        char *v = strtok_s(content, " ,", &next_token1);
        while (v)
        {
            if (--count > 0)
            {
                istringstream os(v);
                float d;
                os >> d;
                if (p)
                    P1->at<float>((12 - count) / 4, (12 - count) % 4) = d;
                else
                    P0->at<float>((12 - count) / 4, (12 - count) % 4) = d;
            }
            if (!strcmp(v, "P0:"))
            {
                count = 13;
                p = 0;
            }
            else if (!strcmp(v, "P1:"))
            {
                count = 13;
                p = 1;
            }
            v = strtok_s(NULL, " ,", &next_token1);
        }
    }

    fclose(fp);
}

vector<Mat> Methods::groundTruthTrajectory(const char *filePath, int data_num)
/*******************************************************************************
    Read the ground truth poses data from 'filePath', return Matrices of
    ground truth poses in a vector.

    Arguments:
        filePath -- file path of the poses data

    Return:
        poses -- a vector of poses in the form of matrix

*******************************************************************************/
{
    vector<Mat> poses;
    FILE *fp;
    fopen_s(&fp, filePath, "r");
    int cols = 12;
    for (int i = 0; i < data_num; i++)
    {
        Mat mat_i = Mat(3, 4, CV_32F);
        for (int j = 0; j < cols; j++)
        {
            fscanf_s(fp, "%e", &mat_i.at<float>(j / 4, j % 4));
        }
        poses.push_back(mat_i);
    }

    fclose(fp);
    return poses;
}

Mat Methods::computeLeftDisparityMap(Mat img_left, Mat img_right, int matcher_name, bool rgb)
/***************************************************************************************
    Takes a left and right stereo pair of images and computes the disparity
    map for the left image. Pass rgb = true if the images are RGB.

    Arguments:
        img_left -- image from left camera
        img_right -- image from right camera

    Optional Arguments:
        matcher -- (bool) can be 'BM' for StereoBM or 'SGBM' for StereoSGBM matching
        rgb -- (bool) set to true if passing RGB images as input

    Returns:
        disp_left -- disparity map for the left camera image

***************************************************************************************/
{
    // Feel free to read OpenCV documentationand tweak these values.These work well
    int sad_window = 6;
    int num_disparities = sad_window * 16;
    int block_size = 11;

    Ptr<StereoMatcher> matcher;
    Mat disp_left;

    if (matcher_name == BM)
    {
        matcher = StereoBM::create(num_disparities, block_size);
    }
    else if (matcher_name == SGBM)
    {
        matcher = StereoSGBM::create(0, num_disparities, block_size, 8 * 3 * pow(sad_window, 2), 32 * 3 * pow(sad_window, 2), 0, 0, 0, 0, 0, StereoSGBM::MODE_SGBM_3WAY);
    }

    if (rgb)
    {
        cvtColor(img_left, img_left, COLOR_BGR2GRAY);
        cvtColor(img_right, img_right, COLOR_BGR2GRAY);
    }

    printf("\n\tComputing disparity map using Stereo%s...\n", (matcher_name == BM) ? "BM" : "SGBM");
    clock_t start = clock();
    if (matcher_name == BM)
    {
        matcher->compute(img_left, img_right, disp_left);
        disp_left.convertTo(disp_left, CV_32F, 1.0 / 16);
    }
    else if (matcher_name == SGBM)
    {
        matcher->compute(img_left, img_right, disp_left);
        disp_left.convertTo(disp_left, CV_32F, 1.0 / 16);
    }

    clock_t end = clock();
    printf("\tTime to compute disparity map using Stereo%s: %lld ms\n", (matcher_name == BM) ? "BM" : "SGBM", end - start);

    int x = 300, y = 1200;

    return disp_left;
}

void Methods::decompose_Projection_Matrix(Mat p, Mat *k, Mat *r, Mat *t)
/***************************************************************************************
    Shortcut to use cv::decomposeProjectionMatrix(), which only returns k, r, t, and
    divides t by the scale, then returns them through pointers

    Arguments:
    p -- projection matrix to be decomposed

    Returns (call by address):
    k, r, t -- intrinsic matrix, rotation matrix, and 3D translation vector

***************************************************************************************/
{
    decomposeProjectionMatrix(p, *k, *r, *t);

    *t = *t / (t->at<float>(3));
}

Mat Methods::calc_depth_map(Mat disp_left, Mat k_left, Mat t_left, Mat t_right, bool rectified)
/***************************************************************************************
    Calculate depth map using a disparity map, intrinsic camera matrix, and translation
    vectors from camera extrinsic matrices(to calculate baseline).
    Note that default behavior is for rectified projection matrix for right camera.
    If using a regular projection matrix, pass rectified = false to avoid issues.

    Arguments:
        disp_left -- disparity map of left camera
        k_left -- intrinsic matrix for left camera
        t_left -- translation vector for left camera
        t_right -- translation vector for right camera
        rectified-- (bool)set to False if t_right is not from rectified projection
                    matrix

    Returns :
        depth_map -- calculated depth map for left camera

***************************************************************************************/
{
    Mat depth_map = Mat::ones(disp_left.rows, disp_left.cols, CV_32F);
    disp_left.convertTo(disp_left, CV_32F);

    // Get focal length of x axis for left camera
    float f = k_left.at<float>(0, 0);

    // Calculate baseline of stereo pair
    float b;
    if (rectified)
        b = t_right.at<float>(0, 0) - t_left.at<float>(0, 0);
    else
        b = t_left.at<float>(0, 0) - t_right.at<float>(0, 0);

    for (int i = 0; i < disp_left.rows; i++)
    {
        for (int j = 0; j < disp_left.cols; j++)
        {
            // Avoid instability and division by zero
            if (disp_left.at<float>(i, j) == 0.0 ||
                disp_left.at<float>(i, j) == -1.0)
                disp_left.at<float>(i, j) = 0.1;

            // Make empty depth map then fill with depth
            depth_map.at<float>(i, j) = f * b / disp_left.at<float>(i, j);
        }
    }

    return depth_map;
}

Mat Methods::stereo_2_depth(Mat img_left, Mat img_right, Mat P0, Mat P1, bool matcher, bool rgb, bool rectified)
/***************************************************************************************
    Takes stereo pair of images and returns a depth map for the left camera.If your
    projection matrices are not rectified, set rectified = false.

    Arguments:
        img_left -- image of left camera
        img_right -- image of right camera
        P0 -- Projection matrix for the left camera
        P1 -- Projection matrix for the right camera

    Optional Arguments :
        matcher-- (str)can be 'bm' for StereoBM or 'sgbm' for StereoSGBM
        rgb-- (bool)set to True if images passed are RGB.Default is False
        rectified-- (bool)set to False if P1 not rectified to P0.Default is True

    Returns :
        depth -- depth map for left camera

***************************************************************************************/
{
    // Compute disparity map
    Mat disp = computeLeftDisparityMap(img_left,
                                       img_right,
                                       matcher,
                                       rgb);

    // Decompose projection matrices
    Mat k_left, r_left, t_left;
    Mat k_right, r_right, t_right;
    decompose_Projection_Matrix(P0, &k_left, &r_left, &t_left);
    decompose_Projection_Matrix(P1, &k_right, &r_right, &t_right);

    // Calculate depth map for left camera
    Mat depth = calc_depth_map(disp, k_left, t_left, t_right, true);

    return depth;
}

Mat Methods::extract_features(Mat image, int detector, Mat mask, vector<KeyPoint> *kp)
/***************************************************************************************
    Find keypoints and descriptors for the image

    Arguments :
        image -- a grayscale image
        detector-- (bool)can be 'Sift' or 'Orb'
        mask -- (Mat) mask to reduce feature search area to where depth information
                available.

    Returns :
        kp (call by address) -- list of the extracted keypoints(features) in an image
        des -- list of the keypoint descriptors in an image

***************************************************************************************/
{

    Ptr<Feature2D> det;
    Mat des;

    if (detector == Sift)
    {
        det = SIFT::create();
        det->Feature2D::detect(image, *kp, mask);
        det->Feature2D::compute(image, *kp, des);
    }
    else if (detector == Orb)
    {
        det = ORB::create();
        det->Feature2D::detect(image, *kp, mask);
        det->Feature2D::compute(image, *kp, des);
    }

    return des;
}

vector<DMatch> Methods::match_features(Mat des1, Mat des2, bool matching, int detector, bool sorting, int k, float dist_threshold)
/***************************************************************************************
    Match features from two images

    Arguments :
        des1 -- list of the keypoint descriptors in the first image
        des2 -- list of the keypoint descriptors in the second image
        matching-- (bool)can be 'BF' for Brute Force or 'FLANN'
        detector-- (int)can be 'Sift or 'Orb'
        sort-- (bool)whether to sort matches by distance.Default is True
        k-- (int)number of neighbors to match to each feature.
        dist_threshold -- maximum allowed relative distance between the best
                        matches, (0.0, 1.0)

    Returns:
        matches -- list of matched features from two images.Each match[i] is k or less
        matches for the same query descriptor
***************************************************************************************/
{
    vector<vector<DMatch>> matches;
    clock_t start = clock();

    if (matching == BF)
    {
        BFMatcher matcher;
        if (detector == Sift)
            matcher.BFMatcher::create(NORM_L2, false);
        else if (detector == Orb)
            matcher.BFMatcher::create(NORM_HAMMING2, false);
        matcher.BFMatcher::knnMatch(des1, des2, matches, k);
    }
    else if (matching == FLANN)
    {

        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        matcher->knnMatch(des1, des2, matches, 2);
    }
    clock_t end = clock();

    vector<DMatch> filtered_match;
    if (dist_threshold)
    {
        for (int m = 0; m < matches.size(); m++)
        {

            if (matches[m][0].distance <= dist_threshold * matches[m][1].distance)
            {
                filtered_match.push_back(matches[m][0]);
            }
        }
    }
    else
    {
        for (int j = 0; j < matches.size(); j++)
            filtered_match.push_back(matches[j][0]);
    }

    printf("\tTime to match keypoints using %s: %lld ms\n\n", (matching == BF) ? "BF" : "FLANN", end - start);

    return filtered_match;
}

void Methods::visualize_matches(Mat image1, vector<KeyPoint> kp1, Mat image2, vector<KeyPoint> kp2, vector<DMatch> match)
/***************************************************************************************
    Visualize corresponding matches in two images.
    Filter matched features from two images by distance between the best matches

    Arguments :
        image1 -- the first image in a matched image pair
        kp1 -- list of the keypoints in the first image
        image2 -- the second image in a matched image pair
        kp2 -- list of the keypoints in the second image
        match -- list of matched features from the pair of images

    Returns :
        image_matches -- an image showing the corresponding matches on both image1 and
        image2 or None if you don't use this function

***************************************************************************************/
{
    Mat image_matches;
    drawMatches(image1, kp1, image2, kp2, match, image_matches);
    imshow("image matches", image_matches);
    waitKey();
    destroyWindow("image matches");
    system("cls");
}

void Methods::estimate_motion(vector<DMatch> match, vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat k, Mat depth1, int max_depth,
                              Mat &rmat, Mat &tvec, Mat &image1_points, Mat &image2_points)
/***************************************************************************************
    Estimate camera motion from a pair of subsequent image frames

    Arguments :
        match -- list of matched features from the pair of images
        kp1 -- list of the keypoints in the first image
        kp2 -- list of the keypoints in the second image
        k -- camera intrinsic calibration matrix
        depth1 -- Depth map of the first frame.Set to None to use Essential Matrix
                decomposition
        max_depth -- Threshold of depth to ignore matched features. 3000 is default

    Returns (call by reference) :
        rmat -- estimated 3x3 rotation matrix
        tvec -- estimated 3x1 translation vector
        image1_points -- matched feature pixel coordinates in the first image.
        image1_points[i] = [u, v]->pixel coordinates of i - th match
        image2_points -- matched feature pixel coordinates in the second image.
        image2_points[i] = [u, v]->pixel coordinates of i - th match
***************************************************************************************/
{
    Mat image1_points__ = Mat(0, 2, CV_32F);
    image1_points = Mat(0, 2, CV_32F);
    Mat image2_points__ = Mat(0, 2, CV_32F);
    image2_points = Mat(0, 2, CV_32F);
    Mat rvec;
    Mat distCoef = Mat::zeros(1, 5, CV_32F);

    for (int m = 0; m < match.size(); m++)
    {
        image1_points__.push_back(kp1[match[m].queryIdx].pt);
        image2_points__.push_back(kp2[match[m].trainIdx].pt);
    }

    if (!depth1.empty())
    {
        float cx = k.at<float>(0, 2);
        float cy = k.at<float>(1, 2);
        float fx = k.at<float>(0, 0);
        float fy = k.at<float>(1, 1);
        Mat object_points = Mat::zeros(0, 3, CV_32F);

        for (int i = 0; i < image1_points__.rows; i++)
        {

            float u = image1_points__.at<float>(i, 0);
            float v = image1_points__.at<float>(i, 1);
            float z = depth1.at<float>((int)v, (int)u);

            if (z > max_depth)
            {
                continue;
            }

            float x = z * (u - cx) / fx;
            float y = z * (v - cy) / fy;

            Mat vec = Mat(1, 3, CV_32F);
            vec.at<float>(0, 0) = x;
            vec.at<float>(0, 1) = y;
            vec.at<float>(0, 2) = z;

            object_points.push_back(vec);
            image1_points.push_back(image1_points__.row(i));
            image2_points.push_back(image2_points__.row(i));
        }
        cv::solvePnPRansac(object_points, image2_points, k, distCoef, rvec, tvec,
                           false, 100, 8.0, 0.99, noArray(), SOLVEPNP_ITERATIVE);

        rmat = Mat::eye(3, 3, CV_32F);
        Rodrigues(rvec, rmat);
    }
}

vector<Mat> Methods::visual_odometry(Dataset_Handler handler, int detector, bool matching,
                                     float filter_match_distance, bool stereo_matcher, int subset, Mat mask)
/***************************************************************************************
    Function to perform visual odometry on a sequence from the KITTI visual odometry
    dataset.
    Takes as input a Dataset_Handler object and optional parameters.

    Arguments:
        handler -- Dataset_Handler object instance
        detector -- (str) can be 'Sift' or 'Orb'.
        matching -- (str) can be 'BF' for Brute Force or 'FLANN'.
        filter_match_distance -- (float) value for ratio test on matched features.
                                Default is None.
        stereo_matcher -- (str) can be 'BM' (faster) or 'SGBM' (more accurate).
        mask -- (array) mask to reduce feature search area to where depth information
                    available.
        subset -- (int) number of frames to compute. Defaults to None to compute
                        all frames.

    Returns:
        trajectory -- Array of shape Nx3x4 of estimated poses of vehicle for each
                    computed frame.
***************************************************************************************/
{
    int num_frames;

    printf("Generating disparities with Stereo %s\n", stereo_matcher ? "SGBM" : "BM");
    printf("Detecting features with %s and matching with %s\n", (detector == Sift) ? "SIFT" : (detector == Orb) ? "ORB"
                                                                                                                : "SURF",
           matching ? "BF" : "FLANN");

    if (filter_match_distance)
        printf("Filtering feature matches at threshold of %f * distance\n", filter_match_distance);

    if (subset)
        num_frames = subset;
    else
        num_frames = handler.num_frames;

    Mat T_tot = Mat::eye(4, 4, CV_64F);

    vector<Mat> trajectory(num_frames);
    Rect rect(0, 0, 4, 3);
    T_tot(rect).copyTo(trajectory[0]);

    int imwidth = handler.imwidth;
    int imheight = handler.imheight;

    Mat k_left, r_left, t_left;
    decompose_Projection_Matrix(handler.P0, &k_left, &r_left, &t_left);

    Mat image_plus1 = imread(handler.left_image_files[0], IMREAD_GRAYSCALE);
    Mat image_left, image_right;
    Mat depth;

    for (int i = 0; i < num_frames - 1; i++)
    {
        printf("Computing frame %d\n", i + 1);
        clock_t start = clock();
        image_left = image_plus1;
        image_right = imread(handler.right_image_files[i], IMREAD_GRAYSCALE);
        image_plus1 = imread(handler.left_image_files[i + 1], IMREAD_GRAYSCALE);

        depth = stereo_2_depth(image_left, image_right, handler.P0, handler.P1, stereo_matcher, false, true);

        vector<KeyPoint> kp0, kp1;
        Mat des0 = extract_features(image_left, detector, mask, &kp0);
        Mat des1 = extract_features(image_plus1, detector, mask, &kp1);

        vector<DMatch> matches = match_features(des0, des1, matching, detector, false, 2, filter_match_distance);

        Mat rmat, tvec, img1_points, img2_points;
        estimate_motion(matches, kp0, kp1, k_left, depth, 3000, rmat, tvec, img1_points, img2_points);

        Mat T_mat;
        Mat I4 = Mat::eye(4, 4, CV_64F);
        hconcat(rmat, tvec, T_mat);
        vconcat(T_mat, I4.row(3), T_mat);
        T_tot = T_tot * T_mat.inv();
        T_tot(rect).copyTo(trajectory[i + 1]);
        clock_t end = clock();

        printf("Time to compute frame %d: %lld ms\n\n", i + 1, end - start);
    }

    return trajectory;
}
