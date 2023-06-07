# Motion Estimation
Estimating the relative motion between camera poses using depth map and matched keypoints. This kernel is using **RANSAC-EPnP** (RANdom SAmple Consensus, Efficient Perspective-n-Point) and **Iterative PnP** (Direct Linear Transform & Levenberg–Marquardt).

## What's in Here
```
.
├── testdata      // testing data
├── host          // estimate_motion testbench
├── kernel        // estimate_motion kernel code
├── Makefile      // build host.exe and estimate_motion.xclbin
├── utils.mk      // build setting file
├── args.mk       // execute arguements setting file
```

## How to Build
1. ```cd``` to directory [program](./program/).
```
cd ./motion-estimation/program
```
2. ```make``` the host executable program & xclbin files using Makefile.
```
make all TARGET=hw
```
3. As long as the host program & kernel are compiled, you may test the functionality of the kernel, using [test data](./program/testdata/). \
Run the program using
```
make run TARGET=hw
```
4. (option) You may modify the arguements by [args.mk](./program/args.mk)

## How to Use
### Kernel Interface

|  #  | Arguments  |  Type  | Size (number of items) | input/ouput  |
| :-: | :--------: | :----: | :--------------------: | :----------: |
|  0  |  matches   | MATCH  |    MAX_KEYPOINT_NUM    |    input     |
|  1  | match_num  |  int   |           1            |    input     |
|  2  |    kp0     | IPOINT |    MAX_KEYPOINT_NUM    |    input     |
|  3  |    kp1     | IPOINT |    MAX_KEYPOINT_NUM    |    input     |
|  4  |     fx     | float  |           1            |    input     |
|  5  |     fy     | float  |           1            |    input     |
|  6  |     cx     | float  |           1            |    input     |
|  7  |     cy     | float  |           1            |    input     |
|  8  |   depth    | float  | IMG_WIDTH * IMG_HEIGHT |    input     |
|  9  | threshold  |  int   |           1            |    input     |
| 10  | confidence | float  |           1            |    input     |
| 11  |  maxiter   |  int   |           1            |    input     |
| 12  |    rmat    | float  |           9            | input/output |
| 13  |    tvec    | float  |           3            | input/output |
| 14  | take_last  |  bool  |           1            |    input     |

### Arguements Description
* matches   \
    An array of matched indices of kp0 and kp1 from ***Feature Matching***. The maximum buffer size is MAX_KEYPOINT_NUM.

* match_num \
    Number of matched keypoint sets.

* kp0, kp1 \
    Keypoint 0, Keypoint 1 from ***Feature Extraction***.

* fx, fy, cx, cy \
    Focal length, optical center from ***intrinsic matrix***. \
    ```fx = K_left[0][0], fy = K_left[1][1], cx = K_left[0][2], cy = K_left[1][2]```.

* depth \
    Depth map from ***Stereo Matching***.

* threshold \
    Parameter for RANSAC. Distance (in pixel) to determine whether the projected 2D point is outlier.

* confidence \
    Parameter for RANSAC. To determine whether the number of inlier is sufficient.

* maxiter \
    Parameter for RANSAC. The maximum number of iteration to operate RANSAC.

* rmat, tvec \
    Outcome rotation matrix and translation vector.

* take_last \
    To determine whether rmat and tvec are taken as inputs to act as initial values of gradient descent.


### Type / Marco Description
* MAX_KEYPOINT_NUM
    ``` cpp
    #define MAX_KEYPOINT_NUM 500
    ```
    
* MATCH
    ``` cpp
    struct MATCH
    {
        int a;      // index of kp0
        int b;      // index of kp1
    };
    ```
* IPOINT
    ``` cpp
    struct IPOINT
    {
        float x;    // x coordinate of 2D point
        float y;    // y coordinate of 2D point
    };
    ```


## Design Flow
- [Algorithms](#algorithms)    
- [Pure C/C++ code](./c-src)
- [HLS code](./hls-src/)
- [Reference](#reference)

## Algorithms

<p align="center">
  <img src="./img/Motion%20Estimation%20block%20diagram.png" width="50%" />
</p>

The brief introduction can be found in the [slide](./doc/motion_estimation.pdf). You may also check out the [video](https://youtu.be/1_DqewUjm7Q).

- ### What we have
  ***depth map*** from Stereo Matching

  matched 2D ***keypoints*** of 1st and 2nd left images

  ***camera intrinsic matrix*** 

  Now we are looking for Rotation matrix ***rmat*** and translation vector ***tvec*** to estimate the relative motion between 1st and 2nd images. 

  ```cpp
  void estimate(Matrix &match, Matrix &kp0, Matrix &kp1, Matrix &k, Matrix &depth, Matrix &rmat, Matrix &tvec);
  ```

- ### Projection
  Before doing the math part, we need to modify the input to match the requirement. Because we're using PnP, which solves relative motion cv problem by 2D-3D points. As a result, we project 1st left image's keypoints to 3D point through depth map and camera matrix. Also we align 2 sets of keypoints through matched index.

<p align="center">
  <img src="./img/EPnP%20block%20diagram.png" />
</p>

## Pure C/C++ code
Extracting and modifying source code from OpenCV, the pure C/C++ code implement most of the function [solvePnPRansac](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html). It's runnable for CPU, but may not be able to synthesis due to dynamic allocated memory and some other reasons.

## HLS code
- [underbaseline](./hls-src/c-synth_underbaseline) \
The codes are synthesizable are and able to do co-simulation, while no optimization is done. In fact, these codes are outcome when taking co-simulation as priority. The utilization may be too high to implement on the board. 

## Reference
- [PnP求解--EPnP | EpsilonJohn's Blog](http://epsilonjohn.club/2020/02/26/SLAM%E4%BB%A3%E7%A0%81%E8%AF%BE%E7%A8%8B/PnP%E6%B1%82%E8%A7%A3-EPnP/#epnp)
- [深入EPnP算法_JesseChen79的博客-CSDN博客_epnp算法](https://blog.csdn.net/jessecw79/article/details/82945918)
- [PnP 算法简介 & 代码解析 - 柴政](https://youtu.be/EX8Y9kB1sSw)
- [Newton's method for solving nonlinear systems of Algebraic equations](https://youtu.be/zPDp_ewoyhM)

- [Singular Value Decomposition (SVD) - 2022.2 English - Xilinx](https://docs.xilinx.com/r/en-US/Vitis_Libraries/quantitative_finance/guide_L1/SVD/SVD.html)
