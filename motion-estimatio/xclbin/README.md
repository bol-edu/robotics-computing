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