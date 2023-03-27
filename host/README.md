# Robotics Computing Host Code

This host code is written in C/C++ and utilizes OpenCV and OpenCL C++ Wrapper. It is designed to run visual odometry and communicate with Xilinx hardware accelerator kernels.

</br>

## Block Diagram and Interfaces

![image](https://user-images.githubusercontent.com/85032763/227933821-b6e32927-c918-4c42-a64b-2f38c047cbb5.png)

Please refer to the [Host Interface](https://hackmd.io/jeLJ6ZYyQmuwtZx6spOqRA) for more information.
</br></br>

## Setup Environment

To set up the Vitis Vision Library Build (for U50), please follow this [guide](https://hackmd.io/3VZbNXG4T2CFOSbVs57WFQ).

</br></br>

## Setup Vitis

1. Create an empty application project using `xilinx_u50` as the platform.
2. Import the host code to the `src` folder under `<ProjectName>[x86]` and kernel code to the `src` folder under `<ProjectName>_kernels` in the Explorer panel.
3. Add kernel functions in the `<ProjectName>_kernels.prj` project editor.
4. Right-click `<ProjectName>.prj` and select **Properties**.
   Add the following includes and libraries in the **Properties**:

   * **C/C++ Build > Settings > Tool Setting > GCC Host Compiler(x86_64) > Includes**
     Add the path ``<path-to-opencv-include-folder>`` to **Include paths(-l)**.
   * **C/C++ Build > Settings > Tool Setting > GCC Host Linker(x86_64) > Libraries**

     Add the path ``<path-to-opencv-lib-folder>`` to **Library search path(-L)**.

     Add the following libraries:
     
     ``opencv_videoio`` ``opencv_imgcodecs`` ``opencv_core`` ``opencv_imgproc`` ``opencv_features2d`` ``opencv_flann`` ``opencv_video`` ``opencv_calib3d`` ``opencv_highgui``
     
     to **Libraries(-l)**.
5. Click the **Run** button (green button at the top panel) to add configurations

   * Double-click **System Project Debug** and click the generated **SystemDebugger**
   * Edit **Program Arguments** and provide arguments as below:
   
     ```
     <path-to-leftimage-folder> <path-to-rightimage-folder> <path-to-calib-file> <number-of-frames> <platform-vendor> <device-name>
     
     e.g. 
     /home/chngh/Desktop/Host/dataset/sequences/00/image_0/ /home/chngh/Desktop/Host/dataset/sequences/00/image_1/ /home/chngh/Desktop/Host/dataset/sequences/00/calib.txt 12 Xilinx xilinx_u50_gen3x16_xdma_5_202210_1
     ```
     
     Enable **Automatically add binary container(s) to arguments**
   * Edit **Configuration** and enable **OpenCL trace**

   **Note:** If you run the pure C code, please delete ``<platform-vendor>`` ``<device-name>`` arguments, and disable **Automatically add binary container(s) to aruments**
   </br></br>

## Setup Host Program Behavier

You can modify the behavior of the host program in `Parameters.hpp`. There are 3 sections that you can modify:

1. Include Section

   In this section, you need to ensure that the header file of the pure C code is correct.
2. Macro Section

   * You can complie the program in pure C code, only one kernel in HLS code, or all kernels in HLS code using 6 provided macros.
   * You can print the program's progress on terminal with ``_INFO_``
   * You can write the output of the function to a .txt file with ``_PRINT_``
3. Parameter Section

   In this section, you need to verify that the name of the main function in your kernel is correct. Additionally, you are required to specify the path to the output folder on your computer while following the directory structure outlined below:

   ```
    ${ProjectFolder}
      |- ${HostFolder}
      |   |- src
      |   |   |- C_FeatureExtraction  
      |   |   |- C_FeatureTracking
      |   |   |- C_MotionEstimation
      |   |   |- C_StereoMatching
      |   |   |- K_FeatureExtraction
      |   |   |- K_FeatureTracking
      |   |   |- K_MotionEstimation
      |   \   \- K_StereoMatching
      |  
      |- output  
      |   |- C_FeatureExtraction  
      |   |- C_FeatureTracking
      |   |- C_MotionEstimation
      |   |- C_StereoMatching
      |   |- K_FeatureExtraction
      |   |- K_FeatureTracking
      |   |- K_MotionEstimation
      |   \- K_StereoMatching
      |- dataset
      |   |- poses
      \   \- sequences 
   ```