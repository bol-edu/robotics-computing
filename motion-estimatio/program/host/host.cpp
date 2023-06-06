/*******************************************************************************
** HOST Code
*******************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <iomanip>

using namespace std;

#include <CL/cl.h>

#include "host.h"

#define ALL_MESSAGES

// ********************************************************************************** //
// ---------------------------------------------------------------------------------- //
//                          M A I N    F U N C T I O N                                //
// ---------------------------------------------------------------------------------- //
// ********************************************************************************** //

int main(int argc, char *argv[])
{
    cout << endl;

// ============================================================================
// Step 1: Check Command Line Arguments
// ============================================================================
//    o) argv[1] Platfrom Vendor
//    o) argv[2] Device Name
//    o) argv[3] XCLBIN file
//    o) argv[4] Input data directory
//    o) argv[5] Output data directory
//    o) argv[6] RANSAC threshold value
//    o) argv[7] RANSAC confidence value
//    o) argv[8] RANSAC maxiter value
// ============================================================================
#ifdef ALL_MESSAGES
    cout << "HOST-Info: ============================================================= " << endl;
    cout << "HOST-Info: (Step 1) Check Command Line Arguments                      " << endl;
    cout << "HOST-Info: ============================================================= " << endl;
#endif

    if (argc != 9)
    {
        cout << "HOST-Error: Incorrect command line syntax " << endl;
        cout << "HOST-Info: " << endl
             << " Usage: " << argv[0] << " <Platform_Vendor> <Device_Name> <XCLBIN_File> <Data_Folder> <Threshold> <Confidence> <Max_iteration>" << endl
             << endl;
        return EXIT_FAILURE;
    }

    const char *Target_Platform_Vendor = argv[1];
    const char *Target_Device_Name = argv[2];
    const char *xclbinFilename = argv[3];
    char *input_data_dir = argv[4];
    char *output_data_dir = argv[5];

    cout << "HOST-Info: Platform_Vendor        : " << Target_Platform_Vendor << endl;
    cout << "HOST-Info: Device_Name            : " << Target_Device_Name << endl;
    cout << "HOST-Info: XCLBIN_file            : " << xclbinFilename << endl;
    cout << "HOST-Info: Input data directory   : " << input_data_dir << endl;
    cout << "HOST-Info: Output data directory  : " << output_data_dir << endl;

    // ============================================================================
    // Step 2: Detect Target Platform and Target Device in a system.
    //         Create Context and Command Queue.
    // ============================================================================
    // Variables:
    //   o) Target_Platform_Vendor[] - defined as main() input argument
    //   o) Target_Device_Name[]     - defined as main() input argument
    //
    // After that
    //   o) Create a Context
    //   o) Create a Command Queue
    // ============================================================================
    cout << endl;
#ifdef ALL_MESSAGES
    cout << "HOST-Info: ============================================================= " << endl;
    cout << "HOST-Info: (Step 2) Detect Target Platform and Target Device in a system " << endl;
    cout << "HOST-Info:          Create Context and Command Queue                     " << endl;
    cout << "HOST-Info: ============================================================= " << endl;
#endif

    cl_uint ui;

    cl_platform_id *Platform_IDs;
    cl_uint Nb_Of_Platforms;
    cl_platform_id Target_Platform_ID;
    bool Platform_Detected;
    char *platform_info;

    cl_device_id *Device_IDs;
    cl_uint Nb_Of_Devices;
    cl_device_id Target_Device_ID;
    bool Device_Detected;
    char *device_info;

    cl_context Context;
    cl_command_queue Command_Queue;

    cl_int errCode;
    size_t size;

    // ------------------------------------------------------------------------------------
    // Step 2.1: Get All PLATFORMS, then search for Target_Platform_Vendor (CL_PLATFORM_VENDOR)
    // ------------------------------------------------------------------------------------

    // Get the number of platforms
    // ..................................................
    errCode = clGetPlatformIDs(0, NULL, &Nb_Of_Platforms);
    if (errCode != CL_SUCCESS || Nb_Of_Platforms <= 0)
    {
        cout << endl
             << "HOST-Error: Failed to get the number of available platforms" << endl
             << endl;
        return EXIT_FAILURE;
    }

#ifdef ALL_MESSAGES
    cout << "HOST-Info: Number of detected platforms : " << Nb_Of_Platforms << endl;
#endif

    // Allocate memory to store platforms
    // ..................................................
    Platform_IDs = new cl_platform_id[Nb_Of_Platforms];
    if (!Platform_IDs)
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for Platform_IDs" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Get and store all PLATFORMS
    // ..................................................
    errCode = clGetPlatformIDs(Nb_Of_Platforms, Platform_IDs, NULL);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "HOST-Error: Failed to get the available platforms" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Search for Platform (ex: Xilinx) using: CL_PLATFORM_VENDOR = Target_Platform_Vendor
    // ....................................................................................
    Platform_Detected = false;
    for (ui = 0; ui < Nb_Of_Platforms; ui++)
    {

        errCode = clGetPlatformInfo(Platform_IDs[ui], CL_PLATFORM_VENDOR, 0, NULL, &size);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "HOST-Error: Failed to get the size of the Platofrm parameter "
                 << "CL_PLATFORM_VENDOR"
                 << " value " << endl
                 << endl;
            return EXIT_FAILURE;
        }

        platform_info = new char[size];
        if (!platform_info)
        {
            cout << endl
                 << "HOST-Error: Out of Memory during memory allocation for Platform Parameter "
                 << "CL_PLATFORM_VENDOR" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        errCode = clGetPlatformInfo(Platform_IDs[ui], CL_PLATFORM_VENDOR, size, platform_info, NULL);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "HOST-Error: Failed to get the "
                 << "CL_PLATFORM_VENDOR"
                 << " platform info" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        // Check if the current platform matches Target_Platform_Vendor
        // .............................................................
        if (strcmp(platform_info, Target_Platform_Vendor) == 0)
        {
            Platform_Detected = true;
            Target_Platform_ID = Platform_IDs[ui];
#ifdef ALL_MESSAGES
            cout << "HOST-Info: Selected platform            : " << Target_Platform_Vendor << endl
                 << endl;
#endif
        }
    }

    if (Platform_Detected == false)
    {
        cout << endl
             << "HOST-Error: Failed to get detect " << Target_Platform_Vendor << " platform" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // ------------------------------------------------------------------------------------
    // Step 2.2:  Get All Devices for selected platform Target_Platform_ID
    //            then search for Xilinx platform (CL_DEVICE_NAME = Target_Device_Name)
    // ------------------------------------------------------------------------------------

    // Get the Number of Devices
    // ............................................................................
    errCode = clGetDeviceIDs(Target_Platform_ID, CL_DEVICE_TYPE_ALL, 0, NULL, &Nb_Of_Devices);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "HOST-Error: Failed to get the number of available Devices" << endl
             << endl;
        return EXIT_FAILURE;
    }
#ifdef ALL_MESSAGES
    cout << "HOST-Info: Number of available devices  : " << Nb_Of_Devices << endl;
#endif

    Device_IDs = new cl_device_id[Nb_Of_Devices];
    if (!Device_IDs)
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for Device_IDs" << endl
             << endl;
        return EXIT_FAILURE;
    }

    errCode = clGetDeviceIDs(Target_Platform_ID, CL_DEVICE_TYPE_ALL, Nb_Of_Devices, Device_IDs, NULL);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "HOST-Error: Failed to get available Devices" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Search for CL_DEVICE_NAME = Target_Device_Name
    // ............................................................................
    Device_Detected = false;
    for (ui = 0; ui < Nb_Of_Devices; ui++)
    {
        errCode = clGetDeviceInfo(Device_IDs[ui], CL_DEVICE_NAME, 0, NULL, &size);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "HOST-Error: Failed to get the size of the Device parameter value "
                 << "CL_DEVICE_NAME" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        device_info = new char[size];
        if (!device_info)
        {
            cout << endl
                 << "HOST-Error: Out of Memory during memory allocation for Device parameter "
                 << "CL_DEVICE_NAME"
                 << " value " << endl
                 << endl;
            return EXIT_FAILURE;
        }

        errCode = clGetDeviceInfo(Device_IDs[ui], CL_DEVICE_NAME, size, device_info, NULL);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "HOST-Error: Failed to get the "
                 << "CL_DEVICE_NAME"
                 << " device info" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        // Check if the current device matches Target_Device_Name
        // ............................................................................
        if (strcmp(device_info, Target_Device_Name) == 0)
        {
            Device_Detected = true;
            Target_Device_ID = Device_IDs[ui];
        }
    }

    if (Device_Detected == false)
    {
        cout << endl
             << "HOST-Error: Failed to get detect " << Target_Device_Name << " device" << endl
             << endl;
        return EXIT_FAILURE;
    }
    else
    {
#ifdef ALL_MESSAGES
        cout << "HOST-Info: Selected device              : " << Target_Device_Name << endl
             << endl;
#endif
    }

// ------------------------------------------------------------------------------------
// Step 2.3: Create Context
// ------------------------------------------------------------------------------------
#ifdef ALL_MESSAGES
    cout << "HOST-Info: Creating Context ... " << endl;
#endif
    Context = clCreateContext(0, 1, &Target_Device_ID, NULL, NULL, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "HOST-Error: Failed to create a Context" << endl
             << endl;
        return EXIT_FAILURE;
    }

// ------------------------------------------------------------------------------------
// Step 2.4: Create Command Queue (commands are executed in-order)
// ------------------------------------------------------------------------------------
#ifdef ALL_MESSAGES
    cout << "HOST-Info: Creating Command Queue ... " << endl;
#endif
    Command_Queue = clCreateCommandQueue(Context, Target_Device_ID, CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE | CL_QUEUE_PROFILING_ENABLE, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "HOST-Error: Failed to create a Command Queue" << endl
             << endl;
        return EXIT_FAILURE;
    }

// ============================================================================
// Step 3: Create Program and Kernel
// ============================================================================
//   o) Create a Program from a Binary File and Build it
//   o) Create a Kernel
// ============================================================================
#ifdef ALL_MESSAGES
    cout << endl;
    cout << "HOST-Info: ============================================================= " << endl;
    cout << "HOST-Info: (Step 3) Create Program and Kernels                           " << endl;
    cout << "HOST-Info: ============================================================= " << endl;
#endif

    // ------------------------------------------------------------------
    // Step 3.1: Load Binary File from a disk to Memory
    // ------------------------------------------------------------------
    unsigned char *xclbin_Memory;
    int program_length;

#ifdef ALL_MESSAGES
    cout << "HOST-Info: Loading " << xclbinFilename << " binary file to memory ..." << endl;
#endif

    program_length = loadFile2Memory(xclbinFilename, (char **)&xclbin_Memory);
    if (program_length < 0)
    {
        cout << endl
             << "HOST-Error: Failed to load " << xclbinFilename << " binary file to memory" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // ------------------------------------------------------------
    // Step 3.2: Create a program using a Binary File
    // ------------------------------------------------------------
    size_t Program_Length_in_Bytes;
    cl_program Program;
    cl_int Binary_Status;

#ifdef ALL_MESSAGES
    cout << "HOST-Info: Creating Program with Binary ..." << endl;
#endif
    Program_Length_in_Bytes = program_length;
    Program = clCreateProgramWithBinary(Context, 1, &Target_Device_ID, &Program_Length_in_Bytes,
                                        (const unsigned char **)&xclbin_Memory, &Binary_Status, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "HOST-Error: Failed to create a Program from a Binary" << endl
             << endl;
        return EXIT_FAILURE;
    }

// ----------------------------------------------------------------------
// Step 3.3: Build (compiles and links) a program executable from binary
// ----------------------------------------------------------------------
#ifdef ALL_MESSAGES
    cout << "HOST-Info: Building the Program ..." << endl;
#endif

    errCode = clBuildProgram(Program, 1, &Target_Device_ID, NULL, NULL, NULL);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "HOST-Error: Failed to build a Program Executable" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // -------------------------------------------------------------
    // Step 3.4: Create a Kernels
    // -------------------------------------------------------------
    cl_kernel EstimateMotion;

#ifdef ALL_MESSAGES
    cout << "HOST-Info: Creating a Kernel: EstimateMotion ..." << endl;
#endif
    EstimateMotion = clCreateKernel(Program, "estimate_motion", &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "HOST-Error: Failed to create EstimateMotion" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // ================================================================
    // Step 4: Prepare Data to Run Kernel
    // ================================================================
    //   o) Generate data for DataIn_1 array
    //   o) Generate data for DataIn_2 array
    //   o) Generate data for DataIn_3 array
    //   o) Allocate Memory to store the results: RES array
    //   o) Create Buffers in Global Memory to store data
    // ================================================================
    MATCH *match;
    int match_num;
    IPOINT *kp0;
    IPOINT *kp1;
    FLOAT fx, fy, cx, cy;
    FLOAT *depth;
    int threshold;
    FLOAT confidence;
    int maxiter;
    bool take_last = false;

    threshold = atoi(argv[6]);
    confidence = atof(argv[7]);
    maxiter = atoi(argv[8]);

    FLOAT *rmat;
    FLOAT *tvec;

#ifdef ALL_MESSAGES
    cout << endl;
    cout << "HOST-Info: ============================================================= " << endl;
    cout << "HOST-Info: (Step 4) Prepare Data to Run Kernels                           " << endl;
    cout << "HOST-Info: ============================================================= " << endl;
#endif

    // ------------------------------------------------------------------
    // Step 4.1: Allocate Memory to pointers
    // ------------------------------------------------------------------

    cout << "HOST-Info: fetching data ...";

    void *ptr = nullptr;

    cout << "HOST-Info: Align memory for match ... ";
    if (posix_memalign(&ptr, 4096, MAX_KEYPOINT_NUM * sizeof(MATCH)))
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for match array" << endl
             << endl;
        return EXIT_FAILURE;
    }
    match = reinterpret_cast<MATCH *>(ptr);

    cout << "HOST-Info: Align memory for kp0 ... ";
    if (posix_memalign(&ptr, 4096, MAX_KEYPOINT_NUM * sizeof(IPOINT)))
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for kp0 array" << endl
             << endl;
        return EXIT_FAILURE;
    }
    kp0 = reinterpret_cast<IPOINT *>(ptr);

    cout << "HOST-Info: Align memory for kp1 ... ";
    if (posix_memalign(&ptr, 4096, MAX_KEYPOINT_NUM * sizeof(IPOINT)))
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for kp1 array" << endl
             << endl;
        return EXIT_FAILURE;
    }
    kp1 = reinterpret_cast<IPOINT *>(ptr);

    cout << "HOST-Info: Align memory for depth ... ";
    if (posix_memalign(&ptr, 4096, IMAGE_WIDTH * IMAGE_HEIGTH * sizeof(FLOAT)))
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for depth array" << endl
             << endl;
        return EXIT_FAILURE;
    }
    depth = reinterpret_cast<FLOAT *>(ptr);

    read_data(match, match_num, kp0, kp1, fx, fy, cx, cy, depth, input_data_dir, 0);

    cout << "HOST-Info: Align memory for rmat ... ";
    if (posix_memalign(&ptr, 4096, 9 * sizeof(FLOAT)))
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for rmat array" << endl
             << endl;
        return EXIT_FAILURE;
    }
    rmat = reinterpret_cast<FLOAT *>(ptr);

    cout << "HOST-Info: Align memory for tvec ... ";
    if (posix_memalign(&ptr, 4096, 3 * sizeof(FLOAT)))
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for tvec array" << endl
             << endl;
        return EXIT_FAILURE;
    }
    tvec = reinterpret_cast<FLOAT *>(ptr);

    cout << endl;

// ------------------------------------------------------------------
// Step 4.2: Create Buffers in Global Memory to store data
//             o) GlobMem_BUF_matches  - stores matches (R)
//             o) GlobMem_BUF_kp0      - stores kp0     (R)
//             o) GlobMem_BUF_kp1      - stores kp1     (R)
//             o) GlobMem_BUF_depth    - stores depth   (R)
//             o) GlobMem_BUF_rmat     - stores Results from rmat  (R/W)
//             o) GlobMem_BUF_tvec     - stores Results from tvec  (R/W)
// ------------------------------------------------------------------
#ifdef ALL_MESSAGES
    cout << "HOST-Info: Allocating buffers in Global Memory to store Input and Output Data ..." << endl;
#endif
    cl_mem GlobMem_BUF_matches, GlobMem_BUF_kp0, GlobMem_BUF_kp1, GlobMem_BUF_depth, GlobMem_BUF_rmat, GlobMem_BUF_tvec;

    // Allocate Global Memory for GlobMem_BUF_matches
    // .......................................................
    GlobMem_BUF_matches = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, MAX_KEYPOINT_NUM * sizeof(MATCH), match, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_matches" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Allocate Global Memory for GlobMem_BUF_kp0
    // .......................................................
    GlobMem_BUF_kp0 = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, MAX_KEYPOINT_NUM * sizeof(IPOINT), kp0, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_kp0" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Allocate Global Memory for GlobMem_BUF_kp1
    // .......................................................
    GlobMem_BUF_kp1 = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, MAX_KEYPOINT_NUM * sizeof(IPOINT), kp1, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_kp1" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Allocate Global Memory for GlobMem_BUF_depth
    // .......................................................
    GlobMem_BUF_depth = clCreateBuffer(Context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, IMAGE_HEIGTH * IMAGE_WIDTH * sizeof(FLOAT), depth, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_depth" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Allocate Global Memory for GlobMem_BUF_rmat
    // .......................................................
    GlobMem_BUF_rmat = clCreateBuffer(Context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, 9 * sizeof(FLOAT), rmat, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_rmat" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Allocate Global Memory for GlobMem_BUF_tvec
    // .......................................................
    GlobMem_BUF_tvec = clCreateBuffer(Context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, 3 * sizeof(FLOAT), tvec, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_tvec" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // ============================================================================
    // Step 5: Set Kernel Arguments and Run the Application
    //         o) Set Kernel Arguments
    // 				----------------------------------------------------
    // 				 #  Argument
    // 				----------------------------------------------------
    //  			 0	GlobMem_BUF_matches
    //				 1	match_num
    //				 2	GlobMem_BUF_kp0
    //				 3	GlobMem_BUF_kp1
    //				 4	fx
    //				 5	fy
    //				 6	cx
    //				 7	cy
    //				 8	GlobMem_BUF_depth
    //				 9	threshold
    //				 10	confidence
    //				 11	maxiter
    //				 12	GlobMem_BUF_rmat
    //				 13	GlobMem_BUF_tvec
    //				 14	take_last
    // 				----------------------------------------------------
    //         o) Copy Input Data from Host to Global Memory
    //         o) Submit Kernels for Execution
    //         o) Copy Results from Global Memory to Host
    // ============================================================================

    int Nb_Of_Mem_Events = 6,
        Nb_Of_Exe_Events = 1;

    cl_event Mem_op_event[Nb_Of_Mem_Events],
        K_exe_event[Nb_Of_Exe_Events];

#ifdef ALL_MESSAGES
    cout << endl;
    cout << "HOST-Info: ============================================================= " << endl;
    cout << "HOST-Info: (Step 5) Run Application                                      " << endl;
    cout << "HOST-Info: ============================================================= " << endl;
#endif

// ----------------------------------------
// Step 5.1: Set Kernel Arguments
// ----------------------------------------
#ifdef ALL_MESSAGES
    cout << "HOST-Info: Setting Kernel arguments ..." << endl;
#endif
    errCode = false;

    errCode |= clSetKernelArg(EstimateMotion, 0, sizeof(cl_mem), &GlobMem_BUF_matches);
    errCode |= clSetKernelArg(EstimateMotion, 1, sizeof(int), &match_num);
    errCode |= clSetKernelArg(EstimateMotion, 2, sizeof(cl_mem), &GlobMem_BUF_kp0);
    errCode |= clSetKernelArg(EstimateMotion, 3, sizeof(cl_mem), &GlobMem_BUF_kp1);
    errCode |= clSetKernelArg(EstimateMotion, 4, sizeof(FLOAT), &fx);
    errCode |= clSetKernelArg(EstimateMotion, 5, sizeof(FLOAT), &fy);
    errCode |= clSetKernelArg(EstimateMotion, 6, sizeof(FLOAT), &cx);
    errCode |= clSetKernelArg(EstimateMotion, 7, sizeof(FLOAT), &cy);
    errCode |= clSetKernelArg(EstimateMotion, 8, sizeof(cl_mem), &GlobMem_BUF_depth);
    errCode |= clSetKernelArg(EstimateMotion, 9, sizeof(int), &threshold);
    errCode |= clSetKernelArg(EstimateMotion, 10, sizeof(FLOAT), &confidence);
    errCode |= clSetKernelArg(EstimateMotion, 11, sizeof(int), &maxiter);
    errCode |= clSetKernelArg(EstimateMotion, 12, sizeof(cl_mem), &GlobMem_BUF_rmat);
    errCode |= clSetKernelArg(EstimateMotion, 13, sizeof(cl_mem), &GlobMem_BUF_tvec);
    errCode |= clSetKernelArg(EstimateMotion, 14, sizeof(bool), &take_last);

    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-ERROR: Failed to set Kernel arguments" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // ------------------------------------------------------
    // Step 5.2: Copy Input data from Host to Global Memory
    // ------------------------------------------------------

    for (int iter = 0; iter < num_frames; iter++)
    {

        read_data(match, match_num, kp0, kp1, fx, fy, cx, cy, depth, input_data_dir, iter);
        errCode |= clSetKernelArg(EstimateMotion, 1, sizeof(int), &match_num);
        errCode |= clSetKernelArg(EstimateMotion, 14, sizeof(bool), &take_last);

#ifdef ALL_MESSAGES
        cout << "HOST_Info: Copy Input data to Global Memory ..." << endl;
#endif
        errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_matches, 0, 0, NULL, &Mem_op_event[0]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to write match to GlobMem_BUF_matches" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_kp0, 0, 0, NULL, &Mem_op_event[1]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to write kp0 to GlobMem_BUF_kp0" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_kp1, 0, 0, NULL, &Mem_op_event[2]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to write kp1 to GlobMem_BUF_kp1" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_depth, 0, 0, NULL, &Mem_op_event[3]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to write depth to GlobMem_BUF_depth" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        errCode = clEnqueueBarrierWithWaitList(Command_Queue, 0, 0, 0);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to barrier" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        // ----------------------------------------
        // Step 5.3: Submit Kernels for Execution
        // ----------------------------------------

        cout << "HOST-Info: Submitting Kernel EstimateMotion ..." << endl;

        errCode = clEnqueueTask(Command_Queue, EstimateMotion, 0, NULL, &K_exe_event[0]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "HOST-Error: Failed to submit EstimateMotion" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        // ---------------------------------------------------------
        // Step 5.4: Submit Copy Results from Global Memory to Host
        // ---------------------------------------------------------
#ifdef ALL_MESSAGES
        cout << "HOST_Info: Submitting Copy Results data from Global Memory to Host ..." << endl;
#endif

        errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_rmat, CL_MIGRATE_MEM_OBJECT_HOST, 1, &K_exe_event[0], &Mem_op_event[4]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to submit Copy Results from GlobMem_BUF_rmat to rmat" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_tvec, CL_MIGRATE_MEM_OBJECT_HOST, 1, &K_exe_event[0], &Mem_op_event[5]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to submit Copy Results from GlobMem_BUF_tvec to tvec" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        cout << endl
             << "HOST_Info: Waiting for application to be completed ..." << endl;
        clFinish(Command_Queue);

// ============================================================================
// Step 6: Processing Output Results
//         o) Store output results to a RES.txt file
//         o) Check correctness of the output results
// ============================================================================
#ifdef ALL_MESSAGES
        cout << endl;
        cout << "HOST-Info: ============================================================= " << endl;
        cout << "HOST-Info: (Step 6) Display result of iteration " << iter << endl;
        cout << "HOST-Info: ============================================================= " << endl;
#endif

        display_store_output(rmat, tvec, iter, output_data_dir);
        take_last = true;
    }

    // ============================================================================
    // Step 7: Release Allocated Resources
    // ============================================================================
    clReleaseDevice(Target_Device_ID); // Only available in OpenCL >= 1.2

    for (int i = 0; i < Nb_Of_Mem_Events; i++)
        clReleaseEvent(Mem_op_event[i]);
    for (int i = 0; i < Nb_Of_Exe_Events; i++)
        clReleaseEvent(K_exe_event[i]);

    clReleaseMemObject(GlobMem_BUF_kp0);
    clReleaseMemObject(GlobMem_BUF_kp1);
    clReleaseMemObject(GlobMem_BUF_matches);
    clReleaseMemObject(GlobMem_BUF_depth);
    clReleaseMemObject(GlobMem_BUF_rmat);
    clReleaseMemObject(GlobMem_BUF_tvec);

    clReleaseKernel(EstimateMotion);

    clReleaseProgram(Program);
    clReleaseCommandQueue(Command_Queue);
    clReleaseContext(Context);

    free(Platform_IDs);
    free(Device_IDs);
    free(kp0);
    free(kp1);
    free(match);
    free(depth);
    free(rmat);
    free(tvec);

    cout << endl
         << "HOST-Info: DONE" << endl
         << endl;

    return EXIT_SUCCESS;
}

// =========================================
// Helper Function: Loads program to memory
// =========================================
int loadFile2Memory(const char *filename, char **result)
{

    int size = 0;

    cout << filename << endl;
    std::ifstream stream(filename, std::ifstream::binary);
    if (!stream)
    {
        return -1;
    }

    stream.seekg(0, stream.end);
    size = stream.tellg();
    stream.seekg(0, stream.beg);

    *result = new char[size + 1];
    stream.read(*result, size);
    if (!stream)
    {
        return -2;
    }
    stream.close();
    (*result)[size] = 0;
    return size;
}

void read_data(MATCH *match, int &match_num,
               IPOINT *kp0, IPOINT *kp1,
               FLOAT &fx, FLOAT &fy, FLOAT &cx, FLOAT &cy,
               FLOAT *depth, char *data_dir, int idx)
{
    cout << endl;

    FILE *fp;

    char *match_dir = new char[100];
    char *match_num_dir = new char[100];
    char *kp0_dir = new char[100];
    char *kp1_dir = new char[100];
    char *depth_dir = new char[100];
    char *k_dir = new char[100];

    sprintf(match_dir, "%s/Matches_iteration%d.txt", data_dir, idx);
    sprintf(match_num_dir, "%s/Detected_Matches_iteration%d.txt", data_dir, idx);
    sprintf(kp0_dir, "%s/KP0_iteration%d.txt", data_dir, idx);
    sprintf(kp1_dir, "%s/KP1_iteration%d.txt", data_dir, idx);
    sprintf(depth_dir, "%s/Depth_iteration%d.txt", data_dir, idx);
    sprintf(k_dir, "%s/k.txt", data_dir);

    cout << "\t--- Reading Matched Index" << endl;

    fp = fopen(match_num_dir, "r");
    if (fp == NULL)
    {
        cout << "Error opening file " << match_num_dir << endl;
        return;
    }
    fscanf(fp, "%d", &match_num);
    fclose(fp);

    fp = fopen(match_dir, "r");
    if (fp == NULL)
    {
        cout << "Error opening file " << match_dir << endl;
        return;
    }
    for (int i = 0; i < match_num; i++)
    {
        int m, n;
        fscanf(fp, "%d %d", &m, &n);
        match[i].a = m;
        match[i].b = n;
    }
    fclose(fp);

    cout << "\t--- Reading Keypoint0" << endl;
    fp = fopen(kp0_dir, "r");
    if (fp == NULL)
    {
        cout << "Error opening file " << kp0_dir << endl;
        return;
    }
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        FLOAT m, n;
        fscanf(fp, "%f %f", &m, &n);
        kp0[i].x = m;
        kp0[i].y = n;
    }
    fclose(fp);

    cout << "\t--- Reading Keypoint1" << endl;
    fp = fopen(kp1_dir, "r");
    if (fp == NULL)
    {
        cout << "Error opening file " << kp1_dir << endl;
        return;
    }
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        FLOAT m, n;
        fscanf(fp, "%f %f", &m, &n);
        kp1[i].x = m;
        kp1[i].y = n;
    }
    fclose(fp);

    cout << "\t--- Reading Calibration Matix" << endl;
    fp = fopen(k_dir, "r");
    if (fp == NULL)
    {
        cout << "Error opening file " << k_dir << endl;
        return;
    }
    fscanf(fp, "%f %f %f %f", &fx, &cx, &fy, &cy);
    fclose(fp);

    cout << "\t--- Reading Depth Map" << endl;
    fp = fopen(depth_dir, "r");
    if (fp == NULL)
    {
        cout << "Error opening file " << depth_dir << endl;
        return;
    }
    for (int i = 0; i < IMAGE_HEIGTH; i++)
    {
        for (int j = 0; j < IMAGE_WIDTH; j++)
        {
            FLOAT m;
            fscanf(fp, "%f", &m);
            depth[IMAGE_WIDTH * i + j] = m;
        }
    }
    fclose(fp);
}

void display_store_output(FLOAT rmat[9], FLOAT tvec[3], int idx, char *output_dir)
{
    cout << "rmat" << endl;
    for (int j = 0; j < 3; j++)
    {
        for (int k = 0; k < 3; k++)
        {
            cout << rmat[j * 3 + k] << " ";
        }
        cout << endl;
    }
    cout << endl;

    cout << "tvec" << endl;
    for (int j = 0; j < 3; j++)
    {
        cout << tvec[j] << " ";
    }
    cout << endl
         << endl;

    sprintf(output_dir, "%s/R_t_%d.txt", output_dir, idx);
    FILE *fp = fopen(output_dir, "w");
    if (fp == NULL)
    {
        cout << "Error opening file " << output_dir << endl;
        return;
    }
    
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            fprintf(fp, "%f ", rmat[i * 3 + j]);
        fprintf(fp, "\n");
    }
    for (int i = 0; i < 3; i++)
        fprintf(fp, "%f ", tvec[i]);
    fclose(fp);
}
