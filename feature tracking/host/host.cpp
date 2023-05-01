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

#include <opencv2/opencv.hpp>

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
// ============================================================================
#ifdef ALL_MESSAGES
    cout << "HOST-Info: ============================================================= " << endl;
    cout << "HOST-Info: (Step 1) Check Command Line Arguments                      " << endl;
    cout << "HOST-Info: ============================================================= " << endl;
#endif

    if (argc != 4)
    {
        cout << "HOST-Error: Incorrect command line syntax " << endl;
        cout << "HOST-Info:  Usage: " << argv[0] << " <Platform_Vendor> <Device_Name> <XCLBIN_File>  <Test Vectors Size>" << endl
             << endl;
        return EXIT_FAILURE;
    }

    const char *Target_Platform_Vendor = argv[1];
    const char *Target_Device_Name = argv[2];
    const char *xclbinFilename = argv[3];
    cout << "HOST-Info: Platform_Vendor   : " << Target_Platform_Vendor << endl;
    cout << "HOST-Info: Device_Name       : " << Target_Device_Name << endl;
    cout << "HOST-Info: XCLBIN_file       : " << xclbinFilename << endl;

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
    cl_kernel FeatureMatching;

#ifdef ALL_MESSAGES
    cout << "HOST-Info: Creating a Kernel: FeatureMatching ..." << endl;
#endif
    FeatureMatching = clCreateKernel(Program, "match_feature", &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "HOST-Error: Failed to create FeatureMatching" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // ================================================================
    // Step 4: Prepare Data to Run Kernel
    // ================================================================
    //   o) Declare pointer for des0 array
    //   o) Declare pointer for des1 array
    //   o) Declare pointer for match array
    //   o) Create Buffers in Global Memory to store data
    // ================================================================
    uchar *des0;
    uchar *des1;
    FLOAT dist_threshold;
    MATCH *match;
    int match_num;

#ifdef ALL_MESSAGES
    cout << endl;
    cout << "HOST-Info: ============================================================= " << endl;
    cout << "HOST-Info: (Step 4) Prepare Data to Run Kernels                           " << endl;
    cout << "HOST-Info: ============================================================= " << endl;
#endif

    // ------------------------------------------------------------------
    // Step 4.1: Declare pointer for des0 array
    //           Declare pointer for des1 array
    //           Declare pointer for match array
    // ------------------------------------------------------------------

    cout << "HOST-Info: fetching data ...";

    void *ptr = nullptr;

    cout << "HOST-Info: Align memory for des0 ... ";
    if (posix_memalign(&ptr, 4096, DES0_SIZE * sizeof(uchar)))
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for des0 array" << endl
             << endl;
        return EXIT_FAILURE;
    }
    des0 = reinterpret_cast<uchar *>(ptr);

    cout << "HOST-Info: Align memory for des1 ... ";
    if (posix_memalign(&ptr, 4096, DES1_SIZE * sizeof(uchar)))
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for des1 array" << endl
             << endl;
        return EXIT_FAILURE;
    }
    des1 = reinterpret_cast<uchar *>(ptr);

    cout << "HOST-Info: Align memory for match ... ";
    if (posix_memalign(&ptr, 4096, MAX_KEYPOINT_NUM * sizeof(MATCH)))
    {
        cout << endl
             << "HOST-Error: Out of Memory during memory allocation for match array" << endl
             << endl;
        return EXIT_FAILURE;
    }
    match = reinterpret_cast<MATCH *>(ptr);

    cout << endl;

// ------------------------------------------------------------------
// Step 4.2: Create Buffers in Global Memory to store data
//             o) GlobMem_BUF_des0 - stores des0 (R)
//             o) GlobMem_BUF_des0 - stores des1 (R)
//             o) GlobMem_BUF_match - stores match (W)
// ------------------------------------------------------------------
#ifdef ALL_MESSAGES
    cout << "HOST-Info: Allocating buffers in Global Memory to store Input and Output Data ..." << endl;
#endif
    cl_mem GlobMem_BUF_match, GlobMem_BUF_des0, GlobMem_BUF_des1;

    // Allocate Global Memory for GlobMem_BUF_match
    // .......................................................
    GlobMem_BUF_match = clCreateBuffer(Context, CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR, MAX_KEYPOINT_NUM * sizeof(MATCH), match, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_match" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Allocate Global Memory for GlobMem_BUF_des0
    // .......................................................
    GlobMem_BUF_des0 = clCreateBuffer(Context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, DES0_SIZE * sizeof(uchar), des0, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_des0" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // Allocate Global Memory for GlobMem_BUF_des1
    // .......................................................
    GlobMem_BUF_des1 = clCreateBuffer(Context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, DES1_SIZE * sizeof(uchar), des1, &errCode);
    if (errCode != CL_SUCCESS)
    {
        cout << endl
             << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_des1" << endl
             << endl;
        return EXIT_FAILURE;
    }

    // ============================================================================
    // Step 5: Set Kernel Arguments and Run the Application
    //         o) Set Kernel Arguments
    // 				----------------------------------------------------
    // 				 Kernel	  		    Argument Nb		Description
    // 				----------------------------------------------------
    //  			 FeatureMatching	0			    GlobMem_BUF_des0
    //				 FeatureMatching	1			    GlobMem_BUF_des1
    //				 FeatureMatching	2			    dist_threshold
    //				 FeatureMatching	3			    GlobMem_BUF_match
    //				 FeatureMatching	4			    match_num
    // 				----------------------------------------------------
    //         o) Copy Input Data from Host to Global Memory
    //         o) Submit Kernels for Execution
    //         o) Copy Results from Global Memory to Host
    // ============================================================================
    int Nb_Of_Mem_Events = 3,
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

    errCode |= clSetKernelArg(FeatureMatching, 0, sizeof(cl_mem), &GlobMem_BUF_des0);
    errCode |= clSetKernelArg(FeatureMatching, 1, sizeof(cl_mem), &GlobMem_BUF_des1);
    errCode |= clSetKernelArg(FeatureMatching, 2, sizeof(FLOAT), &dist_threshold);
    errCode |= clSetKernelArg(FeatureMatching, 3, sizeof(cl_mem), &GlobMem_BUF_match);
    errCode |= clSetKernelArg(FeatureMatching, 4, sizeof(int), &match_num);

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

    for (int ii = 0; ii < num_frames - 1; ii++)
    {
        read_data(des0, des1);

#ifdef ALL_MESSAGES
        cout << "HOST_Info: Copy Input data to Global Memory ..." << endl;
#endif
        errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_des0, 0, 0, NULL, &Mem_op_event[0]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to write match to GlobMem_BUF_match" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_des1, 0, 0, NULL, &Mem_op_event[1]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to write kp0 to GlobMem_BUF_kp0" << endl
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

        cout << "HOST-Info: Submitting Kernel FeatureMatching ..." << endl;

        errCode = clEnqueueTask(Command_Queue, FeatureMatching, 0, NULL, &K_exe_event[0]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "HOST-Error: Failed to submit FeatureMatching" << endl
                 << endl;
            return EXIT_FAILURE;
        }

// ---------------------------------------------------------
// Step 5.4: Submit Copy Results from Global Memory to Host
// ---------------------------------------------------------
#ifdef ALL_MESSAGES
        cout << "HOST_Info: Submitting Copy Results data from Global Memory to Host ..." << endl;
#endif

        errCode = clEnqueueMigrateMemObjects(Command_Queue, 1, &GlobMem_BUF_match, CL_MIGRATE_MEM_OBJECT_HOST, 1, &K_exe_event[0], &Mem_op_event[2]);
        if (errCode != CL_SUCCESS)
        {
            cout << endl
                 << "Host-Error: Failed to submit Copy Results from GlobMem_BUF_match to rmat" << endl
                 << endl;
            return EXIT_FAILURE;
        }

        cout << endl
             << "HOST_Info: Waiting for application to be completed ..." << endl;
        clFinish(Command_Queue);

        // ------------------------------------------------------
        // Step 6.1: Store output Result to the trajectory.txt file
        // ------------------------------------------------------
        FILE *fp;
        char file_name[50];
        sprintf(file_name, "../../../output/match_%d.txt", ii);
        fp = fopen(file_name, "w");
        fprintf(fp, "%d\n", match_num);
        for (int i = 0; i < match_num; i++)
        {
            fprintf(fp, "%d\t%d\n", match[i].a, match[i].b);
        }
        fclose(fp);
    }

    // ============================================================================
    // Step 8: Release Allocated Resources
    // ============================================================================
    clReleaseDevice(Target_Device_ID); // Only available in OpenCL >= 1.2

    for (int i = 0; i < Nb_Of_Mem_Events; i++)
        clReleaseEvent(Mem_op_event[i]);
    for (int i = 0; i < Nb_Of_Exe_Events; i++)
        clReleaseEvent(K_exe_event[i]);

    clReleaseMemObject(GlobMem_BUF_des0);
    clReleaseMemObject(GlobMem_BUF_des1);
    clReleaseMemObject(GlobMem_BUF_match);

    clReleaseKernel(FeatureMatching);

    clReleaseProgram(Program);
    clReleaseCommandQueue(Command_Queue);
    clReleaseContext(Context);

    free(Platform_IDs);
    free(Device_IDs);
    free(des0);
    free(des1);
    free(match);

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

void read_data(uchar *des0, uchar *des1, int idx)
{
    cout << endl;

    FILE *fp;

    char des0_name[50];
    char des1_name[50];

    sprintf(des0_name, "../../../testdata/des0_%d.txt", idx);
    sprintf(des1_name, "../../../testdata/des1_%d.txt", idx);

    cout << "\t--- Reading descriptor 0" << endl;
    fp = fopen(des0_name, "r");
    for (int i = 0; i < DES0_SIZE; i++)
    {
        uchar m;
        fscanf(fp, "%hhu", &m);
        des0[i] = m;
    }
    fclose(fp);

    cout << "\t--- Reading descriptor 1" << endl;
    fp = fopen(des1_name, "r");
    for (int i = 0; i < DES1_SIZE; i++)
    {
        uchar m;
        fscanf(fp, "%hhu", &m);
        des1[i] = m;
    }
    fclose(fp);

    cout << endl;
}
