#include <cstring>
#include <iostream>


using namespace std;
enum errDetailEnum{
    err0,   // HOST-Error: Incorrect command line syntax 
    err1,   // HOST-Error: Failed to get the number of available platforms
    err2,   // HOST-Error: Out of Memory during memory allocation for Platform_IDs
    err3,   // HOST-Error: Failed to get the available platforms
    err4,   // HOST-Error: Failed to get the size of the Platofrm parameter CL_PLATFORM_VENDOR value
    err5,   // HOST-Error: Out of Memory during memory allocation for Platform Parameter CL_PLATFORM_VENDOR
    err6,   // HOST-Error: Failed to get the CL_PLATFORM_VENDOR platform info
    err7,   // HOST-Error: Failed to get detect <Target_Platform_Vendor> platform
    err8,   // HOST-Error: Failed to get the number of available Devices
    err9,   // HOST-Error: Out of Memory during memory allocation for Device_IDs
    err10,  // HOST-Error: Failed to get available Devices
    err11,  // HOST-Error: Failed to get the size of the Device parameter value CL_DEVICE_NAME
    err12,  // HOST-Error: Out of Memory during memory allocation for Device parameter CL_DEVICE_NAME value 
    err13,  // HOST-Error: Failed to get the CL_DEVICE_NAME device info
    err14,  // HOST-Error: Failed to get detect <Target_Device_Name> device
    err15,  // HOST-Error: Failed to create a Context
    err16,  // HOST-Error: Failed to create a Command Queue
    err17,  // HOST-Error: Failed to load <xclbinFilename> binary file to memory
    err18,  // HOST-Error: Failed to create a Program from a Binary
    err19,  // HOST-Error: Failed to build a Program Executable
    err20,  // HOST-Error: Failed to create K_StereoMatching
    err21,  // HOST-Error: Failed to create K_FeatureExtract
    err22,  // HOST-Error: Failed to create K_FeatureTracker
    err23,  // HOST-Error: Failed to create K_MotionEstimation
    err24,  // 
    err25,  // 
    err26,  // 
    err27,  // 
    err28,  // 
    err29,  // 
    err30,  // 
    err31,  // 
    err32,  // 
    err33,  // 

};

int ErrHandler(int errDetail, const char* extraArgv, int EXIT_STATUS){
    switch (errDetail){
    case err0:
        cout << "HOST-Error " << errDetail << " : Incorrect command line syntax " << endl;
        cout << "HOST-Info:  Usage: " << extraArgv << " <Platform_Vendor> <Device_Name> <XCLBIN_File>  <Test Vectors Size>" << endl
            << endl;
        break;
    
    case err1:
        cout << endl
            << "HOST-Error " << errDetail << " : Failed to get the number of available platforms" << endl
            << endl;
        break;

    case err2:
        cout << endl
			 << "HOST-Error " << errDetail << " : Out of Memory during memory allocation for Platform_IDs" << endl
			 << endl;
        break;
    
    case err3:
        cout << endl
			 << "HOST-Error " << errDetail << " : Failed to get the available platforms" << endl
			 << endl;
        break;

    case err4:
        cout << endl
             << "HOST-Error " << errDetail << " : Failed to get the size of the Platofrm parameter "
             << "CL_PLATFORM_VENDOR"
             << " value " << endl
             << endl;
        break;
        
    case err5:
        cout << endl
			 << "HOST-Error " << errDetail << " : Out of Memory during memory allocation for Platform Parameter "
			 << "CL_PLATFORM_VENDOR" << endl
			 << endl;
        break;
        
    case err6:
        cout << endl
             << "HOST-Error " << errDetail << " : Failed to get the "
             << "CL_PLATFORM_VENDOR"
             << " platform info" << endl
             << endl;
        break;
        
    case err7:
    	cout << endl
			 << "HOST-Error " << errDetail << " : Failed to get detect " << extraArgv << " platform" << endl
			 << endl;
        break;
        
    case err8:
        cout << endl
			 << "HOST-Error " << errDetail << " : Failed to get the number of available Devices" << endl
			 << endl;
        break;
        
    case err9:
        cout << endl
			 << "HOST-Error " << errDetail << " : Out of Memory during memory allocation for Device_IDs" << endl
			 << endl;
        break;
        
    case err10:
        cout << endl
			 << "HOST-Error " << errDetail << " : Failed to get available Devices" << endl
			 << endl;
        break;
        
    case err11:
        cout << endl
             << "HOST-Error " << errDetail << " : Failed to get the size of the Device parameter value "
			 << "CL_DEVICE_NAME" << endl
			 << endl;
        break;

    case err12:
        cout << endl
             << "HOST-Error " << errDetail << " : Out of Memory during memory allocation for Device parameter "
             << "CL_DEVICE_NAME"
             << " value " << endl
             << endl;
        break;

    case err13:
        cout << endl
             << "HOST-Error " << errDetail << " : Failed to get the "
             << "CL_DEVICE_NAME"
			 << " device info" << endl
			 << endl;
        break;

    case err14:
        cout << endl
			 << "HOST-Error " << errDetail << " : Failed to get detect " << extraArgv << " device" << endl
			 << endl;
        break;

    case err15:
        cout << endl
			 << "HOST-Error: Failed to create a Context" << endl
			 << endl;
        break;

    case err16:
        cout << endl
			 << "HOST-Error: Failed to create a Command Queue" << endl
			 << endl;
        break;
        
    case err17:
        cout << endl
			 << "HOST-Error: Failed to load " << extraArgv << " binary file to memory" << endl
			 << endl;
        break;
        
    case err18:
        cout << endl
			 << "HOST-Error: Failed to create a Program from a Binary" << endl
			 << endl;
        break;

    case err19:
        cout << endl
			 << "HOST-Error: Failed to build a Program Executable" << endl
			 << endl;
        break;

    case err20:
        cout << endl
			 << "HOST-Error: Failed to create K_StereoMatching" << endl
			 << endl;
        break;

    case err21:
        cout << endl
			 << "HOST-Error: Failed to create K_FeatureExtract" << endl
			 << endl;
        break;

    case err22:
        cout << endl
			 << "HOST-Error: Failed to create K_FeatureTracker" << endl
			 << endl;
        break;
        
    case err23:
        cout << endl
			 << "HOST-Error: Failed to create K_MotionEstimation" << endl
			 << endl;
        break;

    case err24:

        break;

    case err25:

        break;

    case err26:

        break;

    case err27:

        break;

    case err28:

        break;
        







    default:
        break;
    }
    return EXIT_STATUS;
}
