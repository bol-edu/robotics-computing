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
    err24,  // HOST-Error: Out of Memory during memory allocation for ImgLeft_0 array
    err25,  // HOST-Error: Out of Memory during memory allocation for ImgRight_0 array
    err26,  // HOST-Error: Out of Memory during memory allocation for ImgLeft_1 array
    err27,  // HOST-Error: Out of Memory during memory allocation for Mask array
    err28,  // HOST-Error: Out of Memory during memory allocation for K_Left array
    err29,  // HOST-Error: Out of Memory during memory allocation for T_Left array
    err30,  // HOST-Error: Out of Memory during memory allocation for T_Right array
    err31,  // HOST-Error: Out of Memory during memory allocation for Filter array
    err32,  // HOST-Error: Out of Memory during memory allocation for T_Mat array
    err33,  // Host-Error: Failed to allocate Global Memory for GlobMem_ImgLeft_0
    err34,  // Host-Error: Failed to allocate Global Memory for GlobMem_ImgRight_0
    err35,  // Host-Error: Failed to allocate Global Memory for GlobMem_ImgLeft_1
    err36,  // Host-Error: Failed to allocate Global Memory for GlobMem_Mask
    err37,  // Host-Error: Failed to allocate Global Memory for GlobMem_K_Left
    err38,  // Host-Error: Failed to allocate Global Memory for GlobMem_T_Left
    err39,  // Host-Error: Failed to allocate Global Memory for GlobMem_T_Right
    err40,  // Host-Error: Failed to allocate Global Memory for GlobMem_Filter
    err41,  // Host-Error: Failed to allocate Global Memory for GlobMem_T_Mat
    err42,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Depth
    err43,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_KP0
    err44,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_KP1
    err45,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Des0
    err46,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Des1
    err47,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Detected_Points
    err48,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Matches
    err49,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Detected_Matches
    err50,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_rmat
    err51,  // Host-Error: Failed to allocate Global Memory for GlobMem_BUF_tvec
    err52,  // Host-ERROR: Failed to set Kernel arguments
    err53,  // Host-Error: Failed to write ImgLeft_0 to GlobMem_ImgLeft_0
    err54,  // Host-Error: Failed to write ImgRight_0 to GlobMem_ImgRight_0
    err55,  // Host-Error: Failed to write ImgLeft_1 to GlobMem_ImgLeft_1
    err56,  // Host-Error: Failed to write Mask to GlobMem_Mask 
    err57,  // Host-Error: Failed to write K_Left to GlobMem_K_Left
    err58,  // Host-Error: Failed to write T_Left to GlobMem_T_Left
    err59,  // Host-Error: Failed to write T_Right to GlobMem_T_Right
    err60,  // Host-Error: Failed to write Filter to GlobMem_Filter
    err61,  // Host-Error: Failed to write T_Mat to GlobMem_T_Mat
    err62,  // Host-Error: Failed Migrate GlobMem_BUF_Depth without migrating content
    err63,  // Host-Error: Failed Migrate GlobMem_BUF_KP0 without migrating content
    err64,  // Host-Error: Failed Migrate GlobMem_BUF_KP1 without migrating content
    err65,  // Host-Error: Failed Migrate GlobMem_BUF_Des0 without migrating content
    err66,  // Host-Error: Failed Migrate GlobMem_BUF_Des1 without migrating content 
    err67,  // Host-Error: Failed Migrate GlobMem_BUF_Detected_Points without migrating content 
    err68,  // Host-Error: Failed Migrate GlobMem_BUF_Matches without migrating content
    err69,  // Host-Error: Failed Migrate GlobMem_BUF_Detected_Matches without migrating content
    err70,  // Host-Error: Failed Migrate GlobMem_BUF_rmat without migrating content
    err71,  // Host-Error: Failed Migrate GlobMem_BUF_tvec without migrating content
    err72,  // HOST-Error: Failed to Submit BarrierWithWaitList
    err73,  // HOST-Error: Failed to submit K_StereoMatching
    err74,  // HOST-Error: Failed to submit K_FeatureExtraction
    err75,  // HOST-Error: Failed to submit K_FeatureTracking
    err76,  // HOST-Error: Failed to submit K_MotionEstimation
    err77,  // Host-Error: Failed to submit Copy Results from GlobMem_T_Mat to T_Mat
    err78,  // 
    err79,  // 
    err80,  // 
    err81,  // 
    err82,  // 
    err83,  // 
    err84,  // 
    err85,  // 
    err86,  // 
    err87,  // 
    err88,  // 
    err89,  // 
    err90,  // 
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
        cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for ImgLeft_0 array" << endl
			 << endl;
        break;

    case err25:
        cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for ImgRight_0 array" << endl
			 << endl;
        break;

    case err26:
        cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for ImgLeft_1 array" << endl
			 << endl;
        break;

    case err27:
        cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for Mask array" << endl
			 << endl;
        break;

    case err28:
        cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for K_Left array" << endl
			 << endl;
        break;

    case err29:
        cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for T_Left array" << endl
			 << endl;
        break;

    case err30:
        cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for T_Right array" << endl
			 << endl;
        break;
        
    case err31:
        cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for Filter array" << endl
			 << endl;
        break;
        
    case err32:
        cout << endl
			 << "HOST-Error: Out of Memory during memory allocation for T_Mat array" << endl
			 << endl;
        break;
        
    case err33:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_ImgLeft_0" << endl
			 << endl;
        break;
        
    case err34:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_ImgRight_0" << endl
			 << endl;
        break;
        
    case err35:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_ImgLeft_1" << endl
			 << endl;
        break;
        
    case err36:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_Mask" << endl
			 << endl;
        break;
        
    case err37:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_K_Left" << endl
			 << endl;
        break;
        
    case err38:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_T_Left" << endl
			 << endl;
        break;
        
    case err39:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_T_Right" << endl
			 << endl;
        break;
        
    case err40:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_Filter" << endl
			 << endl;
        break;
        
    case err41:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_T_Mat" << endl
			 << endl;
        break;
        
    case err42:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Depth" << endl
			 << endl;
        break;
        
    case err43:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_KP0" << endl
			 << endl;
        break;
        
    case err44:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_KP1" << endl
			 << endl;
        break;
        
    case err45:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Des0" << endl
			 << endl;
        break;
        
    case err46:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Des1" << endl
			 << endl;
        break;
        
    case err47:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Detected_Points" << endl
			 << endl;
        break;

    case err48:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Matches" << endl
			 << endl;
        break;

    case err49:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_Detected_Matches" << endl
			 << endl;
        break;

    case err50:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_rmat" << endl
			 << endl;
        break;

    case err51:
        cout << endl
			 << "Host-Error: Failed to allocate Global Memory for GlobMem_BUF_tvec" << endl
			 << endl;
        break;

    case err52:
        cout << endl
			 << "Host-ERROR: Failed to set Kernel arguments" << endl
			 << endl;
        break;    

    case err53:
        cout << endl
			 << "Host-Error: Failed to write ImgLeft_0 to GlobMem_ImgLeft_0" << endl
			 << endl;
        break;   

    case err54:
        cout << endl
			 << "Host-Error: Failed to write ImgRight_0 to GlobMem_ImgRight_0" << endl
			 << endl;
        break;   

    case err55:
        cout << endl
			 << "Host-Error: Failed to write ImgLeft_1 to GlobMem_ImgLeft_1" << endl
			 << endl;
        break;   
        
    case err56:
        cout << endl
			 << "Host-Error: Failed to write Mask to GlobMem_Mask" << endl
			 << endl;
        break;   
        
    case err57:
        cout << endl
			 << "Host-Error: Failed to write K_Left to GlobMem_K_Left" << endl
			 << endl;
        break;   
        
    case err58:
        cout << endl
			 << "Host-Error: Failed to write T_Left to GlobMem_T_Left" << endl
			 << endl;
        break;   
        
    case err59:
        cout << endl
			 << "Host-Error: Failed to write T_Right to GlobMem_T_Right" << endl
			 << endl;
        break;   
        
    case err60:
        cout << endl
			 << "Host-Error: Failed to write Filter to GlobMem_Filter" << endl
			 << endl;
        break;   
        
    case err61:
        cout << endl
			 << "Host-Error: Failed to write T_Mat to GlobMem_T_Mat" << endl
			 << endl;
        break;   
        
    case err62:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_Depth without migrating content" << endl
			 << endl;
        break;   
        
    case err63:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_KP0 without migrating content" << endl
			 << endl;
        break;   
        
    case err64:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_KP1 without migrating content" << endl
			 << endl;
        break;   
        
    case err65:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_Des0 without migrating content" << endl
			 << endl;
        break;   
        
    case err66:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_Des1 without migrating content" << endl
			 << endl;
        break;   
        
    case err67:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_Detected_Points without migrating content" << endl
			 << endl;
        break;   
        
    case err68:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_Matches without migrating content" << endl
			 << endl;
        break;   
        
    case err69:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_Detected_Matches without migrating content" << endl
			 << endl;
        break;   
        
    case err70:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_rmat without migrating content" << endl
			 << endl;
        break;   
    
    case err71:
        cout << endl
			 << "Host-Error: Failed Migrate GlobMem_BUF_tvec without migrating content" << endl
			 << endl;
        break;  
        
    case err72:
        cout << endl
			 << "HOST-Error: Failed to Submit BarrierWithWaitList" << endl
			 << endl;
        break;  
        
    case err73:
        cout << endl
			 << "HOST-Error: Failed to submit K_StereoMatching" << endl
			 << endl;
        break;  
        
    case err74:
        cout << endl
			 << "HOST-Error: Failed to submit K_FeatureExtraction" << endl
			 << endl;
        break;  
        
    case err75:
        cout << endl
			 << "HOST-Error: Failed to submit K_FeatureTracking" << endl
			 << endl;
        break;  
        
    case err76:
        cout << endl
			 << "HOST-Error: Failed to submit K_MotionEstimation" << endl
			 << endl;
        break;  
        
    case err77:
        cout << endl
			 << "Host-Error: Failed to submit Copy Results from GlobMem_T_Mat to T_Mat" << endl
			 << endl;
        break;  
        
    case err78:
        cout << endl
			 << "" << endl
			 << endl;
        break;  
        
    case err79:
        cout << endl
			 << "" << endl
			 << endl;
        break;  
        
    case err80:
        cout << endl
			 << "" << endl
			 << endl;
        break;  
        
        







    default:
        break;
    }
    return EXIT_STATUS;
}
