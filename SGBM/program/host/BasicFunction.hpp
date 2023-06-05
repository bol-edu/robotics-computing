/**********
Copyright (c) 2019, Xilinx, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********/

#pragma once

//#define CL_API_PREFIX__VERSION_1_2_DEPRECATED
//#define CL_HPP_CL_1_2_DEFAULT_BUILD

//#define CL_HPP_ENABLE_PROGRAM_CONSTRUCTION_FROM_ARRAY_COMPATIBILITY 1
//#define CL_USE_DEPRECATED_OPENCL_1_2_APIS


//#include <CL/opencl.hpp> //"/opt/intel/opencl-1.2-4.4.0.117/include/CL/cl.h"
//#include <CL/cl2.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include "Host.hpp"


// When creating a buffer with user pointer (CL_MEM_USE_HOST_PTR), under the hood
// User ptr is used if and only if it is properly aligned (page aligned). When not
// aligned, runtime has no choice but to create its own host side buffer that backs
// user ptr. This in turn implies that all operations that move data to and from
// device incur an extra memcpy to move data to/from runtime's own host buffer
// from/to user poin#include "BasicFunction.hpp"ter. So it is recommended to use this allocator if user wish to
// Create Buffer/Memory Object with CL_MEM_USE_HOST_PTR to align user buffer to the
// page boundary. It will ensure that user buffer will be used when user create
// Buffer/Mem Object with CL_MEM_USE_HOST_PTR.
template <typename T>
struct aligned_allocator {
    using value_type = T;
    T* allocate(std::size_t num) {
        void* ptr = nullptr;
        if (posix_memalign(&ptr, 4096, num * sizeof(T))) throw std::bad_alloc();
        return reinterpret_cast<T*>(ptr);
    }
    void deallocate(T* p, std::size_t num) { free(p); }
};

namespace BasicFunction {
std::vector<cl::Device> get_devices_of_platform(const std::string& Target_Platform_Vendor);
cl::Device get_specific_device(const std::string& Target_Device_Name, const std::vector<cl::Device>& devices);
/* find_xclbin_file
 *
 *
 * Description:
 *   Find precompiled program (as commonly created by the Xilinx OpenCL
 *   flow). Using search path below.
 *
 *   Search Path:
 *      $XCL_BINDIR/<name>.<target>.<device>.xclbin
 *      $XCL_BINDIR/<name>.<target>.<device_versionless>.xclbin
 *      $XCL_BINDIR/binary_container_1.xclbin
 *      $XCL_BINDIR/<name>.xclbin
 *      xclbin/<name>.<target>.<device>.xclbin
 *      xclbin/<name>.<target>.<device_versionless>.xclbin
 *      xclbin/binary_container_1.xclbin
 *      xclbin/<name>.xclbin
 *      ../<name>.<target>.<device>.xclbin
 *      ../<name>.<target>.<device_versionless>.xclbin
 *      ../binary_container_1.xclbin
 *      ../<name>.xclbin
 *      ./<name>.<target>.<device>.xclbin
 *      ./<name>.<target>.<device_versionless>.xclbin
 *      ./binary_container_1.xclbin
 *      ./<name>.xclbin
 *
 * Inputs:
 *   _device_name - Targeted Device name
 *   xclbin_name - base name of the xclbin to import.
 *
 * Returns:
 *   An opencl program Binaries object that was created from xclbin_name file.
 */
std::string find_binary_file(const std::string& _device_name, const std::string& xclbin_name);
cl::Program::Binaries import_binary_file(std::string xclbin_file_name);
char* read_binary_file(const std::string& xclbin_file_name, unsigned& nb);

std::vector<std::string> read_image_folder(const int FRAME_NUM, const std::string& folder_path);
void read_calibration(const std::string& File_Path, cv::Mat* P0, cv::Mat* P1);

void myimread(uchar* Image_Pointer, const std::string& File_Path, int Width, int Height, int Channel);

void print_content(const std::string& Path_and_Name, cv::Mat* Matrix);

// Template function must be writen at .h
template <typename T>
void print_content(const std::string& File_Path, T* arr, int col, int row) {
    std::ofstream fout;
    fout.open(File_Path.c_str(), std::ios::out);
    for (int i = 0; i < row; i++) {
        if (i == 0) fout << "[";
        else fout << " ";

        for (int j = 0; j < col; j++) {
            fout.width(7);
            fout.setf(std::ios::right);
            fout << arr[i * col + j];
            if (j < col - 1) fout << ", ";
            else if (i < row - 1) fout << ";";
        }

        if (i < row - 1) fout << std::endl;
        else  fout << "]";
    }
    fout.close();

}

template <typename T>
void read_txt(const std::string& File_Path, cv::Mat* Matrix){
    std::ifstream fin;
    std::string row;
    std::vector<T> elem;
    fin.open(File_Path.c_str(), std::ios::in);
    while(true){
        fin >> row;
        std::stringstream s(row);
        
    }
    fin.close();
    
}

void read_txt(const std::string& File_Path, cv::Mat* Matrix);
bool is_emulation();
bool is_hw_emulation();
bool is_xpr_device(const char* device_name);
}
