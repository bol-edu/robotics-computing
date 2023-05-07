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

#include <opencv2/opencv.hpp>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include "libpng/png.h"
#include "Host.hpp"

namespace BasicFunction {

template <typename T>
T* align_allocator (std::size_t Size_of_T, std::size_t Alignment=4096) {
    void* ptr = nullptr;
    if (posix_memalign(&ptr, Alignment, Size_of_T * sizeof(T))){
        std::cout << std::endl << "[HOST-Error] Out of Memory during memory alignment allocation" << std::endl << std::endl;
        throw std::bad_alloc();
    }
    return reinterpret_cast<T*>(ptr);
}

template <typename T>
void deallocate(T* ptr) { free(ptr); }

std::vector<cl::Device> get_devices_of_platform(const std::string& Target_Platform_Vendor);
cl::Device get_specific_device(const std::string& Target_Device_Name, const std::vector<cl::Device>& devices);

std::string find_binary_file(const std::string& _device_name, const std::string& xclbin_name);
cl::Program::Binaries import_binary_file(std::string xclbin_file_name);
char* read_binary_file(const std::string& xclbin_file_name, unsigned& nb);

std::vector<std::string> read_image_folder(const int FRAME_NUM, const std::string& folder_path);
void read_calibration(const std::string& File_Path, cv::Mat* P0, cv::Mat* P1);

void myimread(const std::string& File_Path, uchar* Image_Pointer, cv::Mat* Std_Image=NULL);

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

void matwrite(const std::string& filename, cv::Mat* mat);
void matread(const std::string& filename, cv::Mat* mat);

bool is_emulation();
bool is_hw_emulation();
bool is_xpr_device(const char* device_name);
}
