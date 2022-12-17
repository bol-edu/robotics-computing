/**********
Copyright (c) 2018, Xilinx, Inc.
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

#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;

#include "help_functions.h"

// =========================================
// Helper Function: Loads program to memory
// =========================================
int loadFile2Memory(const char *filename, char **result) {

    int size = 0;

    std::ifstream stream(filename, std::ifstream::binary);
    if (!stream) {
        return -1;
    }

    stream.seekg(0, stream.end);
    size = stream.tellg();
    stream.seekg(0, stream.beg);

    *result = new char[size + 1];
    stream.read(*result, size);
    if (!stream) {
        return -2;
    }
    stream.close();
    (*result)[size] = 0;
    return size;
}

// ==============================================
// Helper Function: Reads an array of int values
// ==============================================
int *read_vector_file(const char *File_Name, int *Nb_Of_Elements) {
  int val;
  int *Array;
  int temp_Nb_Of_Elements;
  fstream DataIn_File;
  void *ptr=nullptr;


  DataIn_File.open(File_Name,ios::in);
  if (! DataIn_File.is_open()) {
      cout << endl << "HOST-Error: Failed to open the " <<  File_Name << " for read" << endl << endl;
      exit(1);
  }

  temp_Nb_Of_Elements = 0;
  while (DataIn_File >> val) temp_Nb_Of_Elements ++;
  DataIn_File.close();

  if (posix_memalign(&ptr,4096,temp_Nb_Of_Elements*sizeof(int))) {
	  cout << endl << "HOST-Error: Out of Memory during memory allocation for Array" << endl << endl;
      exit(1);
  }
  Array = reinterpret_cast<int*>(ptr);

  DataIn_File.open(File_Name,ios::in);
  if (! DataIn_File.is_open()) {
      cout << endl << "HOST-Error: Failed to open the " <<  File_Name << " fore read" << endl << endl;
      exit(1);
  }

  temp_Nb_Of_Elements = 0;
  while (DataIn_File >> Array[temp_Nb_Of_Elements])
      temp_Nb_Of_Elements ++;
  DataIn_File.close();

  *Nb_Of_Elements = temp_Nb_Of_Elements;

  return Array;
}

// ==================================================
// Helper Function: Generate an array of init values
//   Period - defines the number of unique values
//            for example, if Period=4 then we will
//            have the following values:
//            0 1 2 3 0 1 2 3
// ==================================================
void gen_int_values(int *Array, int Nb_Of_Elements, int Period) {
	int val;

	val = 0;
	for (int i=0; i<Nb_Of_Elements; i++) {
		Array[i] = val;
		val ++;
		if (val == Period) val = 0;
	}
}

// ==================================================
// Helper Function: Print table of integer values
// ==================================================
void print_int_table(int *Array, int Nb_Of_Elements, int Nb_Of_Columns, int Nb_Of_Char_Per_Column) {
	int col;

	col = 0;
	for (int i=0; i<Nb_Of_Elements; i++) {
		cout << setw(Nb_Of_Char_Per_Column) << Array[i];
		col ++;
		if (col == Nb_Of_Columns) {
			col = 0;
			cout << endl;
		}
	}

}

// ==================================================
// Application: Checks the output results
// ==================================================
#include "kernel.h"

bool check_output_results (int *DataIn_1, int *DataIn_2, int *DataIn_3, int CONST_arg, int *RES) {
	bool error_detected = false;
	int Max_Number_Of_Failers = 5;

	// KVConstAdd
	// .............
	int Gold_KVConstAdd[SIZE_DataIn_1];
	for (int i=0; i< SIZE_DataIn_1; i++) { Gold_KVConstAdd[i] = DataIn_1[i] + CONST_arg; }

	//	for (int i=0; i< 5; i++) cout << DataIn_1[i] << " " << Gold_KVConstAdd[i] << endl;
	//	exit(0);

	// KA
	// .............
	int Gold_KA[SIZE_BUF_KA];
	for (int i=0; i<SIZE_BUF_KA; i++) {
		Gold_KA[i] = Gold_KVConstAdd[2*i]*3 + Gold_KVConstAdd[2*i+1]*5 + Gold_KVConstAdd[2*i+2]*7 + Gold_KVConstAdd[2*i+3]*9;
	}


	// KpB
	// .............
	int Gold_KpB[SIZE_BUF_KpB];
	for (int i=0; i< SIZE_BUF_KpB; i++) { Gold_KpB[i] = (DataIn_2[i] + DataIn_3[i]) % 3; }

	//	for (int i=0; i< 5; i++) cout << DataIn_2[i] << " " << DataIn_3[i] << " " << Gold_KVAdd[i] << endl;
	//	exit(0);

	// KB
	// .............
	int Gold_KB[SIZE_BUF_KB];
	int TMP_BUF[SIZE_BUF_KpB];
	int val;

	for (int i=0; i<SIZE_BUF_KpB; i++) {
		TMP_BUF[i] = Gold_KpB[i]+10;
	}

	for (int i=0; i<SIZE_BUF_KB; i++) {
		val  =  TMP_BUF[i]*3 + TMP_BUF[i+SIZE_RES]*5 + TMP_BUF[i+2*SIZE_RES]*7;
		Gold_KB[i] = val;
	}


	// KCalc
	// .............
	int Gold_RES[SIZE_BUF_KB];
	int TMP_RES [SIZE_RES];
	int val1, val2;

	for (int i=0; i<SIZE_RES; i++) {
		val1 = (Gold_KA[i] - Gold_KB[i]) * (Gold_KA[i] + Gold_KB[i]);
		TMP_RES[i] = val1;
	}

	for (int i=0; i<SIZE_RES; i++) {
		if (TMP_RES[i] >= 0)
		    val2 = TMP_RES[i] % 3;
		else
			val2 = (TMP_RES[i] % 6) * TMP_RES[i];
		Gold_RES[i] = val2;
	}


	// Compare Results
	// .................
	cout << endl << "Host-Info: =============================================================" << endl;
	cout <<         "Host-Info: Verifying final results (only " << Max_Number_Of_Failers << " first failures are printed)" << endl;


	for (int i=0; i< SIZE_RES; i++) {
		if (RES[i] != Gold_RES[i]) {
			if (error_detected == false) {
				cout <<         "Host-Info: --------------------------------------------------------" << endl;
				cout <<         "Host-Info: " << setw(10) << "i" << setw(10) << "Expected" << setw(10) << "Actual" << setw(10) << "Status" << endl;
				cout <<         "Host-Info: --------------------------------------------------------" << endl;
			}
			if (Max_Number_Of_Failers-- > 0) {
				cout << "Host-Info: " << setw(10) << i << setw(10) << Gold_RES[i] << setw(10) << RES[i];
				cout << setw(10) << "Error" << endl;
				error_detected = true;
			}
		}
	}
	cout << "Host-Info: =============================================================" << endl;

	return (error_detected);
}

// ============================================================================
// Help Funciton: Custom Profiling
// ============================================================================

void run_custom_profiling (int Nb_Of_Kernels, int Nb_Of_Memory_Tranfers, cl_event* K_exe_event, cl_event* Mem_op_event,string* list_of_kernel_names) {
	typedef struct {
		string    action_type; // kernel, "memory (H->G)", "memory (G->H)"
		string    name;
		cl_event  event;
		cl_ulong  profiling_command_start;
		cl_ulong  profiling_command_end;
		double    duration;
	} profile_t;

	cl_int              errCode;

	// ---------------------------------
	// Profiling
	// ---------------------------------
	profile_t *PROFILE;

	PROFILE = new profile_t[Nb_Of_Kernels + Nb_Of_Memory_Tranfers];

	PROFILE[0].action_type="kernel";         PROFILE[0].name="K_KVConstAdd";  PROFILE[0].event = K_exe_event[0];
	PROFILE[1].action_type="kernel";         PROFILE[1].name="K_KpB";         PROFILE[1].event = K_exe_event[1];
	PROFILE[2].action_type="kernel";         PROFILE[2].name="K_KA";          PROFILE[2].event = K_exe_event[2];
	PROFILE[3].action_type="kernel";         PROFILE[3].name="K_KB";          PROFILE[3].event = K_exe_event[3];
	PROFILE[4].action_type="kernel";         PROFILE[4].name="K_KCalc";       PROFILE[4].event = K_exe_event[4];

	for (int i=0; i<Nb_Of_Memory_Tranfers; i++) {
		PROFILE[Nb_Of_Kernels+i].action_type="mem (H<->G)";
		PROFILE[Nb_Of_Kernels+i].name="Transfer_" + to_string(i+1);
		PROFILE[Nb_Of_Kernels+i].event = Mem_op_event[i];
	}

	// -------------------------------------------------------------------------------------
	// Get events Start and End times and calculate duration for
	//   o) each Kernel and
	//   o) Memory (Global <-> Host) transfer.
	// Event Profile Info:
	//   o) CL_PROFILING_COMMAND_QUEUED
	//   o) CL_PROFILING_COMMAND_SUBMIT
	//   o) CL_PROFILING_COMMAND_START
	//   o) CL_PROFILING_COMMAND_END
	// -------------------------------------------------------------------------------------
	size_t nb_of_returned_bytes;

	for (int i=0; i<Nb_Of_Kernels + Nb_Of_Memory_Tranfers; i++) {
		errCode = clGetEventProfilingInfo(PROFILE[i].event, CL_PROFILING_COMMAND_START,
										  sizeof(cl_ulong), &PROFILE[i].profiling_command_start, &nb_of_returned_bytes);
		if (errCode != CL_SUCCESS) {
			cout << endl << "HOST-Error: Failed to get profiling info for " << PROFILE[i].name << " " << PROFILE[i].action_type << endl << endl;
			exit(0);
		}

		errCode = clGetEventProfilingInfo(PROFILE[i].event, CL_PROFILING_COMMAND_END,
										  sizeof(cl_ulong), &PROFILE[i].profiling_command_end, &nb_of_returned_bytes);
		if (errCode != CL_SUCCESS) {
			cout << endl << "HOST-Error: Failed to get profiling info for " << PROFILE[i].name << " " << PROFILE[i].action_type << endl << endl;
			exit(0);
		}

		PROFILE[i].duration = (double)(PROFILE[i].profiling_command_end - PROFILE[i].profiling_command_start) * 1.0e-6;
	}

	// -------------------------------------------------------------------------------------
	// Calculate Duration of
	//   o) All kernels execution time
	//   o) Application execution time (Kernels + Memory transfer)
	// -------------------------------------------------------------------------------------
	struct {
		int      Kernels_Start_Time_Index=0;
		int      Kernels_End_Time_Index=0;
		cl_ulong Kernels_Start_Time=0;
		cl_ulong Kernels_End_Time=0;

		int      Application_Start_Time_Index=0;
		int      Application_End_Time_Index=0;
		cl_ulong Application_Start_Time=0;
		cl_ulong Application_End_Time=0;
	} PROFILE_STAT;


	for (int i=0; i<Nb_Of_Kernels + Nb_Of_Memory_Tranfers; i++) {

		// Calculate Application statistics
		// .................................
		if ((PROFILE_STAT.Application_Start_Time == 0) || (PROFILE_STAT.Application_Start_Time > PROFILE[i].profiling_command_start)) {
			PROFILE_STAT.Application_Start_Time       = PROFILE[i].profiling_command_start;
			PROFILE_STAT.Application_Start_Time_Index = i;
		}

		if (PROFILE_STAT.Application_End_Time < PROFILE[i].profiling_command_end) {
			PROFILE_STAT.Application_End_Time       = PROFILE[i].profiling_command_end;
			PROFILE_STAT.Application_End_Time_Index = i;
		}

		// Calculate Kernel statistics
		// .................................
		if (PROFILE[i].action_type == "kernel") {
			if ((PROFILE_STAT.Kernels_Start_Time == 0) || (PROFILE_STAT.Kernels_Start_Time > PROFILE[i].profiling_command_start)) {
				PROFILE_STAT.Kernels_Start_Time       = PROFILE[i].profiling_command_start;
				PROFILE_STAT.Kernels_Start_Time_Index = i;
			}

			if (PROFILE_STAT.Kernels_End_Time < PROFILE[i].profiling_command_end) {
				PROFILE_STAT.Kernels_End_Time       = PROFILE[i].profiling_command_end;
				PROFILE_STAT.Kernels_End_Time_Index = i;
			}
		}
	}

	// ------------------------------
	// Print Profiling Data
	// ------------------------------
	int Column_Widths[5] = {15, 16, 15, 15, 15}, Separation_Line_Width = 0;

	// Print Table Header
	// ....................
	for (int i=0; i<5; i++)  Separation_Line_Width += Column_Widths[i];
	Separation_Line_Width += 3;
	cout << "HOST-Info: " << string(Separation_Line_Width, '-') << endl;

	cout << "HOST-Info: "          << left << setw(Column_Widths[0]-1) << "Name"
						  << " | " << left << setw(Column_Widths[1]-3) << "type"
						  << " | " << left << setw(Column_Widths[2]-3) << "start"
						  << " | " << left << setw(Column_Widths[2]-3) << "end"
						  << " | " << left << setw(Column_Widths[2]-3) << "Duration(ms)"
						  << endl;

	cout << "HOST-Info: " << string(Separation_Line_Width, '-') << endl;


	// Print Individual info for each Kernel and Memory transfer
	// ..........................................................
	for (int i=0; i<Nb_Of_Kernels + Nb_Of_Memory_Tranfers; i++) {
		cout << "HOST-Info: "          << left  << setw(Column_Widths[0]-1) << PROFILE[i].name
							  << " | " << left  << setw(Column_Widths[1]-3) << PROFILE[i].action_type
							  << " | " << right << setw(Column_Widths[2]-3) << PROFILE[i].profiling_command_start
							  << " | " << right << setw(Column_Widths[2]-3) << PROFILE[i].profiling_command_end
							  << " | " << right << setw(Column_Widths[2]-3) << PROFILE[i].duration
							  << endl;
	}
	cout << "HOST-Info: " << string(Separation_Line_Width, '-') << endl;

	// Print Duration of Kernels and Application execution
	// ..........................................................
	cout << "HOST-Info:     Kernels Execution Time (ms) :  "
			<< (double) (PROFILE_STAT.Kernels_End_Time - PROFILE_STAT.Kernels_Start_Time) * 0.000001 //1.0e-6
			<< "  (" << PROFILE[PROFILE_STAT.Kernels_End_Time_Index].name << "\'end - " << PROFILE[PROFILE_STAT.Kernels_Start_Time_Index].name << "\'begin)"
			<< endl;

	cout << "HOST-Info: Application Execution Time (ms) :  "
			<< (double) (PROFILE_STAT.Application_End_Time - PROFILE_STAT.Application_Start_Time) * 0.000001 //1.0e-6
			<< "  (" << PROFILE[PROFILE_STAT.Application_End_Time_Index].name << "\'end - " << PROFILE[PROFILE_STAT.Application_Start_Time_Index].name << "\'begin)"
			<< endl;

	cout << "HOST-Info: " << string(Separation_Line_Width, '-') << endl << endl;

}

