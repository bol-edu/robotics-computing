#include "Dataset_Handler.h"

Dataset_Handler::Dataset_Handler(int sequence)
{
    Methods method;

    seq = sequence;

    seq_dir = new char[200];
    sprintf_s(seq_dir, 200, "../dataset/sequences/%02d/", seq);

    poses_dir = new char[200];
    sprintf_s(poses_dir, 200, "../dataset/poses/%02d.txt", seq);

    poses = method.groundTruthTrajectory(poses_dir, 4541);

    read_data();
    cv::Mat first_image_left = cv::imread(left_image_files[0], cv::IMREAD_GRAYSCALE);
    imwidth = first_image_left.cols;
    imheight = first_image_left.rows;

    char *calib_dir = new char[200];
    sprintf_s(calib_dir, 200, "../dataset/sequences/%02d/calib.txt", seq);
    method.read_calib((const char *)calib_dir, &P0, &P1);
}

void Dataset_Handler::read_data()
{
    // folder pinter
    WIN32_FIND_DATA ffd;
    // folder path
    TCHAR szDir[MAX_PATH];
    _sntprintf_s(szDir, MAX_PATH, TEXT("../dataset/sequences/%02d/image_0/*"), seq);
    // find folder
    HANDLE hFind = INVALID_HANDLE_VALUE;

    hFind = FindFirstFile(szDir, &ffd);

    if (INVALID_HANDLE_VALUE == hFind)
    {
        // pirint folder is broken
        cout << "Folder \"data\" is gone\n";
    }

    // to upper level folder
    TCHAR parent[] = _T("..");
    // this level folder
    TCHAR child[] = _T(".");

    int left_image_num = 0;
    do
    {
        // count how many folders
        if (_tcscmp(ffd.cFileName, parent) && _tcscmp(ffd.cFileName, child) && ffd.dwFileAttributes && FILE_ATTRIBUTE_DIRECTORY)
        {
            char *image_name = new char[1000];
            sprintf_s(image_name, 1000, "../dataset/sequences/00/image_0/%ws", ffd.cFileName);
            // sprintf_s(image_name, 1000, "../dataset/sequences/00/image_0/%s", image_name);
            left_image_files.push_back(image_name);
            left_image_num++;
        }

    } while (FindNextFile(hFind, &ffd) != 0);

    TCHAR szDir1[MAX_PATH];
    _sntprintf_s(szDir1, MAX_PATH, TEXT("../dataset/sequences/%02d/image_1/*"), seq);
    // find folder
    hFind = INVALID_HANDLE_VALUE;

    hFind = FindFirstFile(szDir1, &ffd);

    if (INVALID_HANDLE_VALUE == hFind)
    {
        cout << "Folder \"data\" is gone\n";
    }

    int right_image_num = 0;
    do
    {
        // count how many folders
        if (_tcscmp(ffd.cFileName, parent) && _tcscmp(ffd.cFileName, child) && ffd.dwFileAttributes && FILE_ATTRIBUTE_DIRECTORY)
        {
            char *image_name = new char[1000];
            sprintf_s(image_name, 1000, "../dataset/sequences/00/image_1/%ws", ffd.cFileName);
            // sprintf_s(image_name, 1000, "../dataset/sequences/00/image_1/%s", image_name);
            right_image_files.push_back(image_name);
            right_image_num++;
        }

    } while (FindNextFile(hFind, &ffd) != 0);

    if (left_image_num != right_image_num)
        printf("(Warning) Numbers of left and right images doesn't match\n\n");
    num_frames = min(left_image_num, right_image_num);
    printf("num_frames: %d\n\n", num_frames);

    FindClose(hFind);
}