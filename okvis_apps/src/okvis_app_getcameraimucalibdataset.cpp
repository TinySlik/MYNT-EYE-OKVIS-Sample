#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>
#include <stdio.h>
#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <okvis/VioParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>

#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>

#include "camera.h"
#include "utility.h"
using namespace std;
using namespace mynteye;

int main(int argc, char **argv)
{
    cout << "Open Camera: "<< endl;
    stringstream ss;
    ss << argv[1];
    mynteye::Camera cam;
    InitParameters params(ss.str());

    cam.Open(params);
    cout << "Open Camera: "  << endl;
    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    std::cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m\n";

    // cam.ActivateAsyncGrabFeature();

    ErrorCode code;
    cv::Mat img_left, img_right;
    char filename[20]={0};
    ofstream outFile;
    outFile.open(argv[4],ios::out | ios::trunc);
    outFile << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z" <<"\n";
    string s1 = argv[2];
    string s2 = argv[3];
    string s3 = ".png";
    string s4;
    string img0fileName;
    string img1fileName;
    string imu_time;
    unsigned long long int imu_time_;

    IMUData imudata;
    vector<IMUData> imudatas;
    std::uint32_t timestamp;
    int i = 0;



    for(;;) {
        code = cam.Grab();

        if (code != ErrorCode::SUCCESS) {
            continue;
        }

        if (cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED) == ErrorCode::SUCCESS &&
                cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED) == ErrorCode::SUCCESS) {

            cv::imshow("left", img_left);
            cv::imshow("right", img_right);

            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
                break;

            }
            cout << "please press space key to start capture data" << endl;

            if(key == 32)
            {
                cout << "capturing ..." << endl;
                for(; i < 1800; ++i) {
                    code = cam.Grab();

                    if (code != ErrorCode::SUCCESS) {
                        continue;
                    }

                    if (cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED) == ErrorCode::SUCCESS &&
                            cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED) == ErrorCode::SUCCESS) {

                        cv::imshow("left", img_left);
                        cv::imshow("right", img_right);
                        int key = cv::waitKey(1);
                        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
                            break;

                        }
                        cam.RetrieveIMUData(imudatas,timestamp);
                        sprintf(filename,"%014d",timestamp);
                        s4 = filename;
                        string s5 = "00000";
                        img0fileName = s1 + s4 +s5+ s3;
                        img1fileName = s2 + s4 +s5+ s3;
                        size_t size = imudatas.size();

                        cv::imwrite(img0fileName.c_str(),img_left);
                        cv::imwrite(img1fileName.c_str(),img_right);

                        for(int k= 0;k<size;k++)
                        {
                            imudata = imudatas[k];
                            imu_time_ = timestamp + imudata.time_offset;
                            sprintf(filename,"%014llu",imu_time_);
                            imu_time = filename + s5;
                            outFile << imu_time <<','<< (imudata.gyro_x/57.2956) << ',' << (imudata.gyro_y/57.2956) << ',' << (imudata.gyro_z/57.2956) <<','<<(imudata.accel_x*9.7959) << ',' << (imudata.accel_y*9.7959) << ',' << (imudata.accel_z*9.7959)<<"\n";
                        }
                    }
                    cout << "i" <<i<<endl;
                }
            }
        }
        if(i == 1800)
        {
            cout<<"ok"<<endl;
            break;
        }
    }

    cam.Close();
    cv::destroyAllWindows();

    return 0;

}














