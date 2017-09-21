
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

    ErrorCode code;
    cv::Mat img_left, img_right;
    char filename[20]={0};
    string s1 = argv[2];  //left image path
    string s2 = argv[3];  //right image path
    int n = stoi(argv[4]);      //image number
    string s3 = ".png";
    string s4;
    string img0fileName;
    string img1fileName;
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
                for(; i < 3000; ++i) {
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

                        if(i%5 == 0)
                        {
                            cam.RetrieveIMUData(imudatas,timestamp);
                            sprintf(filename,"%014d",timestamp);
                            s4 = filename;
                            string s5 = "00000";
                            img0fileName = s1 + s4 + s5 + s3;
                            img1fileName = s2 + s4 + s5 + s3;
                            cv::imwrite(img0fileName.c_str(),img_left);
                            cv::imwrite(img1fileName.c_str(),img_right);
                        }
                    }
                    cout << "i" <<i<<endl;
                }
                if(i == 3000)
                {
                    cout << "ok" << endl;
                    break;
                }
            }


        }
    }

    cam.Close();
    cv::destroyAllWindows();

    return 0;
}














