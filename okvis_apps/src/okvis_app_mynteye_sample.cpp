/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jun 26, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file okvis_app_mynteye_sample.cpp

 This node goes through mynteye

 * @author MYNTEYE
 */

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

#pragma GCC diagnostic pop


#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <okvis/VioParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>
#include "camera.h"
#include "utility.h"

using namespace std;
using namespace mynteye;

class PoseViewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    constexpr static const double imageSize = 955.0;
    PoseViewer()
    {
        cv::namedWindow("MYNTEYE Top View");
        _image.create(imageSize, imageSize, CV_8UC3);
        drawing_ = false;
        showing_ = false;
    }
    // this we can register as a callback
    void publishFullStateAsCallback(
            const okvis::Time & /*t*/, const okvis::kinematics::Transformation & T_WS,
            const Eigen::Matrix<double, 9, 1> & speedAndBiases,
            const Eigen::Matrix<double, 3, 1> & /*omega_S*/)
    {

        // just append the path
        Eigen::Vector3d r = T_WS.r();
        Eigen::Matrix3d C = T_WS.C();
        double r_x = r[1]*(-1);
        double r_y = r[0];
        double r_z = r[2];
        _path.push_back(cv::Point2d(r_x, r_y));
        _heights.push_back(r[2]);
        // maintain scaling
        if (r_x - _frameScale < _min_x)
            _min_x = r_x - _frameScale;
        if (r_y - _frameScale < _min_y)
            _min_y = r_y - _frameScale;
        if (r_y < _min_z)
            _min_z = r_z;
        if (r_x + _frameScale > _max_x)
            _max_x = r_x + _frameScale;
        if (r_y + _frameScale > _max_y)
            _max_y = r_y + _frameScale;
        if (r_z > _max_z)
            _max_z = r_z;
        _scale = std::min(imageSize / (_max_x - _min_x), imageSize / (_max_y - _min_y));

        // draw it
        while (showing_) {
        }
        drawing_ = true;
        // erase
        _image.setTo(cv::Scalar(10, 10, 10));
        drawPath();
        // draw axes
        Eigen::Vector3d e_x = C.col(1);
        Eigen::Vector3d e_y = C.col(0);
        Eigen::Vector3d e_z = C.col(2);
        cv::line(
                    _image,
                    convertToImageCoordinates(_path.back()),
                    convertToImageCoordinates(
                        _path.back() + cv::Point2d(e_x[0], e_x[1]) * _frameScale),
                cv::Scalar(0, 0, 255), 1, CV_AA);
        cv::line(
                    _image,
                    convertToImageCoordinates(_path.back()),
                    convertToImageCoordinates(
                        _path.back() + cv::Point2d(e_y[0], e_y[1]) * _frameScale),
                cv::Scalar(0, 255, 0), 1, CV_AA);
        cv::line(
                    _image,
                    convertToImageCoordinates(_path.back()),
                    convertToImageCoordinates(
                        _path.back() + cv::Point2d(e_z[0], e_z[1]) * _frameScale),
                cv::Scalar(255, 0, 0), 1, CV_AA);

        // some text:
        std::stringstream postext;

        postext << "position = [" << r_x << "," << r_y << ", " << r_z << "]";
        cv::putText(_image, postext.str(), cv::Point(15,25),
                    cv::FONT_HERSHEY_COMPLEX, 0.9, cv::Scalar(255,255,255), 1);
        std::stringstream veltext;
        veltext << "velocity = [" << speedAndBiases[1] << ", " << speedAndBiases[0] << ", " << speedAndBiases[2] << "]";
        cv::putText(_image, veltext.str(), cv::Point(15,60),
                    cv::FONT_HERSHEY_COMPLEX, 0.9, cv::Scalar(255,255,255), 1);
        drawing_ = false; // notify
    }
    void display()
    {
        while (drawing_) {
        }
        showing_ = true;
        cv::imshow("MYNTEYE Top View", _image);
        showing_ = false;
        cv::waitKey(1);
    }
private:
    cv::Point2d convertToImageCoordinates(const cv::Point2d & pointInMeters) const
    {
        cv::Point2d pt = (pointInMeters - cv::Point2d(_min_x, _min_y)) * _scale;
        return cv::Point2d(pt.x, imageSize - pt.y); // reverse y for more intuitive top-down plot
    }
    void drawPath()
    {
        for (size_t i = 0; i + 1 < _path.size(); ) {
            cv::Point2d p0 = convertToImageCoordinates(_path[i]);
            cv::Point2d p1 = convertToImageCoordinates(_path[i + 1]);
            cv::Point2d diff = p1-p0;

            if(diff.dot(diff)<2.0){
                _path.erase(_path.begin() + i + 1);  // clean short segment
                _heights.erase(_heights.begin() + i + 1);
                continue;
            }

            double rel_height = (_heights[i] - _min_z + _heights[i + 1] - _min_z)
                    * 0.5 / (_max_z - _min_z);
            cv::line(
                        _image,
                        p0,
                        p1,
                        rel_height * cv::Scalar(125, 0, 125)
                        + (1.0 - rel_height) * cv::Scalar(0, 0, 255),
                        1, CV_AA);
            i++;
        }
    }
    cv::Mat _image;
    std::vector<cv::Point2d> _path;
    std::vector<double> _heights;
    double _scale = 1.0;
    double _min_x = -0.5;
    double _min_y = -0.5;
    double _min_z = -0.5;
    double _max_x = 0.5;
    double _max_y = 0.5;
    double _max_z = 0.5;
    const double _frameScale = 0.2;  // [m]
    std::atomic_bool drawing_;
    std::atomic_bool showing_;
};



int main(int argc, char **argv)
{

    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    FLAGS_colorlogtostderr = 1;

    okvis::Duration deltaT(0.0);
    if (argc == 4) {
        deltaT = okvis::Duration(atof(argv[3]));
    }

    // read configuration file
    std::string configFilename(argv[1]);

    okvis::VioParametersReader vio_parameters_reader(configFilename);
    okvis::VioParameters parameters;
    vio_parameters_reader.getParameters(parameters);

    okvis::ThreadedKFVio okvis_estimator(parameters);

    PoseViewer poseViewer;
    okvis_estimator.setFullStateCallback(
                std::bind(&PoseViewer::publishFullStateAsCallback, &poseViewer,
                          std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3, std::placeholders::_4));

    okvis_estimator.setBlocking(true);


    cout << "Open Camera: "<< endl;
    stringstream ss;
    ss << argv[2];
    mynteye::Camera cam;
    InitParameters params(ss.str());
    cout << "Open Camera:1 " << endl;

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
    string image_seconds;
    string image_nonseconds;
    string imu_seconds;
    string imu_nonseconds;

    string image_time;
    string imu_time;
    unsigned long long int imu_time_;


    okvis::Time t_imu;
    okvis::Time t_image;
    IMUData imudata;
    vector<IMUData> imudatas;
    std::uint32_t timestamp;

    int count=0;
    for(;;)
    {
        code = cam.Grab();

        if (code != ErrorCode::SUCCESS) {
            std::cout << "Warning: Grab failed <" << code << ">" << std::endl;
            continue;
        }
        cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED);
        cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED);
        cam.RetrieveIMUData(imudatas,timestamp);
        sprintf(filename,"%014d",timestamp);
        string s5 = "00000";
        image_time = filename + s5;
        image_seconds = image_time.substr(0,image_time.size()-9);
        image_nonseconds = image_time.substr(image_time.size()-9,9);


        okvis_estimator.display();
        poseViewer.display();

        size_t size = imudatas.size();

        t_image = okvis::Time(std::stoi(image_seconds),std::stoi(image_nonseconds));

        if(count > 20)
        {
            for(int k= 0;k<size;k++)
            {
                imudata = imudatas[k];
                imu_time_ = timestamp + imudata.time_offset;
                sprintf(filename,"%014llu",imu_time_);
                imu_time = filename + s5;
                imu_seconds = imu_time.substr(0,imu_time.size()-9);
                imu_nonseconds = imu_time.substr(imu_time.size()-9,9);

                Eigen::Vector3d gyr;
                gyr[0] = imudata.gyro_x/57.2956;
                gyr[1] = imudata.gyro_y/57.2956;
                gyr[2] = imudata.gyro_z/57.2956;

                Eigen::Vector3d acc;
                acc[0] = imudata.accel_x*parameters.imu.g;
                acc[1] = imudata.accel_y*parameters.imu.g;
                acc[2] = imudata.accel_z*parameters.imu.g;

                //add the IMU measurement for (blocking) processing;
                t_imu = okvis::Time(std::stoi(imu_seconds),std::stoi(imu_nonseconds));
                okvis_estimator.addImuMeasurement(t_imu, acc, gyr);
            }

            //add the IMAGE measurement for (blocking) processing;
            if(count%3==0)
            {
                okvis_estimator.addImage(t_image, 0, img_left);
                okvis_estimator.addImage(t_image, 1, img_right);
            }
        }
        count++;
    }

    cam.Close();
    cv::destroyAllWindows();
    return 0;
}














