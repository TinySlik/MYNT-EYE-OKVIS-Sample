# kalibr calibration

## How to install kalibr

1. DOWNLOAD：https://github.com/ethz-asl/kalibr/wiki/downloads
2. INSTALL：https://github.com/ethz-asl/kalibr/wiki/installation

## How to use kalibr to calibrate camera and imu

1. Get dataset of stereo camera

    ```
    $ cd mynt-eye-okvis-sample/build
    $ ./okvis_app_getcameracalibdataset 1 ./camera/cam0/ ./camera/cam1/ 30
    ```

    `1` means that it use video1, `./camera/cam0/` is the folder to store left images, `./camera/cam0/` is the folder to store right images, `30` means that it capture 30 photos.

2. Use kalibrbagcreate creat calibration bag of stereo camera.
First move the folder of camera to kalibr's workspace, then creat calibration bag.

    ```
    $ kalibr_bagcreater --folder camera/. --output-bag cameracalib.bag
    ```

3. Calibrate stereo camera

    ```
    $ kalibr_calibrate_cameras --target april_6x6_50x50cm.yaml --bag cameracalib.bag --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw
    ```

    `april_6x6_50x50cm.yaml` is the config file of calibration target which can download from the [website](https://github.com/ethz-asl/kalibr/wiki/downloads).

    It will produce a file named 'camchain-homehlkalibr_workspacecameracalib.yaml' which store the result.

4. Get dataset of stereo camera and IMU.

    ```
    $ cd mynt-eye-okvis-sample/build
    $ ./okvis_app_getcameraimucalibdataset 1 ./cameraimu/cam0/ ./cameraimu/cam1/ ./cameraimu/imu0.csv
    ```

    `1` means that it use video1, `./cameraimu/cam0` is the folder to store left images, `./camera/cam0` is the folder to store right images, `./cameraimu/imu0.csv` is the file to store the data of imu.

5. Use kalibrbagcreate creat calibration bag of stereo camera and imu.

    First move the folder of camera to kalibr's workspace, then creat calibration bag.

    ```
    $ kalibr_bagcreater --folder cameraimu/. --output-bag cameraimucalib.bag
    ```

6. Camera stereo camera and imu.

    ```
    $ kalibr_calibrate_imu_camera --target april_6x6_50x50cm.yaml --cam camchain-homehlkalibr_workspacecameracalib.yaml --imu imu.ymal --bag cameraimucalib.bag --bag-from-to 5 45
    ```

    `april_6x6_50x50cm.yaml` is the config file of calibration target which can download from the [website] (https://github.com/ethz-asl/kalibr/wiki/downloads), `camchain-homehlkalibr_workspacecameracalib.yaml` is the config file of stereo camera，`imu.ymal` is config file of imu, `from-to 5 45` means that it use bag from 5s to 45s to calibration.

    It will produce a file named 'camchain-imucam-homehlkalibr_workspacecameraimucalib.yaml' which store the result.

7. Change the result's format which can be used by okvis directly.

    ```
    $ kalibr_aslam_config --cam camchain-imucam-homehlkalibr_workspacecameraimucalib.yaml
    ```

    then the config data will appear in terminal.
