# bmi160_ros
A ROS driver for the BMI160 IMU

The implementation of this driver is based on the implementation in [BOSCH-BMI160-ROS-Library(https://github.com/berktepebag/BOSCH-BMI160-ROS-Library/tree/master)

# Features

* Provides configurable sensor parameters through a YAML file, [config/bmi160_params.yaml](config/bmi160_params.yaml)
* Provides a calibration node with 6-side accelerometer calibration. Example calibration ouptut is in [config/bmi160_calibration.yaml](config/bmi160_calibration.yaml)

# Dependencies
* Eigen `sudo apt-get install libeigen3-dev`
* yaml-cpp `sudo apt-get install -y libyaml-cpp-dev`

# Build

* Install the dependecies
* Clone this package to your workspace, e.g. `catkin_ws/src`
* Build the package `catkin build bmi160_ros`
* Source your workspace `source devel/setup.bash`

# Calibration

* It is better to calibrate the sensor before using it. To run the calibration node
    ```sh
    roslaunch bmi160_ros bmi160_calibration.launch
    ```

* The calibration node should instruct you to place the sensor on different sides (6 sides for accelerometer, and one for gyroscope) to collect measurements and performe least square fitting and compute sensor gains and offsets.

* The calibration parameters are saved in the output file defined in the [bmi160_calibration.launch](launch/bmi160_calibration.launch). The default is [config/bmi160_params.yaml](config/bmi160_params.yaml).

# Run the driver
To run the driver simply launch its node
```sh
roslaunch bmi160_calibration.launch
```
This will publish the `/imu/data_raw` topic. The measurements are corrected using the calibration parameters.
