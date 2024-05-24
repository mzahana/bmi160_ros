#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include "bmi160_ros/bmi160_interface.h"
#include <eigen3/Eigen/Dense>

static const float gravity_value = 9.81;
static const float deg_to_rad_factor = M_PI / 180.0;

class BMI160Publisher {
public:
    BMI160Publisher() : prev_sensor_time_(0) {
        ros::NodeHandle nh("~");
        nh.getParam("publish_rate", publish_rate_);
        nh.getParam("accel_odr", accel_odr_);
        nh.getParam("accel_range", accel_range_);
        nh.getParam("accel_bw", accel_bw_);
        nh.getParam("gyro_odr", gyro_odr_);
        nh.getParam("gyro_range", gyro_range_);
        nh.getParam("gyro_bw", gyro_bw_);
        nh.getParam("imu_frame_id", imu_frame_id_);

        nh.param("Xgain", A_(0, 0), 1.0);
        nh.param("YtoX", A_(0, 1), 0.0);
        nh.param("ZtoX", A_(0, 2), 0.0);
        nh.param("XtoY", A_(1, 0), 0.0);
        nh.param("Ygain", A_(1, 1), 1.0);
        nh.param("ZtoY", A_(1, 2), 0.0);
        nh.param("XtoZ", A_(2, 0), 0.0);
        nh.param("YtoZ", A_(2, 1), 0.0);
        nh.param("Zgain", A_(2, 2), 1.0);

        nh.param("Xofs", B_(0), 0.0);
        nh.param("Yofs", B_(1), 0.0);
        nh.param("Zofs", B_(2), 0.0);

        nh.param("gyro_x_offset", gyro_offsets_(0), 0.0);
        nh.param("gyro_y_offset", gyro_offsets_(1), 0.0);
        nh.param("gyro_z_offset", gyro_offsets_(2), 0.0);

        if (!computeInverseCalibrationMatrix()) {
            ROS_ERROR("Calibration matrix is not invertible. Please recalibrate the sensor.");
            ros::shutdown();
        }

        printConfiguration();

        imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 10);

        if (!initializeIMU()) {
            ROS_ERROR("Failed to initialize BMI160");
            ros::shutdown();
        }
    }

    void spin() {
        ros::Rate rate(publish_rate_);
        while (ros::ok()) {
            publishIMUData();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Publisher imu_pub_;
    double publish_rate_;
    int accel_odr_, accel_range_, accel_bw_, gyro_odr_, gyro_range_, gyro_bw_;
    std::string imu_frame_id_;
    BMI160 imu_;
    bmi160_dev sensor_;
    uint32_t prev_sensor_time_;
    ros::Time last_ros_time_;

    Eigen::Matrix3d A_;  // Calibration matrix
    Eigen::Vector3d B_;  // Offset vector
    Eigen::Vector3d gyro_offsets_;  // Gyroscope offsets

    bool computeInverseCalibrationMatrix() {
        try {
            A_ = A_.inverse();
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to invert calibration matrix: %s", e.what());
            return false;
        }
    }

    void printConfiguration() {
        ROS_INFO("Sensor configuration:");
        ROS_INFO("  Publish rate: %f Hz", publish_rate_);
        ROS_INFO("  Accelerometer ODR: 0x%02X", accel_odr_);
        ROS_INFO("  Accelerometer range: 0x%02X", accel_range_);
        ROS_INFO("  Accelerometer bandwidth: 0x%02X", accel_bw_);
        ROS_INFO("  Gyroscope ODR: 0x%02X", gyro_odr_);
        ROS_INFO("  Gyroscope range: 0x%02X", gyro_range_);
        ROS_INFO("  Gyroscope bandwidth: 0x%02X", gyro_bw_);
        ROS_INFO("  IMU frame ID: %s", imu_frame_id_.c_str());
    }

    bool initializeIMU() {
        int8_t rslt = BMI160_OK;
        sensor_ = imu_.initialize(rslt);
        if (rslt != BMI160_OK) {
            return false;
        }

        sensor_.accel_cfg.odr = static_cast<uint8_t>(accel_odr_);
        sensor_.accel_cfg.range = static_cast<uint8_t>(accel_range_);
        sensor_.accel_cfg.bw = static_cast<uint8_t>(accel_bw_);
        sensor_.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

        sensor_.gyro_cfg.odr = static_cast<uint8_t>(gyro_odr_);
        sensor_.gyro_cfg.range = static_cast<uint8_t>(gyro_range_);
        sensor_.gyro_cfg.bw = static_cast<uint8_t>(gyro_bw_);
        sensor_.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

        rslt = bmi160_set_sens_conf(&sensor_);
        return (rslt == BMI160_OK);
    }

    void publishIMUData() {
        sensor_msgs::Imu imu_msg;
        bmi160_sensor_data acc_data;
        bmi160_sensor_data gyro_data;

        if (bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL, &acc_data, &gyro_data, &sensor_) == BMI160_OK) {
            if (acc_data.sensortime != prev_sensor_time_) {
                prev_sensor_time_ = acc_data.sensortime;
                last_ros_time_ = ros::Time::now();
            } else {
                ROS_WARN("Sensor time has not been updated.");
            }

            imu_msg.header.stamp = last_ros_time_;
            imu_msg.header.frame_id = imu_frame_id_;

            Eigen::Vector3d raw_acc_g(-1.0*convertRawAccelToG(acc_data.x, accel_range_),
                                    -1.0*convertRawAccelToG(acc_data.y, accel_range_),
                                    convertRawAccelToG(acc_data.z, accel_range_));
            Eigen::Vector3d corrected_acc = A_ * (raw_acc_g - B_);

            imu_msg.linear_acceleration.x = corrected_acc.x() * gravity_value; // m/s/s
            imu_msg.linear_acceleration.y = corrected_acc.y() * gravity_value;
            imu_msg.linear_acceleration.z = corrected_acc.z() * gravity_value;

            Eigen::Vector3d raw_gyro(-1.0 * gyro_data.x, -1.0 * gyro_data.y, gyro_data.z);
            Eigen::Vector3d corrected_gyro = raw_gyro - gyro_offsets_;

            imu_msg.angular_velocity.x = convertRawGyroToRads(corrected_gyro.x()); // rad/s
            imu_msg.angular_velocity.y = convertRawGyroToRads(corrected_gyro.y());
            imu_msg.angular_velocity.z = convertRawGyroToRads(corrected_gyro.z());

            imu_pub_.publish(imu_msg);
        } else {
            ROS_WARN("Failed to read sensor data");
        }
    }

    double convertRawAccelToMs2(int16_t raw_data, uint8_t range) {
        double range_multiplier = 0.0;
        switch (range) {
            case BMI160_ACCEL_RANGE_2G:
                range_multiplier = 4.0;
                break;
            case BMI160_ACCEL_RANGE_4G:
                range_multiplier = 8.0;
                break;
            case BMI160_ACCEL_RANGE_8G:
                range_multiplier = 16.0;
                break;
            case BMI160_ACCEL_RANGE_16G:
                range_multiplier = 32.0;
                break;
            default:
                ROS_WARN("Unknown accelerometer range, defaulting to 2G");
                range_multiplier = 4.0;
                break;
        }
        double multiplier = range_multiplier / std::pow(2.0, 16) * gravity_value;
        return raw_data * multiplier;
    }

    double convertRawAccelToG(int16_t raw_data, uint8_t range) {
        double range_multiplier = 0.0;
        switch (range) {
            case BMI160_ACCEL_RANGE_2G:
                range_multiplier = 4.0;
                break;
            case BMI160_ACCEL_RANGE_4G:
                range_multiplier = 8.0;
                break;
            case BMI160_ACCEL_RANGE_8G:
                range_multiplier = 16.0;
                break;
            case BMI160_ACCEL_RANGE_16G:
                range_multiplier = 32.0;
                break;
            default:
                ROS_WARN("Unknown accelerometer range, defaulting to 2G");
                range_multiplier = 4.0;
                break;
        }
        double multiplier = range_multiplier / std::pow(2.0, 16);
        return raw_data * multiplier;
    }

    double convertRawGyroToRads(int16_t raw_data) {
        return raw_data * deg_to_rad_factor; // Convert dps to rad/s
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bmi160_publisher");
    BMI160Publisher bmi160_publisher;
    bmi160_publisher.spin();
    return 0;
}
