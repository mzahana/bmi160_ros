#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "bmi160_ros/bmi160_interface.h"

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
            imu_msg.linear_acceleration.x = convertRawAccelToMs2(acc_data.x, accel_range_);
            imu_msg.linear_acceleration.y = convertRawAccelToMs2(acc_data.y, accel_range_);
            imu_msg.linear_acceleration.z = convertRawAccelToMs2(acc_data.z, accel_range_);

            imu_msg.angular_velocity.x = convertRawGyroToRads(gyro_data.x);
            imu_msg.angular_velocity.y = convertRawGyroToRads(gyro_data.y);
            imu_msg.angular_velocity.z = convertRawGyroToRads(gyro_data.z);

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
        double multiplier = range_multiplier / std::pow(2.0, 16) * 9.81;
        return raw_data * multiplier;
    }

    double convertRawGyroToRads(int16_t raw_data) {
        return raw_data * 0.0174533; // Convert dps to rad/s
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bmi160_publisher");
    BMI160Publisher bmi160_publisher;
    bmi160_publisher.spin();
    return 0;
}
