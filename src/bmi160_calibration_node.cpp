#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "bmi160_ros/bmi160_interface.h"

class SensorCalibration {
public:
    SensorCalibration() {}

    void addMeasurement(const Eigen::Vector3d& measured_acc, const Eigen::Vector3d& true_acc) {
        measured_accs_.push_back(measured_acc);
        true_accs_.push_back(true_acc);
    }

    void addGyroMeasurement(const Eigen::Vector3d& measured_gyro) {
        measured_gyros_.push_back(measured_gyro);
    }

    void calibrate() {
        int N = measured_accs_.size();
        Eigen::MatrixXd Acc(3, N);
        Eigen::MatrixXd TrueAcc(4, N);

        for (int i = 0; i < N; ++i) {
            Acc.col(i) = measured_accs_[i];
            TrueAcc.col(i) << true_accs_[i], 1.0;
        }

        Eigen::MatrixXd TrueAccT = TrueAcc.transpose();
        Eigen::MatrixXd product = Acc * TrueAccT;
        Eigen::MatrixXd inv_product = (TrueAcc * TrueAccT).inverse();

        Eigen::MatrixXd Unknowns = product * inv_product;

        // Extract the offsets, gains, and cross-gains
        Xgain_ = Unknowns(0, 0);
        YtoX_ = Unknowns(0, 1);
        ZtoX_ = Unknowns(0, 2);
        Xofs_ = Unknowns(0, 3);

        XtoY_ = Unknowns(1, 0);
        Ygain_ = Unknowns(1, 1);
        ZtoY_ = Unknowns(1, 2);
        Yofs_ = Unknowns(1, 3);

        XtoZ_ = Unknowns(2, 0);
        YtoZ_ = Unknowns(2, 1);
        Zgain_ = Unknowns(2, 2);
        Zofs_ = Unknowns(2, 3);

        A_ << Xgain_, YtoX_, ZtoX_,
              XtoY_, Ygain_, ZtoY_,
              XtoZ_, YtoZ_, Zgain_;

        B_ << Xofs_, Yofs_, Zofs_;

        // Compute gyro offsets
        gyro_offsets_ = Eigen::Vector3d::Zero();
        for (const auto& gyro : measured_gyros_) {
            gyro_offsets_ += gyro;
        }
        gyro_offsets_ /= measured_gyros_.size();
    }

    void writeCalibrationToYAML(const std::string& filename) const {
        YAML::Emitter out;
        out << YAML::BeginMap;

        out << YAML::Key << "Xgain" << YAML::Value << Xgain_;
        out << YAML::Key << "YtoX" << YAML::Value << YtoX_;
        out << YAML::Key << "ZtoX" << YAML::Value << ZtoX_;
        out << YAML::Key << "Xofs" << YAML::Value << Xofs_;
        out << YAML::Key << "XtoY" << YAML::Value << XtoY_;
        out << YAML::Key << "Ygain" << YAML::Value << Ygain_;
        out << YAML::Key << "ZtoY" << YAML::Value << ZtoY_;
        out << YAML::Key << "Yofs" << YAML::Value << Yofs_;
        out << YAML::Key << "XtoZ" << YAML::Value << XtoZ_;
        out << YAML::Key << "YtoZ" << YAML::Value << YtoZ_;
        out << YAML::Key << "Zgain" << YAML::Value << Zgain_;
        out << YAML::Key << "Zofs" << YAML::Value << Zofs_;

        out << YAML::Key << "gyro_x_offset" << YAML::Value << gyro_offsets_.x();
        out << YAML::Key << "gyro_y_offset" << YAML::Value << gyro_offsets_.y();
        out << YAML::Key << "gyro_z_offset" << YAML::Value << gyro_offsets_.z();
        out << YAML::EndMap;

        std::ofstream fout(filename);
        fout << out.c_str();
    }

    void printCalibration() {
        std::cout << "Xgain: " << Xgain_ << ", YtoX: " << YtoX_ << ", ZtoX: " << ZtoX_ << ", Xofs: " << Xofs_ << std::endl;
        std::cout << "XtoY: " << XtoY_ << ", Ygain: " << Ygain_ << ", ZtoY: " << ZtoY_ << ", Yofs: " << Yofs_ << std::endl;
        std::cout << "XtoZ: " << XtoZ_ << ", YtoZ: " << YtoZ_ << ", Zgain: " << Zgain_ << ", Zofs: " << Zofs_ << std::endl;
        std::cout << "Gyro offsets: " << gyro_offsets_.transpose() << std::endl;
    }

private:
    std::vector<Eigen::Vector3d> measured_accs_;
    std::vector<Eigen::Vector3d> true_accs_;
    std::vector<Eigen::Vector3d> measured_gyros_;

    double Xgain_, Ygain_, Zgain_;
    double XtoY_, XtoZ_, YtoX_, YtoZ_, ZtoX_, ZtoY_;
    double Xofs_, Yofs_, Zofs_;

    Eigen::Vector3d gyro_offsets_;
    Eigen::Matrix3d A_;  // Matrix of gains and cross-axis gains
    Eigen::Vector3d B_;  // Vector of offsets
};

class BMI160CalibrationNode {
public:
    BMI160CalibrationNode() : nh_("~"), bmi160_() {
        loadParameters();
        if (!initializeIMU()) {
            ROS_ERROR("Failed to initialize BMI160");
            ros::shutdown();
        }
    }

    void run() {
        std::vector<std::string> side_names{
            "+X up", "-X up", "+Y up", "-Y up", "+Z up", "-Z up"
        };

        std::vector<Eigen::Vector3d> true_accs{
            {1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0},
            {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}
        };

        std::set<int> used_sides;

        while (ros::ok() && used_sides.size() < 6) {
            std::cout << "Place the sensor in a new position and press Enter." << std::endl;
            std::cin.ignore();

            Eigen::Vector3d avg_measurement = collectMeasurements();

            int side = detectSide(avg_measurement);
            if (used_sides.find(side) != used_sides.end()) {
                std::cout << "This side has already been used. Please use another side." << std::endl;
                continue;
            }

            std::cout << "Collecting measurements for side " << side_names[side] << "." << std::endl;
            std::cout << "Hold the current position for 2 seconds..." << std::endl;
            ros::Duration(2.0).sleep();  // Wait for 2 seconds to ensure the IMU is stable

            collectMeasurementsForSide(true_accs[side]);
            used_sides.insert(side);
            std::cout << "Measurements for side " << side_names[side] << " collected." << std::endl;
        }

        std::cout << "Now, place the sensor with +Z pointing up and keep it level. Press Enter to start gyroscope calibration." << std::endl;
        std::cin.ignore();

        collectGyroMeasurements();
        
        if (ros::ok()) {
            calibration_.calibrate();
            calibration_.printCalibration();
            calibration_.writeCalibrationToYAML(yaml_output_path_);
        }

        ros::shutdown();
    }

private:
    ros::NodeHandle nh_;
    std::string i2c_bus_uri_;
    int bmi160_addr_;
    int num_measurements_;
    std::string yaml_output_path_;
    int accel_odr_, accel_range_, accel_bw_, gyro_odr_, gyro_range_, gyro_bw_;
    BMI160 bmi160_;
    bmi160_dev sensor_;
    SensorCalibration calibration_;

    void loadParameters() {
        nh_.param<std::string>("yaml_output_path", yaml_output_path_, "");

        nh_.param<int>("accel_odr", accel_odr_, BMI160_ACCEL_ODR_1600HZ);
        nh_.param<int>("accel_range", accel_range_, BMI160_ACCEL_RANGE_2G);
        nh_.param<int>("accel_bw", accel_bw_, BMI160_ACCEL_BW_NORMAL_AVG4);
        nh_.param<int>("gyro_odr", gyro_odr_, BMI160_GYRO_ODR_3200HZ);
        nh_.param<int>("gyro_range", gyro_range_, BMI160_GYRO_RANGE_2000_DPS);
        nh_.param<int>("gyro_bw", gyro_bw_, BMI160_GYRO_BW_NORMAL_MODE);
    }

    bool initializeIMU() {
        int8_t rslt = BMI160_OK;
        sensor_ = bmi160_.initialize(rslt);
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

    Eigen::Vector3d collectMeasurements() {
        std::vector<Eigen::Vector3d> measurements;
        for (int i = 0; i < num_measurements_; ++i) {
            bmi160_sensor_data acc_data;
            bmi160_get_sensor_data(BMI160_ACCEL_SEL, &acc_data, nullptr, &sensor_);
            Eigen::Vector3d measured_acc(acc_data.x, acc_data.y, acc_data.z);
            measurements.push_back(measured_acc);
            ros::Duration(0.02).sleep(); // Adjust the delay as necessary
        }
        Eigen::Vector3d avg_measurement = Eigen::Vector3d::Zero();
        for (const auto& measurement : measurements) {
            avg_measurement += measurement;
        }
        avg_measurement /= measurements.size();
        return avg_measurement;
    }

    void collectMeasurementsForSide(const Eigen::Vector3d& true_acc) {
        for (int i = 0; i < num_measurements_; ++i) {
            bmi160_sensor_data acc_data;
            bmi160_get_sensor_data(BMI160_ACCEL_SEL, &acc_data, nullptr, &sensor_);
            Eigen::Vector3d measured_acc(convertRawAccelToG(-1.0 * acc_data.x, accel_range_), convertRawAccelToG(-1.0 * acc_data.y, accel_range_), convertRawAccelToG(acc_data.z, accel_range_));
            calibration_.addMeasurement(measured_acc, true_acc);
            ros::Duration(0.02).sleep(); // Adjust the delay as necessary
        }
    }

    void collectGyroMeasurements() {
        for (int i = 0; i < num_measurements_; ++i) {
            bmi160_sensor_data gyro_data;
            bmi160_get_sensor_data(BMI160_GYRO_SEL, nullptr, &gyro_data, &sensor_);
            Eigen::Vector3d measured_gyro(-1.0 * gyro_data.x, -1.0 * gyro_data.y, gyro_data.z);
            calibration_.addGyroMeasurement(measured_gyro);
            ros::Duration(0.02).sleep(); // Adjust the delay as necessary
        }
    }

    int detectSide(const Eigen::Vector3d& measurement) {
        int max_index;
        measurement.cwiseAbs().maxCoeff(&max_index);
        double sign = measurement[max_index] > 0 ? 1.0 : -1.0;
        return max_index * 2 + (sign < 0 ? 1 : 0);
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bmi160_calibration_node");
    BMI160CalibrationNode node;
    node.run();
    return 0;
}
