/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 *
 */

#include <signal.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>

#include "CYdLidar.h"
#include "rclcpp/rclcpp.hpp"

using namespace ydlidar;
using namespace std::chrono_literals;
bool need_exit = false;

#define ROSVerision "1.3.9"

class LaserScanPublisher : public rclcpp::Node {
   public:
    LaserScanPublisher()
        : Node("ydlidar_node") {
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    }
    int work_loop() {
        this->init_param();

        rclcpp::WallRate loop_rate(30);
        while (rclcpp::ok() && !need_exit) {
            bool hardError;
            LaserScan scan;
            if (this->laser.doProcessSimple(scan, hardError)) {
                auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
                scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.system_time_stamp);
                scan_msg->header.stamp.nanosec = scan.system_time_stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
                scan_msg->header.frame_id = frame_id;
                scan_msg->angle_min = scan.config.min_angle;
                scan_msg->angle_max = scan.config.max_angle;
                scan_msg->angle_increment = scan.config.ang_increment;
                scan_msg->scan_time = scan.config.scan_time;
                scan_msg->time_increment = scan.config.time_increment;
                scan_msg->range_min = scan.config.min_range;
                scan_msg->range_max = scan.config.max_range;
                scan_msg->ranges = scan.ranges;
                scan_msg->intensities = scan.intensities;

                scan_pub->publish(*scan_msg);
            }
            loop_rate.sleep();
            rclcpp::spin_some(shared_from_this());
        }

        this->laser.turnOff();
        printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
        this->laser.disconnecting();
        return 0;
    }

   private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    std::string port;
    int baudrate = 115200;
    std::string frame_id;
    bool resolution_fixed, intensities, low_exposure, auto_reconnect, reversion;
    double angle_max, angle_min;
    int samp_rate;
    double max_range, min_range, _frequency;
    std::string list;
    std::vector<float> ignore_array;
    CYdLidar laser;

    void init_param() {
        this->declare_parameter<std::string>("port");
        this->declare_parameter<int>("baudrate");
        this->declare_parameter<std::string>("frame_id");
        this->declare_parameter<bool>("resolution_fixed");
        this->declare_parameter<bool>("intensity");
        this->declare_parameter<bool>("low_exposure");
        this->declare_parameter<bool>("auto_reconnect");
        this->declare_parameter<bool>("reversion");
        this->declare_parameter<double>("angle_max");
        this->declare_parameter<double>("angle_min");
        this->declare_parameter<int>("samp_rate");
        this->declare_parameter<double>("range_max");
        this->declare_parameter<double>("range_min");
        this->declare_parameter<double>("frequency");
        this->declare_parameter<std::string>("ignore_array");

        this->get_parameter_or<std::string>("port", port, "/dev/ydlidar");
        this->get_parameter_or<int>("baudrate", baudrate, 115200);
        this->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
        this->get_parameter_or<bool>("resolution_fixed", resolution_fixed, true);
        this->get_parameter_or<bool>("intensity", intensities, false);
        this->get_parameter_or<bool>("low_exposure", low_exposure, false);
        this->get_parameter_or<bool>("auto_reconnect", auto_reconnect, true);
        this->get_parameter_or<bool>("reversion", reversion, false);
        this->get_parameter_or<double>("angle_max", angle_max, 180.0);
        this->get_parameter_or<double>("angle_min", angle_min, -180.0);
        this->get_parameter_or<int>("samp_rate", samp_rate, 4);
        this->get_parameter_or<double>("range_max", max_range, 16.0);
        this->get_parameter_or<double>("range_min", min_range, 0.08);
        this->get_parameter_or<double>("frequency", _frequency, 7.0);
        this->get_parameter_or<std::string>("ignore_array", list, "");

        ignore_array = this->split(list, ',');

        if (ignore_array.size() % 2) {
            printf("ignore array is odd need be even\n");
        }
        for (uint16_t i = 0; i < ignore_array.size(); i++) {
            if (ignore_array[i] < -180 && ignore_array[i] > 180) {
                printf("ignore array should be between -180 and 180\n");
            }
        }
        if (_frequency < 5) {
            _frequency = 7.0;
        }
        if (_frequency > 12) {
            _frequency = 12;
        }
        if (angle_max < angle_min) {
            double temp = angle_max;
            angle_max = angle_min;
            angle_min = temp;
        }
        this->laser.setSerialPort(port);
        this->laser.setSerialBaudrate(baudrate);
        this->laser.setIntensities(intensities);
        this->laser.setMaxRange(max_range);
        this->laser.setMinRange(min_range);
        this->laser.setMaxAngle(angle_max);
        this->laser.setMinAngle(angle_min);
        this->laser.setReversion(reversion);
        this->laser.setFixedResolution(resolution_fixed);
        this->laser.setAutoReconnect(auto_reconnect);
        this->laser.setExposure(low_exposure);
        this->laser.setScanFrequency(_frequency);
        this->laser.setSampleRate(samp_rate);
        this->laser.setReversion(reversion);
        this->laser.setIgnoreArray(ignore_array);
        this->laser.initialize();
    }

    std::vector<float> split(const std::string &s, char delim) {
        std::vector<float> elems;
        std::stringstream ss(s);
        std::string number;
        while (std::getline(ss, number, delim)) {
            elems.push_back(atof(number.c_str()));
        }
        return elems;
    }
};

void ExitHandler(int sig) {
    (void)sig;
    need_exit = true;
}

int main(int argc, char *argv[]) {
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    fflush(stdout);
    rclcpp::init(argc, argv);
    auto ydlidar_node = std::make_shared<LaserScanPublisher>();
    signal(SIGINT, ExitHandler);
    int ret = ydlidar_node->work_loop();
    rclcpp::shutdown();
    return ret;
}
