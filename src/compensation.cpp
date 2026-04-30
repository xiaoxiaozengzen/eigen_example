#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/QR>

/**
 * 1.运动补偿：
 *   对于lidar的点云数据，运动补偿是指在点云数据采集过程中，由于传感器或平台的运动导致的点云数据失真。为了获得准确的点云数据，需要对这些失真进行补偿。
 *   运动补偿的基本原理是通过传感器或平台的运动信息（如速度、加速度、姿态等）来计算每个点云点的位置，并将其调整到一个统一的参考坐标系中。这样可以消除由于运动引起的点云数据失真，提高点云数据的准确性和可靠性。
 *   运动补偿的步骤通常包括：
 *   1. 获取传感器或平台的运动信息：通过传感器或平台的IMU（惯性测量单元）等设备获取运动信息，如速度、加速度、姿态等。
 *   2. 计算每个点云点的位置：根据运动信息和点云数据的采集时间，计算每个点云点的位置。通常使用运动模型（如匀速运动模型、加速度运动模型等）来计算点云点的位置。
 *   3. 调整点云数据：将每个点云点的位置调整到一个统一的参考坐标系中，消除由于运动引起的点云数据失真。
 *   4. 输出补偿后的点云数据：将补偿后的点云数据输出，以供后续处理和分析。
 * 2.模型：
 *  CV模型：Constant Velocity Model，假设目标以恒定速度运动，适用于目标运动较为平稳的情况。
 *  CA模型：Constant Acceleration Model，假设目标以恒定加速度运动，适用于目标运动较为复杂的情况。
 */

void compensation() {
    // 假设我们有一个点云数据和对应的IMU数据，我们需要对点云数据进行运动补偿。
    Eigen::VectorXd imu_velocity(3);
    imu_velocity << 1.0, 0.0, 0.0;
    Eigen::VectorXd imu_angle_velocity(3);
    imu_angle_velocity << 0.0, 0.0, 0.1;

    // 点云的原始位置和采集时间
    Eigen::VectorXd src_position(3);
    src_position << 20.1, 10.2, 5.3;
    double src_time = 1777519418.123;

    // 目标时间，我们需要将点云数据补偿到这个时间点
    double target_time = 1777519418.223;

    // 计算时间差
    double time_diff = target_time - src_time;
    if(time_diff < 1e-6) {
        std::cout << "Time difference is too small, no compensation needed." << std::endl;
        return;
    }

    // 对于静止的目标，只用计算自车旋转和平移引起的点云位置变化即可。
    // 1.计算自车平移引起的点云位置变化
    Eigen::Translation3d translation(-time_diff * imu_velocity(0), -time_diff * imu_velocity(1), -time_diff * imu_velocity(2));
    Eigen::VectorXd dst_position = src_position;
    dst_position = translation * dst_position;

    // 2.计算自车旋转引起的点云位置变化
    Eigen::Quaterniond rotation = Eigen::AngleAxisd(-time_diff * imu_angle_velocity(2), Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(-time_diff * imu_angle_velocity(1), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(-time_diff * imu_angle_velocity(0), Eigen::Vector3d::UnitX());
    dst_position = rotation * dst_position;

    std::cout << "src_position: " << src_position.transpose() << std::endl;
    std::cout << "dst_position: " << dst_position.transpose() << std::endl;

    // 对于运动的目标，还需要考虑目标的运动引起的点云位置变化。

}

int main() {
    std::cout << "================Compensation================" << std::endl;
    compensation();
    return 0;
}