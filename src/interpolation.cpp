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
 * 使用Eigen进行插值的示例代码
 * 使用场景：
 *   在进行传感器同步过程中，例如获取了一帧图片，但对应的IMU数据不完全对齐，
 *   此时可以使用插值方法来估计IMU数据在图片时间点的值。
 */

/**
 * 插值方法简介：
 * 1.范德蒙德插值(Vandermonde interpolation)：
 * 范德蒙德矩阵(Vandermonde matrix)是什么：
 * 给一组定点，x0, x1, x2, ..., xn
 * 则范德蒙德矩阵V为：V(i, j) = x_i^j，i，j = 0, 1, ..., n
 * 展开后的矩阵如下：
 * V = | 1  x0  x0^2  ...  x0^n |
 *     | 1  x1  x1^2  ...  x1^n |
 *     | 1  x2  x2^2  ...  ...  x2^n |
 *     | ... ... ...  ...  ...   |
 *     | 1  xn  xn^2  ...  xn^n |
 * 怎么用
 * 通过求解线性方程组V * a = y，可以得到插值多项式的系数a，从而实现对任意时间点的插值估计
 *
 * 2. 拉格朗日插值(Lagrange interpolation)：
 * 重心拉格朗日插值(Barycentric Lagrange interpolation)是什么：
 * 已知点(xi, yi)，i = 0, 1, ..., n
 * 先算权重：
 * w_i = 1 / Π_{j≠i} (xi - xj)
 * 然后查询点x的插值结果为：
 * P(x) = (Σ_{i=0}^n (w_i * yi / (x - xi))) / (Σ_{i=0}^n (w_i / (x - xi))) 
 */

void interpolate_vandermonde() {
    // 模拟一些时间点和对应的传感器数据
    std::vector<double> time_points = {
        1773642558.134, 1773642558.245, 1773642558.352
    };
    std::vector<double> x_points = {
        100.89, 101.23, 101.56
    };
    double target_time = 1773642558.200;

    if(time_points.size() == 0 || x_points.size() == 0) {
        std::cerr << "Error: Time points and data points cannot be empty." << std::endl;
        return;
    }
    if(time_points.size() != x_points.size()) {
        std::cerr << "Error: Time points and data points must have the same size." << std::endl;
        return;
    }
    std::stringstream time_ss;
    time_ss << "Time points: ";
    time_ss << std::fixed << std::setprecision(3);
    for(const auto& t : time_points) {
        time_ss << t << " ";
    }
    std::cout << time_ss.str() << std::endl;
    std::cout << "Data points: ";
    for(const auto& x : x_points) {
        std::cout << x << " ";
    }
    std::cout << std::endl;

    // 构建Vandermonde矩阵进行插值
    int data_size = static_cast<int>(time_points.size());
    Eigen::MatrixXd V(data_size, data_size);
    V.setZero();
    for(size_t i = 0U; i < time_points.size(); ++i) {
        for(size_t j = 0U; j < x_points.size(); ++j) {
            V(i, j) = std::pow(time_points[i], j);
        }
    }
    std::cout << "Vandermonde Matrix V:\n" << V << std::endl;

    // 构建y向量
    Eigen::VectorXd y(data_size);
    for(size_t i = 0U; i < x_points.size(); ++i) {
        y(i) = x_points[i];
    }

    // 求解插值多项式的系数
    Eigen::VectorXd coefficients = V.colPivHouseholderQr().solve(y);
    std::cout << "Coefficients of the interpolation polynomial:\n" << coefficients.transpose() << std::endl;

    // 计算目标时间点的插值结果
    double interpolated_value = 0.0;
    for(size_t j = 0U; j < coefficients.size(); ++j) {
        interpolated_value += std::pow(target_time, j) * coefficients(j);
    }
    std::stringstream result_ss;
    result_ss << std::fixed << std::setprecision(3);
    result_ss << "Interpolated value at time " << target_time << ": " << interpolated_value;
    std::cout << result_ss.str() << std::endl;
}

void interpolate_barycentric_lagrange() {
    // 模拟一些时间点和对应的传感器数据
    std::vector<double> time_points = {
        1773642558.134, 1773642558.245, 1773642558.352
    };
    std::vector<double> x_points = {
        100.89, 101.23, 101.56
    };
    double target_time = 1773642558.200;

    if(time_points.size() == 0 || x_points.size() == 0) {
        std::cerr << "Error: Time points and data points cannot be empty." << std::endl;
        return;
    }
    if(time_points.size() != x_points.size()) {
        std::cerr << "Error: Time points and data points must have the same size." << std::endl;
        return;
    }
    std::stringstream time_ss;
    time_ss << "Time points: ";
    time_ss << std::fixed << std::setprecision(3);
    for(const auto& t : time_points) {
        time_ss << t << " ";
    }
    std::cout << time_ss.str() << std::endl;
    std::cout << "Data points: ";
    for(const auto& x : x_points) {
        std::cout << x << " ";
    }
    std::cout << std::endl;

    // 计算权重
    std::vector<double> weights(time_points.size(), 1.0);
    for(size_t i = 0U; i < time_points.size(); ++i) {
        for(size_t j = 0U; j < time_points.size(); ++j) {
            if(i == j) {
                continue;
            }
            weights[i] /= (time_points[i] - time_points[j]);            
        }
    }

    // 计算插值结果
    // 1.命中直接返回(防止除0错误)
    for(size_t i = 0U; i < time_points.size(); ++i) {
        if(std::abs(target_time - time_points[i]) < 1e-9) {
            std::stringstream result_ss;
            result_ss << std::fixed << std::setprecision(3);
            result_ss << "Target time matches time point " << time_points[i] << ": " << x_points[i];
            std::cout << result_ss.str() << std::endl;
            return;
        }
    }
    // 2.计算插值结果
    double numerator = 0.0;
    double denominator = 0.0;
    for(size_t i = 0U; i < time_points.size(); ++i) {
        double w_i = weights[i] / (target_time - time_points[i]);
        numerator += w_i * x_points[i];
        denominator += w_i;
    }
    double interpolated_value = numerator / denominator;
    std::stringstream result_ss;
    result_ss << std::fixed << std::setprecision(3);
    result_ss << "Interpolated value at time " << target_time << ": " << interpolated_value;
    std::cout << result_ss.str() << std::endl;
}

int main() {
    std::cout << "=========== interpolate_vandermonde ============" << std::endl;
    interpolate_vandermonde();
    std::cout << "=========== interpolate_barycentric_lagrange ============" << std::endl;
    interpolate_barycentric_lagrange();
    return 0;
}