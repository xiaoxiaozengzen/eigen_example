#include <Eigen/Dense>
#include <iostream>
#include <vector>

/**
 * 对矩阵进行求逆： 
 *  1. 分块矩阵求逆
 *  2. 伴随矩阵求逆
 *  3. LU分解求逆
 *  4. 直接求逆(高斯消元法)：直观但是稳定性较差，效率低。矩阵很大时不建议使用。
 */

void inverse_stright(const Eigen::Matrix3d& r, const Eigen::Vector3d& t) {
    Eigen::Matrix4d ext = Eigen::Matrix4d::Identity();
    ext.block<3, 3>(0, 0) = r;
    ext.block<3, 1>(0, 3) = t;
    Eigen::Matrix4d ext_inv = ext.inverse();
    std::cout << "ext_inv:\n" << ext_inv << std::endl;

}

/**
 * @brief 基于分块矩阵求逆，对4x4的外参矩阵进行求逆：
 *  1.一个4x4的矩阵：
 *      [ R  t ]
 *      [ 0  1 ]
 *  2.则上述矩阵的逆矩阵为：
 *      [ R^-1  -R^-1 * t ]
 *      [ 0     1          ]
 *  3.其中R^-1是R的逆矩阵，-R^-1 * t是R^-1与t的乘积的相反数。
 */
void inverse_block(const Eigen::Matrix3d& r, const Eigen::Vector3d& t) {
    Eigen::Matrix4d ext = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d r_inv = r.inverse();
    ext.block<3, 3>(0, 0) = r_inv;
    Eigen::Vector3d t_inv = r_inv * t;
    ext(0, 3) = t_inv(0) * -1;
    ext(1, 3) = t_inv(1) * -1;
    ext(2, 3) = t_inv(2) * -1;
    std::cout << "ext:\n" << ext << std::endl;
}

int main() {
    /**
     * 相机的外参数据
     */
    Eigen::Quaterniond r_q(0.505459, -0.495639, 0.497202, -0.501643);
    Eigen::Vector3d t(2.16793, -0.041, 1.62928);
    Eigen::Matrix3d r_m = r_q.toRotationMatrix();

    std::cout << "================== stright inverse ==================\n";
    inverse_stright(r_m, t);
    std::cout << "================== block inverse ==================\n";
    inverse_block(r_m, t);

    return 0;
}