#include <iostream>
#include <cmath>

#include <Eigen/Dense>

int main() {
    /* rfu2ego matrix */

    // 旋转向量
    std::cout << "=============rotation_vector===============" << std::endl;
    Eigen::AngleAxisd rotation_vector1(-M_PI / 2, Eigen::Vector3d(0, 0, 1));
    std::cout << "rotation vector1 axis: " << rotation_vector1.axis().transpose() << std::endl;
    std::cout << "rotation vector1 angle: " << rotation_vector1.angle() << std::endl;
    std::cout << "Vector3d::UnitZ: " << Eigen::Vector3d::UnitZ().transpose() << std::endl;

    // 旋转矩阵
    std::cout << "==============rotation_matrix==============" << std::endl;
    Eigen::Matrix3d rotation_matrix1 = rotation_vector1.matrix();
    std::cout << "rotation matrix1 =\n" << rotation_matrix1 << std::endl;

    // 四元素
    std::cout << "==============Quaterniond==============" << std::endl;
    Eigen::Quaterniond quaternion1(rotation_vector1);
    std::cout << "quaternion1 x: " << quaternion1.x() << std::endl;
    std::cout << "quaternion1 y: " << quaternion1.y() << std::endl;
    std::cout << "quaternion1 z: " << quaternion1.z() << std::endl;
    std::cout << "quaternion1 w: " << quaternion1.w() << std::endl;

    // 欧拉角
    std::cout << "============euler================" << std::endl;
    Eigen::Vector3d eulerAngle1 = rotation_vector1.matrix().eulerAngles(2, 1, 0);
    std::cout << "eulerAngle1, z y x: " << eulerAngle1.transpose() << std::endl;

    // 测试
    std::cout << "============测试================" << std::endl;
    Eigen::Vector3d transformer_v = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d rfu_pos = Eigen::Vector3d(-1, 2, 0);
    std::cout << "rfu_pos: " << rfu_pos.transpose() << std::endl;
    Eigen::Vector3d ego_pos = rotation_matrix1 * rfu_pos + transformer_v;
    std::cout << "ego_pos: " << ego_pos.transpose() << std::endl;    
}