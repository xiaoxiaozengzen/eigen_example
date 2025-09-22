#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

void construct() {
    // 1. 基于matrix的构造
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    Eigen::Affine3d affine3d_1(matrix);
    std::cout << "affine3d_1: " << std::endl << affine3d_1.matrix() << std::endl;

    // 2. 基于平移和旋转的构造
    Eigen::Vector3d translation_vector(1.0, 2.0, 3.0);
    Eigen::Translation3d translation(translation_vector);
    Eigen::AngleAxisd rotation(M_PI / 4, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quaternion(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d affine3d_2 = translation * rotation;
    std::cout << "affine3d_2: " << std::endl << affine3d_2.matrix() << std::endl;
    Eigen::Affine3d affine3d_3 = translation * quaternion;
    std::cout << "affine3d_3: " << std::endl << affine3d_3.matrix() << std::endl;

    // 3.默认构造函数
    Eigen::Affine3d affine3d_4 = Eigen::Affine3d::Identity();
    std::cout << "affine3d_4: " << std::endl << affine3d_4.matrix() << std::endl;
}

void interface() {
    // 一定要使用Identity初始化，否则矩阵本身会有随机值，其加上后续的旋转和平移会导致结果错误
    Eigen::Affine3d affine3d = Eigen::Affine3d::Identity();
    std::cout << "affine3d: " << std::endl << affine3d.matrix() << std::endl;

    // 平移：需要先设置平移部分，再进行旋转，否则会导致旋转后平移部分不正确
    Eigen::Vector3d translation_vector(1.0, 2.0, 3.0);
    affine3d.translate(translation_vector);
    std::cout << "after translate, affine3d: " << std::endl << affine3d.matrix() << std::endl;

    // 旋转
    Eigen::AngleAxisd rotation(M_PI / 4, Eigen::Vector3d::UnitZ());
    affine3d.rotate(rotation);
    std::cout << "after rotate, affine3d: " << std::endl << affine3d.matrix() << std::endl;

    // 获取旋转部分
    Eigen::Matrix3d rotation_matrix = affine3d.rotation();
    std::cout << "rotation_matrix: " << std::endl << rotation_matrix << std::endl;
    Eigen::Matrix3d rotation_matrix_2 = affine3d.linear();
    std::cout << "rotation_matrix_2: " << std::endl << rotation_matrix_2 << std::endl;

    // 获取平移部分
    Eigen::Vector3d translation_part = affine3d.translation();
    std::cout << "translation_part: " << translation_part.transpose() << std::endl;

    // 获取矩阵部分
    Eigen::Matrix4d matrix = affine3d.matrix();
    std::cout << "matrix: " << std::endl << matrix << std::endl;

    // 变换应用
    Eigen::Vector3d point(1.0, 1.0, 1.0);
    Eigen::Vector3d transformed_point = affine3d * point;
    std::cout << "transformed_point: " << transformed_point.transpose() << std::endl;

    // 直接使用平移和旋转构造
    Eigen::Translation3d translation_2(translation_vector);
    Eigen::Affine3d affine3d_2 = translation_2 * rotation_matrix;
    std::cout << "affine3d_2: " << std::endl << affine3d_2.matrix() << std::endl;
}

int main() {
    std::cout << "================= construct ===================" << std::endl;
    construct();
    std::cout << "================= interface ===================" << std::endl;
    interface();

    return 0;
}