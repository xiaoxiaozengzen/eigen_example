#include <Eigen/Dense>
#include <iostream>
#include <vector>

/**
 * @brief 计算从A到B的刚体变换（旋转和平移）
 * @param A 点集A
 * @param B 点集B
 * @param R 输出的旋转矩阵
 * @param t 输出的平移向量
 * 
 * @note 使得 B = R * A + t
 */
void rigid_transform(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, Eigen::Matrix3d& R, Eigen::Vector3d& t) {
    // 1. 计算质心
    Eigen::Vector3d centroid_A = A.rowwise().mean();
    Eigen::Vector3d centroid_B = B.rowwise().mean();

    // 2. 去中心化
    Eigen::MatrixXd AA = A.colwise() - centroid_A;
    Eigen::MatrixXd BB = B.colwise() - centroid_B;

    // 3. 计算协方差矩阵
    Eigen::Matrix3d H = AA * BB.transpose();

    // 4. SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // 5. 计算旋转
    R = V * U.transpose();
    if (R.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    // 6. 计算平移
    t = centroid_B - R * centroid_A;
}

int main() {
    std::cout << "========================= ego2rfu ========================" << std::endl;
    // ego点集
    Eigen::MatrixXd ego_points(3, 4);
    ego_points << 2, 2, -2, -2,
                  1, -1, -1, 1,
                  0, 0, 0, 0;
    // rfu点集
    Eigen::MatrixXd rfu_points(3, 4);
    rfu_points << -1, 1, 1, -1,
                  2, 2, -2, -2,
                  -0.3, -0.3, -0.3, -0.3;

    Eigen::Matrix3d ego2rfu_rotation;
    Eigen::Vector3d ego2rfu_translation;
    rigid_transform(ego_points, rfu_points, ego2rfu_rotation, ego2rfu_translation);

    std::cout << "Estimated R:\n" << ego2rfu_rotation << std::endl;
    std::cout << "Estimated t:\n" << ego2rfu_translation.transpose() << std::endl;

    double yaw = std::atan2(ego2rfu_rotation(1, 0), ego2rfu_rotation(0, 0));
    std::cout << "Estimated yaw: " << yaw * 180 / M_PI << " degrees" << std::endl;

    std::cout << "========================= test ========================" << std::endl;
    Eigen::Vector3d ego_point(3, 1, 0);
    Eigen::Vector3d rfu_point = ego2rfu_rotation * ego_point + ego2rfu_translation;
    std::cout << "Raw ego point: " << ego_point.transpose() << std::endl;
    std::cout << "Transformed rfu point: " << rfu_point.transpose() << std::endl;

    Eigen::Vector3d ego_point2(1, -1, 0);
    Eigen::Vector3d rfu_point2 = ego2rfu_rotation * ego_point2 + ego2rfu_translation;
    std::cout << "Raw ego point2: " << ego_point2.transpose() << std::endl;
    std::cout << "Transformed rfu point2: " << rfu_point2.transpose() << std::endl;
}