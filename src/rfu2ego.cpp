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

void ego2rfu() {
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
}

void ego2rfu_test() {
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
    double yaw = std::atan2(ego2rfu_rotation(1, 0), ego2rfu_rotation(0, 0));

    Eigen::Vector3d ego_point(3, 1, 0);
    Eigen::Vector3d rfu_point = ego2rfu_rotation * ego_point + ego2rfu_translation;
    std::cout << "Raw ego point: " << ego_point.transpose() << std::endl;
    std::cout << "Transformed rfu point: " << rfu_point.transpose() << std::endl;

    Eigen::Vector3d ego_point2(1, -1, 0);
    Eigen::Vector3d rfu_point2 = ego2rfu_rotation * ego_point2 + ego2rfu_translation;
    std::cout << "Raw ego point2: " << ego_point2.transpose() << std::endl;
    std::cout << "Transformed rfu point2: " << rfu_point2.transpose() << std::endl;
}

/**
 * @brief Eigen中的旋转角度的正负测试
 * @note 右手定则，逆时针为正，顺时针为负
 * @note 旋转是指的坐标系的旋转，而不是点的旋转
 * @note 平移是指的坐标系的平移，而不是点的平移
 * @note 旋转和平移的顺序是先旋转后平移
 * @note 先旋转后平移：先绕原点旋转，再平移到新位置，常见于坐标变换。p' = R * p + t
 * @note 先平移后旋转：先平移到新位置，再绕原点旋转，常见于物体运动。p' = R * (p + t)
 * 
 */
void rotation_test() {
    Eigen::Vector3d ori_point(1, 1, 0);
    /**
     * 逆时针旋转90度，相当于将ego的坐标上的点逆时针旋转90度，就得到rfu的坐标
     * @note 例如：ego坐标系下的点(1, 0，0)，逆时针旋转90度后变为(0, 1, 0)，即rfu坐标系下的点
     */
    double rotation_angle = 90.0;
    Eigen::AngleAxisd rotation_vector(rotation_angle * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
    Eigen::Vector3d rotated_point = rotation_matrix * ori_point;
    std::cout << "Original point: " << ori_point.transpose() << std::endl;
    std::cout << "Rotation angle: " << rotation_angle << " degrees" << std::endl;
    std::cout << "Rotation matrix:\n" << rotation_matrix << std::endl;
    std::cout << "Rotated point: " << rotated_point.transpose() << std::endl;
}

int main() {
    std::cout << "========================= ego2rfu ========================" << std::endl;
    ego2rfu();
    std::cout << "========================= ego2rfu_test ========================" << std::endl;
    ego2rfu_test();
    std::cout << "========================= rotation_test ========================" << std::endl;
    rotation_test();

    return 0;
}