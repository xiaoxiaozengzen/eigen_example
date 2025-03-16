/**
 * @file realusing.cpp
 * @brief Eigen library example
 * @note show real using of Eigen library
 */

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

struct IntrinsicDistortion {
  double k1;
  double k2;
  double k3;
  double k4;
  double k5;
  double k6;
  double p1;
  double p2;
};

struct IntrinsicParam {
  double fx;
  double fy;
  double cx;
  double cy;

  IntrinsicDistortion intrinsic_distortion;
  std::string distortion_model;

  int image_width;
  int image_height;
};


int GetRfu2Ins(Eigen::Matrix4d* matrix) {
    std::string root_path = "/mnt/workspace/cgz_workspace/Exercise/eigen_example/calibration";
    std::string lidar_ego_file = root_path + "/lidar_params/lidar_gnss.json";
    std::ifstream ifs(lidar_ego_file);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file: " << lidar_ego_file << std::endl;
        return -1;
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    std::string json_str = ss.str();

    nlohmann::json json_data = nlohmann::json::parse(json_str);
    std::array<double, 3> translation;
    std::array<double, 4> rotation;
    translation[0] = json_data.at("transform").at("translation").at("x");
    translation[1] = json_data.at("transform").at("translation").at("y");
    translation[2] = json_data.at("transform").at("translation").at("z");
    rotation[0] = json_data.at("transform").at("rotation").at("w");
    rotation[1] = json_data.at("transform").at("rotation").at("x");
    rotation[2] = json_data.at("transform").at("rotation").at("y");
    rotation[3] = json_data.at("transform").at("rotation").at("z");
    Eigen::Vector3d translation_eigen(translation[0], translation[1], translation[2]);
    Eigen::Translation3d translation_matrix(translation_eigen);
    Eigen::Quaterniond rotation_eigen(rotation[0], rotation[1], rotation[2], rotation[3]);
    Eigen::Affine3d transform_matrix = translation_matrix * rotation_eigen;
    Eigen::Matrix4d transform_matrix4d = transform_matrix.matrix();
    std::cout << "transform_matrix4d: " << std::endl << transform_matrix4d << std::endl;
    Eigen::Matrix4d transform_matrix4d_2 = Eigen::Matrix4d::Identity();
    transform_matrix4d_2.block<3, 3>(0, 0) = rotation_eigen.toRotationMatrix();
    transform_matrix4d_2.block<3, 1>(0, 3) = translation_eigen;
    std::cout << "transform_matrix4d_2: " << std::endl << transform_matrix4d_2 << std::endl;

    *matrix = transform_matrix4d;
    // 欧拉角转换有点问题，不清楚是不是矩阵问题
    std::array<double, 3> euler_angles;
    euler_angles[0] = json_data.at("euler_degree").at("RotZ");
    euler_angles[1] = json_data.at("euler_degree").at("RotY");
    euler_angles[2] = json_data.at("euler_degree").at("RotX");
    Eigen::Vector3d euler_angles_eigen;
    euler_angles_eigen[0] = euler_angles[0] * M_PI / 180.0;
    euler_angles_eigen[1] = euler_angles[1] * M_PI / 180.0;
    euler_angles_eigen[2] = euler_angles[2] * M_PI / 180.0;
    std::cout << "euler_angles_eigen: " << euler_angles_eigen.transpose() << std::endl;
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(euler_angles_eigen[0], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(euler_angles_eigen[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(euler_angles_eigen[2], Eigen::Vector3d::UnitX());
    std::cout << "rotation_matrix: " << std::endl << rotation_matrix << std::endl;
    Eigen::Matrix3d rotation_matrix_transpose = rotation_matrix.transpose();
    std::cout << "rotation_matrix_transpose: " << std::endl << rotation_matrix_transpose << std::endl;
    Eigen::Matrix3d rotation_matrix_inverse = rotation_matrix.inverse();
    std::cout << "rotation_matrix_inverse: " << std::endl << rotation_matrix_inverse << std::endl;

    return 0;
}

int GetIns2Ego(Eigen::Matrix4d* matrix) {
    std::string root_path = "/mnt/workspace/cgz_workspace/Exercise/eigen_example/calibration";
    std::string gnss_ego_file = root_path + "/gnss_params/gnss_ego.json";
    std::ifstream ifs(gnss_ego_file);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file: " << gnss_ego_file << std::endl;
        return -1;
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    std::string json_str = ss.str();

    nlohmann::json json_data = nlohmann::json::parse(json_str);
    std::array<double, 3> translation;
    std::array<double, 4> rotation;
    translation[0] = json_data.at("transform").at("translation").at("x");
    translation[1] = json_data.at("transform").at("translation").at("y");
    translation[2] = json_data.at("transform").at("translation").at("z");
    rotation[0] = json_data.at("transform").at("rotation").at("w");
    rotation[1] = json_data.at("transform").at("rotation").at("x");
    rotation[2] = json_data.at("transform").at("rotation").at("y");
    rotation[3] = json_data.at("transform").at("rotation").at("z");
    Eigen::Vector3d translation_eigen(translation[0], translation[1], translation[2]);
    Eigen::Translation3d translation_matrix(translation_eigen);
    Eigen::Quaterniond rotation_eigen(rotation[0], rotation[1], rotation[2], rotation[3]);
    Eigen::Affine3d transform_matrix = translation_matrix * rotation_eigen;
    Eigen::Matrix4d transform_matrix4d = transform_matrix.matrix();
    std::cout << "transform_matrix4d: " << std::endl << transform_matrix4d << std::endl;
    Eigen::Matrix4d transform_matrix4d_2 = Eigen::Matrix4d::Identity();
    transform_matrix4d_2.block<3, 3>(0, 0) = rotation_eigen.toRotationMatrix();
    transform_matrix4d_2.block<3, 1>(0, 3) = translation_eigen;
    std::cout << "transform_matrix4d_2: " << std::endl << transform_matrix4d_2 << std::endl;

    *matrix = transform_matrix4d;

    return 0;
}

// 成像的过程实质上是几个坐标系的转换。
// 1. 首相有一个世界坐标系的点坐标，这个点坐标是在世界坐标系中的。
// 2. 世界坐标系转换到摄像机坐标系 ，world2camera。
//    相机坐标系：原点是相机的光心，z轴是相机的光轴，x轴是相机的水平方向，y轴是相机的垂直方向。成像原理参考小孔成像原理。
// 3. 然后再将其投影到成像平面图像物理坐标系，camera2image
//    相机坐标系原点O1到图像坐标系原点O2的距离是焦距f，O1到O2的方向是z轴方向
// 4. 最后再将成像平面上的数据转换到图像平面图像像素坐标系。image2pixel。
//    3和4可以合并成使用相机的内参得到矩阵，即像素坐标=内参矩阵*相机坐标。
// 由于相机的内参和畸变参数是固定的，所以这个过程是固定的。
// 但是由于相机的外参是不固定的，所以world2camera是不固定的。

// 畸变参数数量取决于畸变模型：
// 1. 鱼眼相机的畸变模型是4个参数，分别是k1, k2, k3, k4。
// 2. 径向切向畸变模型是5个参数，分别是k1, k2, p1, p2, k3。
// 3. 径向切向畸变模型是3个参数，分别是k1, k2, k3。
// 3. 切向畸变模型是2个参数，分别是p1, p2。

/**
 * @brief Get camera parameters
 * @note 相机内参包括两部分：内参矩阵K和畸变系数D
 * 内参矩阵K是3X3的矩阵。K=[fx 0 cx; 0 fy cy; 0 0 1]，其中fx和fy是焦距，cx和cy是光心的坐标。
 * 畸变系数D是一个5X1的向量，D=[k1 k2 p1 p2 k3]，其中k1-k3是径向畸变系数，p1和p2是切向畸变系数。
 */
int GetCamParam() {
    std::string root_path = "/mnt/workspace/cgz_workspace/Exercise/eigen_example/calibration";
    std::string camera_name = "cam_front_30";
    std::string extrinsic_file = root_path + "/camera_params/" + camera_name + "_extrinsic.json";
    std::string intrinsic_file = root_path + "/camera_params/" + camera_name + "_intrinsic.json";
    std::ifstream ifs_extrinsic(extrinsic_file);
    if (!ifs_extrinsic.is_open()) {
        std::cerr << "Failed to open file: " << extrinsic_file << std::endl;
        return -1;
    }
    std::ifstream ifs_intrinsic(intrinsic_file);
    if (!ifs_intrinsic.is_open()) {
        std::cerr << "Failed to open file: " << intrinsic_file << std::endl;
        return -1;
    }

    // 获取外参
    std::stringstream ss_extrinsic;
    ss_extrinsic << ifs_extrinsic.rdbuf();
    std::string json_str_extrinsic = ss_extrinsic.str();
    nlohmann::json json_data_extrinsic = nlohmann::json::parse(json_str_extrinsic);
    std::array<double, 3> translation;
    std::array<double, 4> rotation;
    translation[0] = json_data_extrinsic.at("transform").at("translation").at("x");
    translation[1] = json_data_extrinsic.at("transform").at("translation").at("y");
    translation[2] = json_data_extrinsic.at("transform").at("translation").at("z");
    rotation[0] = json_data_extrinsic.at("transform").at("rotation").at("w");
    rotation[1] = json_data_extrinsic.at("transform").at("rotation").at("x");
    rotation[2] = json_data_extrinsic.at("transform").at("rotation").at("y");
    rotation[3] = json_data_extrinsic.at("transform").at("rotation").at("z");
    Eigen::Vector3d translation_eigen(translation[0], translation[1], translation[2]);
    Eigen::Translation3d translation_matrix(translation_eigen);
    Eigen::Quaterniond rotation_eigen(rotation[0], rotation[1], rotation[2], rotation[3]);
    Eigen::Affine3d rfu2camfront30_affine = translation_matrix * rotation_eigen;
    Eigen::Matrix4d rfu2camfront30_matrix4d = rfu2camfront30_affine.matrix();
    std::cout << "rfu2camfront30_matrix4d: " << std::endl << rfu2camfront30_matrix4d << std::endl;

    // 获取内参
    std::stringstream ss_intrinsic;
    ss_intrinsic << ifs_intrinsic.rdbuf();
    std::string json_str_intrinsic = ss_intrinsic.str();
    nlohmann::json json_data_intrinsic = nlohmann::json::parse(json_str_intrinsic);
    IntrinsicParam intrinsic_param;
    intrinsic_param.fx = json_data_intrinsic["K"][0][0];
    intrinsic_param.fy = json_data_intrinsic["K"][1][1];
    intrinsic_param.cx = json_data_intrinsic["K"][0][2];
    intrinsic_param.cy = json_data_intrinsic["K"][1][2];
    intrinsic_param.distortion_model = json_data_intrinsic["distortion_model"];
    intrinsic_param.intrinsic_distortion.k1 = json_data_intrinsic["D"][0][0];
    intrinsic_param.intrinsic_distortion.k2 = json_data_intrinsic["D"][1][0];
    intrinsic_param.intrinsic_distortion.k3 = json_data_intrinsic["D"][2][0];
    intrinsic_param.intrinsic_distortion.k4 = json_data_intrinsic["D"][3][0];

    if(intrinsic_param.distortion_model == "pinhole") {
      intrinsic_param.intrinsic_distortion.k5 = json_data_intrinsic["D"][4][0];
    } else {
      intrinsic_param.intrinsic_distortion.k5 = 0.0;
    }
    intrinsic_param.intrinsic_distortion.k6 = 0.0;
    intrinsic_param.intrinsic_distortion.p1 = 0.0;
    intrinsic_param.intrinsic_distortion.p2 = 0.0;
    intrinsic_param.image_width = json_data_intrinsic["resolution"][0];
    intrinsic_param.image_height = json_data_intrinsic["resolution"][1];

    Eigen::Matrix3d K;
    K << intrinsic_param.fx, 0, intrinsic_param.cx,
         0, intrinsic_param.fy, intrinsic_param.cy,
         0, 0, 1;
    std::cout << "K: " << std::endl << K << std::endl;

    return 0;
}

int main(int argc, char** argv) {
    Eigen::Matrix4d rfu2ins_matrix;
    GetRfu2Ins(&rfu2ins_matrix);
    Eigen::Matrix4d ins2ego_matrix;
    GetIns2Ego(&ins2ego_matrix);
    std::cout << "===============rfu2ego_matrix===============" << std::endl;
    Eigen::Matrix4d rfu2ego_matrix = ins2ego_matrix * rfu2ins_matrix;
    std::cout << "rfu2ego_matrix: " << std::endl << rfu2ego_matrix << std::endl;
    Eigen::Affine3d rfu2ego_affine(rfu2ego_matrix);
    Eigen::Vector3d translation = rfu2ego_affine.translation();
    std::cout << "translation: " << translation.transpose() << std::endl;
    Eigen::Matrix3d rotation = rfu2ego_affine.rotation();
    std::cout << "rotation: " << std::endl << rotation << std::endl;
    Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0);
    std::cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << std::endl;

    std::cout << "====================testing=====================" << std::endl;
    Eigen::Vector3d rfu_point(1.0, 2.0, 0.0);
    Eigen::Vector3d ego_point = rfu2ego_affine * rfu_point;
    std::cout << "rfu_point: " << rfu_point.transpose() << std::endl;
    std::cout << "ego_point: " << ego_point.transpose() << std::endl;
    std::cout << "=====================CamParam=======================" << std::endl;
    GetCamParam();
    return 0;
}