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
// 1. 首先：有一个世界坐标系的点坐标，这个点坐标是在世界坐标系中的。
// 2. 其次：世界坐标系转换到摄像机坐标系 ，world2camera。
//    相机坐标系：原点是相机的光心，z轴是相机的光轴，x轴是相机的水平方向，y轴是相机的垂直方向。
// 3. 然后：将其投影到成像平面图像物理坐标系，camera2image
//    图像坐标系：与成像平面重合，以成像平面的中心(光轴与成像平面交点)为坐标原点，x轴跟y轴分别平行于成像平面的水平和垂直方向。
// 4. 最后：将成像平面上的数据转换到图像平面图像像素坐标系。image2pixel。
//    像素坐标系：与成像平面重合，以成像平面的左上角为坐标原点，x轴水平向右，y轴垂直向下。
//    3和4可以合并成使用相机的内参得到矩阵，即像素坐标=内参矩阵*相机坐标。
// 由于相机的内参和畸变参数是固定的，所以这个过程是固定的。
// 但是由于相机的外参是不固定的，所以world2camera是不固定的。

// 畸变：
//  理想的成像是针孔成像模型，但是实际的成像是有畸变的， 主要是由于镜头的制造工艺和材料的原因，导致光线在通过镜头时发生偏折，
//  从而导致成像点的位置偏离了理想位置，产生畸变。会从画面中心到边缘逐渐加剧，主要在画面边缘处畸变最为明显。
// 畸变主要分为两种：径向畸变和切向畸变(参考圆上一个点的径向跟切向)。
// 1. 径向畸变：是指光线在通过镜头时，偏离了理想的径向方向， 导致成像点的位置偏离了理想位置，产生畸变。
//    径向畸变主要有两种类型：桶形畸变和枕形畸变。
//    - 桶形畸变：成像点向外偏移，产生桶形畸变。
//    - 枕形畸变：成像点向内偏移，产生枕形畸变。
// 2. 切向畸变：是指光线在通过镜头时，偏离了理想的切向方向， 导致成像点的位置偏离了理想位置，产生畸变。
//    切向畸变主要是由于镜头的制造工艺和材料的原因，导致光线在通过镜头时发生偏折， 从而导致成像点的位置偏离了理想位置，产生畸变。
// 修正畸变，一般使用泰勒展开公式，具体的畸变参数数量取决于畸变模型：
// 1. 鱼眼相机的畸变模型是4个参数，分别是k1, k2, k3, k4。
// 2. 切向畸变模型是5个参数，分别是k1, k2, k3， p1, p2。
// 3. 径向畸变模型是3个参数，分别是k1, k2, k3。

/**
 * @brief Get camera parameters
 * @note 相机内参包括两部分：内参矩阵K和畸变系数D
 * 内参矩阵K是3X3的矩阵。K=[fx 0 cx; 0 fy cy; 0 0 1]
 *  - fx,fy,cx,cy的单位都是像素
 *  - fx = f / dx, fy = f / dy; f是焦距，单位是mm; dx,dy是像素的物理尺寸，单位是mm/像素
 *  - cx,cy是主点坐标，单位是像素
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

/******************************* calCenterPointFromKeyPoint ********************************************** */

enum class ReferencePosition {
  FrontRight = 0,
  FrontMiddle = 1,
  FrontLeft = 2,
  LeftMiddle = 3,
  RearLeft = 4,
  RearMiddle = 5,
  RearRight = 6,
  RightMiddle = 7,
  Center = 8,
  UNKNOWN = 9,
  MaxReferencePoint = 10,
};

void calCenterPointFromKeyPoint(const Eigen::Vector3d& key_point, const Eigen::Vector3d& lwh, const double& yaw_degree, ReferencePosition ref_postion, Eigen::Vector3d* center_point) {
  Eigen::Vector3d cal_center_point = key_point;
  
  /**
   * @brief 计算车辆8个顶点坐标
   * 
   */
  // 目标框的底部和顶部z坐标
  double bottom = 0.0;
  double up = lwh[2];
  // 基于参考点位置，得到其到目标框的front, rear, left, right的偏移量
  double front = 0.0;
  double rear = 0.0;
  double left = 0.0;
  double right = 0.0;
  switch (ref_postion) {
    case ReferencePosition::FrontRight:
      front = 0.0;
      rear = -lwh[0];
      left = lwh[1];
      right = 0.0;
      break;
    case ReferencePosition::FrontMiddle:
      front = 0.0;
      rear = -lwh[0];
      left = lwh[1] / 2;
      right = -lwh[1] / 2;
      break;
    case ReferencePosition::FrontLeft:
      front = 0.0;
      rear = -lwh[0];
      left = 0.0;
      right = -lwh[1];
      break;
    case ReferencePosition::LeftMiddle:
      front = lwh[0] / 2;
      rear = -lwh[0] / 2;
      left = 0.0;
      right = -lwh[1];
      break;
    case ReferencePosition::RearLeft:
      front = lwh[0];
      rear = 0.0;
      left = 0.0;
      right = -lwh[1];
      break;
    case ReferencePosition::RearMiddle:
      front = lwh[0];
      rear = 0.0;
      left = lwh[1] / 2;
      right = -lwh[1] / 2;
      break;
    case ReferencePosition::RearRight:
      front = lwh[0];
      rear = 0.0;
      left = lwh[1];
      right = 0.0;
      break;
    case ReferencePosition::RightMiddle:
      front = lwh[0] / 2;
      rear = -lwh[0] / 2;
      left = lwh[1];
      right = 0.0;
      break;
    case ReferencePosition::Center:
      front = lwh[0] / 2;
      rear = -lwh[0] / 2;
      left = lwh[1] / 2;
      right = -lwh[1] / 2;
      break;
    default:
      break;
  }

  Eigen::MatrixXd cube(8, 3);
  cube << front, left, bottom,
          front, right, bottom,
          rear, right, bottom,
          rear, left, bottom,
          front, left, up,
          front, right, up,
          rear, right, up,
          rear, left, up;

  Eigen::Affine3d rotation_matrix = Eigen::Affine3d::Identity();
  Eigen::Vector3d transform_matrix(key_point.x(), key_point.y(), key_point.z());
  rotation_matrix.translate(transform_matrix);
  rotation_matrix.rotate(Eigen::AngleAxisd(yaw_degree * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
  Eigen::MatrixXd rotated_cube = (rotation_matrix.rotation() * cube.transpose()).transpose();
  rotated_cube.rowwise() += transform_matrix.transpose();

  cal_center_point[0] = (rotated_cube(0, 0) + rotated_cube(6, 0)) / 2.0;
  cal_center_point[1] = (rotated_cube(0, 1) + rotated_cube(6, 1)) / 2.0;
  cal_center_point[2] = (rotated_cube(0, 2) + rotated_cube(6, 2)) / 2.0;
  
  *center_point = cal_center_point;
}

void test_calCenterPointFromKeyPoint() {
  Eigen::Vector3d key_point(10, -1.0, 0.0);
  Eigen::Vector3d lwh(2.0, 2.0, 0.0);
  std::cout << "key_point: " << key_point.transpose() << std::endl;
  // test 1
  double yaw_degree_1 = 0.0;
  Eigen::Vector3d center_point_1;
  ReferencePosition ref_postion_1 = ReferencePosition::RearLeft;
  calCenterPointFromKeyPoint(key_point, lwh, yaw_degree_1, ref_postion_1, &center_point_1);
  std::cout << "center_point_1: " << center_point_1.transpose() << std::endl;
  // test 2
  double yaw_degree_2 = 45.0;
  Eigen::Vector3d center_point_2;
  ReferencePosition ref_postion_2 = ReferencePosition::FrontLeft;
  calCenterPointFromKeyPoint(key_point, lwh, yaw_degree_2, ref_postion_2, &center_point_2);
  std::cout << "center_point_2: " << center_point_2.transpose() << std::endl;
  // test 3
  double yaw_degree_3 = 45.0;
  Eigen::Vector3d center_point_3;
  ReferencePosition ref_postion_3 = ReferencePosition::Center;
  calCenterPointFromKeyPoint(key_point, lwh, yaw_degree_3, ref_postion_3, &center_point_3);
  std::cout << "center_point_3: " << center_point_3.transpose() << std::endl;
  // test 4
  double yaw_degree_4 = 45.0;
  Eigen::Vector3d center_point_4;
  ReferencePosition ref_postion_4 = ReferencePosition::RearLeft;
  calCenterPointFromKeyPoint(key_point, lwh, yaw_degree_4, ref_postion_4, &center_point_4);
  std::cout << "center_point_4: " << center_point_4.transpose() << std::endl;
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

    std::cout << "=================calCenterPointFromKeyPoint===================" << std::endl;
    test_calCenterPointFromKeyPoint();
    return 0;
}