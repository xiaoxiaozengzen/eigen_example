#include <iostream>
#include <Eigen/Dense>

enum class ZeekrReferencePoint {
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

Eigen::MatrixXd get_fake_point3D(const double& x, const double& y, const double& z, const Eigen::Vector3d& lwh,
                                 ZeekrReferencePoint key_point_type) {
  double left = 0.0;
  double right = 0.0;
  double front = 0.0;
  double rear = 0.0;
  double btm = z;
  double up = z + lwh(2);
  // rfu系的keypoint
  switch (key_point_type) {
    case ZeekrReferencePoint::Center:
      left = x - lwh(1) / 2;
      right = x + lwh(1) / 2;
      rear = y - lwh(0) / 2;
      front = y + lwh(0) / 2;
      break;
    case ZeekrReferencePoint::RearMiddle:
      left = x - lwh(1) / 2;
      right = x + lwh(1) / 2;
      front = y + lwh(0);
      rear = y;
      break;
    case ZeekrReferencePoint::RearLeft:
      left = x;
      right = x + lwh(1);
      front = y + lwh(0);
      rear = y;
      break;
    case ZeekrReferencePoint::RearRight:
      left = x - lwh(1);
      right = x;
      front = y + lwh(0);
      rear = y;
      break;
    case ZeekrReferencePoint::FrontMiddle:
      left = x - lwh(1) / 2;
      right = x + lwh(1) / 2;
      front = y;
      rear = y - lwh(0);
      break;
    case ZeekrReferencePoint::LeftMiddle:
      left = x;
      right = x + lwh(1);
      front = y + lwh(0) / 2;
      rear = y - lwh(0) / 2;
      break;
    case ZeekrReferencePoint::RightMiddle:
      left = x - lwh(1);
      right = x;
      front = y + lwh(0) / 2;
      rear = y - lwh(0) / 2;
      break;
    case ZeekrReferencePoint::FrontLeft:
      left = x;
      right = x + lwh(1);
      front = y;
      rear = y - lwh(0);
      break;
    case ZeekrReferencePoint::FrontRight:
      left = x - lwh(1);
      right = x;
      front = y;
      rear = y - lwh(0);
      break;
    default:
      break;
  }

  Eigen::MatrixXd point3D(8, 3);
  point3D << left, front, btm, left, rear, btm, right, rear, btm, right, front, btm, left, front, up, left, rear, up,
      right, rear, up, right, front, up;
  return point3D;
}

Eigen::Vector3d CalCenterPointUseKeyPoint(const Eigen::Vector3d& key_point, const Eigen::Vector3d& lwh,
                                          const double& yaw_degree, ZeekrReferencePoint ref_postion) {
  Eigen::Vector3d center_point = key_point;
  Eigen::MatrixXd new_point3D = get_fake_point3D(0, 0, 0, lwh, ref_postion);

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(Eigen::Vector3d(center_point[0], center_point[1], center_point[2]));
  double yaw_rad = yaw_degree / 180.0 * M_PI - M_PI / 2.0;
  transform.rotate(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
  new_point3D = new_point3D * transform.rotation().transpose();
  new_point3D.rowwise() += transform.translation().transpose();

  center_point[0] = (new_point3D(0, 0) + new_point3D(6, 0)) / 2;
  center_point[1] = (new_point3D(0, 1) + new_point3D(6, 1)) / 2;
  center_point[2] = (new_point3D(0, 2) + new_point3D(6, 2)) / 2;
  return center_point;
}

int main() {
    Eigen::Vector3d key_point(-10.120, -0.50, 0.0);
    Eigen::Vector3d lwh(5.60, 1.7, 0);
    double lgt = -0.727;
    double lat = -2.834;
    Eigen::Vector3d cal_center_point = key_point + Eigen::Vector3d(lgt, lat, 0);
    double yaw_degree = 0.043;
    double yaw_rad = yaw_degree / 180.0 * M_PI;
    double yaw_rad_rfu = yaw_rad - M_PI / 2.0;
    ZeekrReferencePoint ref_position = ZeekrReferencePoint::FrontRight;

    uint8_t v_1 = 1;
    double v_2 = static_cast<double>(v_1);
    std::cout << "v_2: " << v_2 << std::endl;
    int v_3 = static_cast<int>(v_2);
    std::cout << "v_3: " << v_3 << std::endl;

    Eigen::Vector3d center_point_degree = CalCenterPointUseKeyPoint(key_point, lwh, yaw_degree, ref_position);
    Eigen::Vector3d center_point_rad = CalCenterPointUseKeyPoint(key_point, lwh, yaw_rad, ref_position);
    Eigen::Vector3d center_point_rfu = CalCenterPointUseKeyPoint(key_point, lwh, yaw_rad_rfu, ref_position);
    std::cout << "Key Point: " << key_point.transpose() << std::endl;
    std::cout << "Center Point Degree: " << center_point_degree.transpose() << std::endl;
    std::cout << "Center Point Rad: " << center_point_rad.transpose() << std::endl;
    std::cout << "Center Point Rfu: " << center_point_rfu.transpose() << std::endl;

    return 0;
}