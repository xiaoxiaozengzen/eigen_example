#include <iostream>
#include <cmath>

#include <Eigen/Dense>

// 旋转矩阵（3X3）:Eigen::Matrix3d
// 旋转向量（3X1）:Eigen::AngleAxisd
// 四元数（4X1）:Eigen::Quaterniond
// 平移向量（3X1）:Eigen::Vector3d
// 欧氏变换矩阵（4X4）:Eigen::Isometry3d
// 欧拉角（3X1）:Eigen::Vector3d
// 仿射变换（4X4）:Eigen::Affine3d
// 平移变换（4X4）:Eigen::Translation3d
// 射影变换（4X4）:Eigen::Projective3d

/**
 * @brief 旋转向量（Rotation Vector）
 * 
 * @note 任意的旋转都可以用一个旋转轴和绕轴的旋转角来描述，简称“轴角”（Axis-Angle）；
 * 旋转向量，是一个三维向量，其方向与旋转轴一致，长度等于旋转角；
 */
void AngleAxisd_test() {
    /**** 1. 旋转向量 ****/
    std::cout << std::endl << "********** AngleAxis **********" << std::endl;
    //1.0 初始化旋转向量,沿Z轴旋转45度的旋转向量
    Eigen::AngleAxisd rotation_vector1 (M_PI/2, Eigen::Vector3d(0, 0, 1)); //以（0,0,1）为旋转轴，旋转90度，右手定则
    double angle = rotation_vector1.angle();
    std::cout << "rotation_vector1.agnle = " << angle << std::endl;
    Eigen::Vector3d axis = rotation_vector1.axis();
    std::cout << "rotation_vector1.axis = " << axis.transpose() << std::endl;
 
    //1.1 旋转向量转换为旋转矩阵
    //旋转向量用matrix()转换成旋转矩阵
    Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity();
    rotation_matrix1 = rotation_vector1.matrix();
    std::cout << "rotation matrix1 =\n" << rotation_matrix1 << std::endl;                
    //或者由罗德里格公式进行转换
    rotation_matrix1 = rotation_vector1.toRotationMatrix();
    std::cout << "rotation matrix1 =\n" << rotation_matrix1 << std::endl; 
 
    /*1.2 旋转向量转换为欧拉角*/
    //将旋转向量转换为旋转矩阵,再由旋转矩阵转换为欧拉角,详见旋转矩阵转换为欧拉角
    Eigen::Vector3d eulerAngle1 = rotation_vector1.matrix().eulerAngles(2,1,0);
    std::cout << "eulerAngle1, z y x: " << eulerAngle1.transpose() << std::endl;
 
    /*1.3 旋转向量转四元数*/
    Eigen::Quaterniond quaternion1(rotation_vector1);
    //或者
    Eigen::Quaterniond quaternion1_1;
    quaternion1_1 = rotation_vector1;
    std::cout << "quaternion1 x: " << quaternion1.x() << std::endl;
    std::cout << "quaternion1 y: " << quaternion1.y() << std::endl;
    std::cout << "quaternion1 z: " << quaternion1.z() << std::endl;
    std::cout << "quaternion1 w: " << quaternion1.w() << std::endl;
    
    std::cout << "quaternion1_1 x: " << quaternion1_1.x() << std::endl;
    std::cout << "quaternion1_1 y: " << quaternion1_1.y() << std::endl;
    std::cout << "quaternion1_1 z: " << quaternion1_1.z() << std::endl;
    std::cout << "quaternion1_1 w: " << quaternion1_1.w() << std::endl;
}

/**
 * @brief EulerAngles
 * @note 欧拉角提供了一种比较直观的表示法：将旋转分解成3次绕不同轴的旋转。
 * 
 * @note 比较常用得欧拉角RPY(roll-pitch-yaw)表示法，等价于ZYX轴得旋转，即：(一般飞机头部朝向为x轴，左翼为y轴，上翼为z轴)
 * 绕物体的 Z 轴旋转，得到偏航角 Yaw；
 * 绕物体的 Y 轴旋转，得到俯仰角 Pitch；
 * 绕物体的 X 轴旋转，得到横滚角 Roll。
 */
void EulerAngles_test() {
    /**** 3. 欧拉角 ****/
    std::cout << std::endl << "********** EulerAngle **********" << std::endl;
    //3.0 初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)
    Eigen::Vector3d ea(0.785398, -0, 0);
 
    //3.1 欧拉角转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    std::cout << "rotation matrix3 =\n" << rotation_matrix3 << std::endl;   
 
    //3.2 欧拉角转换为四元数,
    Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                  Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    std::cout << "quaternion3 x: " << quaternion3.x() << std::endl;
    std::cout << "quaternion3 y: " << quaternion3.y() << std::endl;
    std::cout << "quaternion3 z: " << quaternion3.z() << std::endl;
    std::cout << "quaternion3 w: " << quaternion3.w() << std::endl;
 
    //3.3 欧拉角转换为旋转向量
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());  
    std::cout << "rotation_vector3 " << "angle is: " << rotation_vector3.angle() * (180 / M_PI) 
                                << " axis is: " << rotation_vector3.axis().transpose() << std::endl;
}

/**
 * @brief 旋转矩阵（Rotation Matrix）
 * @note 用 9 个量描述旋转的3个自由度，有冗余；旋转矩阵是一个正交矩阵，行列式为1，其逆矩阵等于其转置矩阵。
 */
void RotationMatrix_test() {
    /**** 2. 旋转矩阵 *****/
    std::cout << std::endl << "********** RotationMatrix **********" << std::endl;
    //2.0 旋转矩阵初始化
    Eigen::Matrix3d rotation_matrix2;
    rotation_matrix2 << 0.707107, -0.707107, 0, 0.707107, 0.707107, 0, 0, 0, 1;

    //或直接单位矩阵初始化
    Eigen::Matrix3d rotation_matrix2_1 = Eigen::Matrix3d::Identity();
 
    //2.1 旋转矩阵转换为欧拉角
    //ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
    Eigen::Vector3d euler_angles = rotation_matrix2.eulerAngles(2, 1, 0); 
    std::cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << std::endl;
 
    //2.2 旋转矩阵转换为旋转向量
    Eigen::AngleAxisd rotation_vector2;
    rotation_vector2.fromRotationMatrix(rotation_matrix2);
    //或者
    Eigen::AngleAxisd rotation_vector2_1(rotation_matrix2);
    std::cout << "rotation_vector2 " << "angle is: " << rotation_vector2.angle() * (180 / M_PI) 
                                << " axis is: " << rotation_vector2.axis().transpose() << std::endl;
 
    std::cout << "rotation_vector2_1 " << "angle is: " << rotation_vector2_1.angle() * (180 / M_PI) 
                                  << " axis is: " << rotation_vector2_1.axis().transpose() << std::endl;
 
    //2.3 旋转矩阵转换为四元数
    Eigen::Quaterniond quaternion2(rotation_matrix2);
    //或者
    Eigen::Quaterniond quaternion2_1;
    quaternion2_1 = rotation_matrix2;
    std::cout << "quaternion2 x: " << quaternion2.x() << std::endl;
    std::cout << "quaternion2 y: " << quaternion2.y() << std::endl;
    std::cout << "quaternion2 z: " << quaternion2.z() << std::endl;
    std::cout << "quaternion2 w: " << quaternion2.w() << std::endl;
    
    std::cout << "quaternion2_1 x: " << quaternion2_1.x() << std::endl;
    std::cout << "quaternion2_1 y: " << quaternion2_1.y() << std::endl;
    std::cout << "quaternion2_1 z: " << quaternion2_1.z() << std::endl;
    std::cout << "quaternion2_1 w: " << quaternion2_1.w() << std::endl;
}

/**
 * @brief 四元数（Quaternion）
 * @note 用 4 个量描述旋转的3个自由度，没有冗余；四元数是一个超复数，有实部和虚部，可以用来表示旋转。 
 */
void Quaterniond_test() {
    /**** 4.四元数 ****/
    std::cout << std::endl << "********** Quaternion **********" << std::endl;
    //4.0 初始化四元素,注意eigen Quaterniond类四元数初始化参数顺序为w,x,y,z
    Eigen::Quaterniond quaternion4(0.92388, 0, 0, 0.382683);
    Eigen::Quaterniond quaternion4_1(Eigen::Vector4d(0, 0, 0.382683, 0.92388)); // 顺序为x,y,z,w
    std::cout << "quaternion4.coeffs: " << quaternion4.coeffs().transpose() << std::endl;
    std::cout << "quaternion4.vec: " << quaternion4.vec().transpose() << std::endl;
    std::cout << "quaternion4_1.coeffs: " << quaternion4_1.coeffs().transpose() << std::endl;
    std::cout << "quaternion4_1.vec: " << quaternion4_1.vec().transpose() << std::endl;
 
    //4.1 四元数转换为旋转向量
    Eigen::AngleAxisd rotation_vector4(quaternion4);
    //或者
    Eigen::AngleAxisd rotation_vector4_1;
    rotation_vector4_1 = quaternion4;
    std::cout << "rotation_vector4 " << "angle is: " << rotation_vector4.angle() * (180 / M_PI) 
                                << " axis is: " << rotation_vector4.axis().transpose() << std::endl;
 
    std::cout << "rotation_vector4_1 " << "angle is: " << rotation_vector4_1.angle() * (180 / M_PI) 
                                  << " axis is: " << rotation_vector4_1.axis().transpose() << std::endl;
 
    //4.2 四元数转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix4;
    rotation_matrix4 = quaternion4.matrix();
    Eigen::Matrix3d rotation_matrix4_1;
    rotation_matrix4_1 = quaternion4.toRotationMatrix();
    std::cout << "rotation matrix4 =\n" << rotation_matrix4 << std::endl;
    std::cout << "rotation matrix4_1 =\n" << rotation_matrix4_1 << std::endl;      
 
    //4.4 四元数转欧拉角(Z-Y-X，即RPY)
    Eigen::Vector3d eulerAngle4 = quaternion4.matrix().eulerAngles(2,1,0);
    std::cout << "yaw(z) pitch(y) roll(x) = " << eulerAngle4.transpose() << std::endl;  
}

/**
 * @brief 平移变换（Translation）
 * @note 用 3 个量描述平移的3个自由度，没有冗余；平移变换是一个特殊的仿射变换，只有平移部分，没有旋转部分。
 */
void Translation3d_test() {
    /**** 5.平移变换 ****/
    std::cout << std::endl << "********** Translation **********" << std::endl;
    //5.0 初始化平移向量
    Eigen::Translation3d translation3d(1, 0, 0);
    Eigen::Translation3d translation3d_1(Eigen::Vector3d(1, 0, 0));
    std::cout << "translation3d: " << translation3d.vector().transpose() << std::endl;
    std::cout << "translation3d_1: " << translation3d_1.vector().transpose() << std::endl;
 
    //5.1 点平移
    Eigen::Vector3d point(1.0, 2.0, 3.0);
    Eigen::Vector3d pointTransformed = translation3d * point;
    std::cout << "point: " << point.transpose() << std::endl;
    std::cout << "point after translation: " << pointTransformed.transpose() << std::endl;
}

/**
 * @brief 仿射变换（Affine
 * 
 * @note Affine模式表示一种保持直线平行性质的线性变换，包括平移、旋转、缩放和错切等。
 * 在Affine模式下，可以表示任意线性组合的这些变换，即平移、旋转、缩放、错切的组合。
 * 这种变换不会改变物体之间的相对位置关系，只会改变其大小、形状和方向。
 */
void Affine3d_test() {
  /**** 6.仿射变换 ****/
  std::cout << std::endl << "********** Affine **********" << std::endl;
  //6.0 初始化仿射变换
  // 单位矩阵初始化
  Eigen::Affine3d affine3d = Eigen::Affine3d::Identity();
  std::cout << "affine3d: \n" << affine3d.matrix() << std::endl;
  // 设置平移和旋转矩阵
  Eigen::Affine3d affine3d_1 = Eigen::Affine3d::Identity();
  affine3d_1.translate(Eigen::Vector3d(1, 0, 0));
  affine3d_1.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)));
  std::cout << "affine3d_1: \n" << affine3d_1.matrix() << std::endl;
  // 通过矩阵初始化
  Eigen::Affine3d affine3d_2;
  affine3d_2.matrix() << 0, -1, 0, 1,
                         1, 0, 0, 2,
                         0, 0, 1, 3,
                         0, 0, 0, 1;
  std::cout << "affine3d_2: \n" << affine3d_2.matrix() << std::endl;
  // 平移和旋转
  Eigen::Translation3d translation3d(1, 0, 0);
  Eigen::Quaterniond quaternion(0.92388, 0, 0, 0.382683);
  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
  std::cout << "rotation_matrix: \n" << rotation_matrix << std::endl;
  Eigen::Affine3d affine3d_3 = translation3d * rotation_matrix;
  std::cout << "affine3d_3: \n" << affine3d_3.matrix() << std::endl;

  //6.1 仿射变换的接口
  Eigen::Vector3d translation3d_2 = affine3d_3.translation();
  std::cout << "translation3d_2: " << translation3d_2.transpose() << std::endl;
  Eigen::Quaterniond quaternion_2 = Eigen::Quaterniond(affine3d_3.rotation());
  std::cout << "quaternion_2 : " << quaternion_2.coeffs().transpose() << std::endl;
  Eigen::Matrix3d rotation_matrix_2 = affine3d_3.rotation();
  std::cout << "rotation_matrix_2: \n" << rotation_matrix_2 << std::endl;
  Eigen::Matrix4d matrix4d = affine3d_3.matrix();
  std::cout << "matrix4d: \n" << matrix4d << std::endl;

  //6.2 block组装
  Eigen::Matrix4d matrix4d_1 = Eigen::Matrix4d::Identity();
  matrix4d_1.block<3, 3>(0, 0) = rotation_matrix_2;
  matrix4d_1.block<3, 1>(0, 3) = translation3d_2;
  std::cout << "matrix4d_1: \n" << matrix4d_1 << std::endl;
}

/**
 * @brief 等距变换（Isometry
 * 
 * @note 等距变换是一种特殊的仿射变换，包括平移和旋转，没有缩放和错切。
 * 在这种变换下，物体之间的相对距离和角度关系保持不变。这对于保持形状和大小的不变性很有用，例如在计算机图形学中的刚体变换。
 */
void Isometry3d_test() {

}
 
int main(int argc, char **argv) 
{
  std::cout << "==================== AngleAxisd_test ====================" << std::endl;
  AngleAxisd_test();
  std::cout << "==================== RotationMatrix_test ====================" << std::endl;
  RotationMatrix_test();
  std::cout << "==================== EulerAngles_test ====================" << std::endl;
  EulerAngles_test();
  std::cout << "==================== Quaterniond_test ====================" << std::endl;
  Quaterniond_test();
  std::cout << "==================== Translation3d_test ====================" << std::endl;
  Translation3d_test();
  std::cout << "==================== Affine3d_test ====================" << std::endl;
  Affine3d_test();
  std::cout << "==================== Isometry3d_test ====================" << std::endl;
  Isometry3d_test();
}