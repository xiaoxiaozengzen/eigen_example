#include <iostream>
#include <Eigen/Dense>

// typedef Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options> MyMatrixType;
// 1.Scalar is the scalar type of the coefficients (e.g., float, double, bool, int, etc.).
// 2.RowsAtCompileTime and ColsAtCompileTime are the number of rows and columns of the matrix as known at compile-time or Dynamic.
// 3.Options can be ColMajor or RowMajor, default is ColMajor. (see class Matrix for more options)，在内存中行列存储的顺序

// Examples:
// Matrix<double, 6, Dynamic>                  // Dynamic number of columns (heap allocation)
// Matrix<double, Dynamic, 2>                  // Dynamic number of rows (heap allocation)
// Matrix<double, Dynamic, Dynamic, RowMajor>  // Fully dynamic, row major (heap allocation)
// Matrix<double, 13, 3>                       // Fully fixed (usually allocated on stack)

// typedef Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> MatrixType;
// Matrix<float,Dynamic,Dynamic>   <=>   MatrixXf
// Matrix<double,Dynamic,1>        <=>   VectorXd
// Matrix<int,1,Dynamic>           <=>   RowVectorXi
// Matrix<float,3,3>               <=>   Matrix3f
// Matrix<float,4,1>               <=>   Vector4f

/******************************************************************************* */
/* 注意： */
/* 1. Eigen的矩阵是列优先存储的，即先存储第一列的数据，再存储第二列的数据，以此类推 */
/* 2. Eigen的矩阵是从0开始的 */
/* 3. 不要使用auto，可能导致类型推断错误 */
/******************************************************************************* */

void ConFun() {
    // 一维
    Eigen::Vector4d v4;
    Eigen::Vector2d v1(3, 4);

    // 二维
    Eigen::Matrix3d m1;
    Eigen::Matrix<double, 3, 3> m2;
    Eigen::MatrixXd m3(2, 3); // 2行3列
}

void Init() {
    // 一维
    Eigen::Vector4d v1;
    v1 << 1, 2, 3, 4;
    Eigen::VectorXd v2(3);
    v2 << 1, 2, 3;

    // 二维
    Eigen::Matrix3d m1;
    m1 << 1, 2, 3,
          4, 5, 6,
          7, 8, 9;
    std::cout << m1 << std::endl;
}

void Runtime() {
    // 一维
    Eigen::VectorXd v1(3);
    v1 << 1, 2, 3;
    std::cout << "v1.size(): " << v1.size() << std::endl;
    std::cout << "v1.rows(): " << v1.rows() << std::endl;
    std::cout << "v1.cols(): " << v1.cols() << std::endl;
    double* ptr = v1.data();
    std::cout << "ptr: " << *ptr << std::endl;
    std::cout << "ptr+1: " << *(ptr+1) << std::endl;

    // 二维
    Eigen::MatrixXd m1(2, 3);
    m1 << 1, 2, 3,
          4, 5, 6;
    std::cout << "m1.size(): " << m1.size() << std::endl;
    std::cout << "m1.rows(): " << m1.rows() << std::endl;
    std::cout << "m1.cols(): " << m1.cols() << std::endl;
    std::cout << "m1.innerSize: " << m1.innerSize() << std::endl;
    std::cout << "m1.outerSize: " << m1.outerSize() << std::endl;
    double *ptr2 = m1.data();
    std::cout << "ptr2: " << *ptr2 << std::endl;
    std::cout << "ptr2+1: " << *(ptr2+1) << std::endl;
    std::cout << "ptr2+2: " << *(ptr2+2) << std::endl;
    std::cout << "ptr2+3: " << *(ptr2+3) << std::endl;

    // resize
    // 如果满足之前大小则不调整，否则会丢失数据
    m1.resize(3, 3);
    std::cout << "m1: \n" << m1 << std::endl;
    m1.resize(4, Eigen::NoChange);
    std::cout << "m1: \n" << m1 << std::endl;
}

void Access() {
    // 一维
    Eigen::VectorXd v1(3);
    v1 << 1, 2, 3;

    double ret = v1(1);
    std::cout << "ret: " << ret << std::endl;
    ret = v1[1];
    std::cout << "ret: " << ret << std::endl;
    ret = v1.x();
    std::cout << "ret: " << ret << std::endl;
    v1.x() = 10;
    std::cout << "v1: " << v1 << std::endl;

    Eigen::Vector2d v2 = v1.head<2>();
    std::cout << "v2: " << v2 << std::endl;
    Eigen::Vector2d v3 = v1.head(2);
    std::cout << "v3: " << v3 << std::endl;
    Eigen::Vector2d v4 = v1.tail<2>();
    std::cout << "v4: " << v4 << std::endl;
    Eigen::Vector2d v5 = v1.segment(1, 2);
    std::cout << "v5: " << v5 << std::endl;
    Eigen::VectorXd v6 = v1.segment<2>(1);
    std::cout << "v6: " << v6 << std::endl;

    // 二维
    Eigen::MatrixXd m1(2, 3);
    m1 << 1, 2, 3,
          4, 5, 6;
    m1(1, 1) = 10;
    std::cout << "m1: \n" << m1 << std::endl;
    ret = m1(0, 1);
    std::cout << "ret: " << ret << std::endl;
}

void Predefined() {
    // fixed size
    Eigen::Matrix3d m1 = Eigen::Matrix3d::Identity();
    std::cout << "Identity: \n" << m1 << std::endl;
    Eigen::Matrix3d m2 = Eigen::Matrix3d::Zero();
    std::cout << "Zero: \n" << m2 << std::endl;
    Eigen::Matrix3d m3 = Eigen::Matrix3d::Ones();
    std::cout << "Ones: \n" << m3 << std::endl;
    Eigen::Matrix3d m4 = Eigen::Matrix3d::Random();
    std::cout << "Random: \n" << m4 << std::endl;
    Eigen::Matrix3d m5 = Eigen::Matrix3d::Constant(2);
    std::cout << "Constant: \n" << m5 << std::endl;
    m1.setZero();
    std::cout << "setZero: \n" << m1 << std::endl;

    // dynamic size
    Eigen::MatrixXd m6 = Eigen::MatrixXd::Random(2, 3);
    std::cout << "Random: \n" << m6 << std::endl;
    Eigen::MatrixXd m7 = Eigen::MatrixXd::Constant(2, 3, 2); // 2行3列，值为2
    std::cout << "Constant: \n" << m7 << std::endl;
    Eigen::MatrixXd m8 = Eigen::MatrixXd::Zero(2, 3);
    std::cout << "Zero: \n" << m8 << std::endl;

    // 单位
    Eigen::Vector3d v1 = Eigen::Vector3d::UnitX();
    std::cout << "UnitX: \n" << v1 << std::endl;
    Eigen::Vector3d v2 = Eigen::Vector3d::UnitY();
    std::cout << "UnitY: \n" << v2 << std::endl;
    Eigen::Vector3d v3 = Eigen::Vector3d::UnitZ();
    std::cout << "UnitZ: \n" << v3 << std::endl;
    Eigen::Vector4d v4 = Eigen::Vector4d::Unit(2);
    std::cout << "Unit(2): \n" << v4 << std::endl;
    v1.setUnit(1);
    std::cout << "setUnit(1): \n" << v1 << std::endl;
    Eigen::MatrixXd m9(2, 3);
    m9.setIdentity();
    std::cout << "setIdentity: \n" << m9 << std::endl;
    Eigen::VectorXd::Unit(3, 1);
    std::cout << "Unit(3, 1): \n" << Eigen::VectorXd::Unit(3, 1) << std::endl;
}

void Math() {

    Eigen::MatrixXd m1(2, 3);
    m1 << 1, 2, 3,
          4, 5, 6;
    Eigen::MatrixXd m2(2, 3);
    m2 << 1, 2, 3,
          4, 5, 6;
    // add
    Eigen::MatrixXd m3 = m1 + m2;
    std::cout << "m3: \n" << m3 << std::endl;
    // sub
    Eigen::MatrixXd m4 = m1 - m2;
    std::cout << "m4: \n" << m4 << std::endl;
    // sclar multiply
    Eigen::MatrixXd m5 = m1 * 2;
    std::cout << "m5: \n" << m5 << std::endl;
    // sclar divide
    Eigen::MatrixXd m6 = m1 / 2;
    std::cout << "m6: \n" << m6 << std::endl;
    // matrix multiply
    Eigen::MatrixXd m7(3, 2);
    m7 << 1, 2,
          3, 4,
          5, 6;
    Eigen::MatrixXd m8 = m1 * m7;
    std::cout << "m8: \n" << m8 << std::endl;
    // transpose
    Eigen::MatrixXd m9 = m1.transpose();
    std::cout << "m9: \n" << m9 << std::endl;
    Eigen::MatrixXd m10 = m1.adjoint();
    std::cout << "m10: \n" << m10 << std::endl;
    m1.transposeInPlace();
    std::cout << "m1: \n" << m1 << std::endl;
    // dot，计算的是向量的点积，点积=|v1|*|v2|*cosθ
    Eigen::VectorXd v1(3);
    v1 << 1, 2, 3;
    Eigen::VectorXd v2(3);
    v2 << 4, 5, 6;
    double ret = v1.dot(v2);
    std::cout << "ret: " << ret << std::endl;
    // norm，计算的到的是向量的模
    ret = v1.norm();
    std::cout << "ret: " << ret << std::endl;
    // eulerAngles
    Eigen::Matrix3d m11;
    m11 << 0.707107, -0.707107, 0,
           0.707107, 0.707107, 0,
           0, 0, 1;
    Eigen::Vector3d v3 = m11.eulerAngles(2, 1, 0);
    std::cout << "v3: \n" << v3 << std::endl;
    // 插值
    Eigen::VectorXd v4(3);
    v4 << 1, 2, 3;
    std::cout << "v4: \n" << v4 << std::endl;
    Eigen::VectorXd v5(3);
    v5 << 4, 5, 6;
    double alpha = 0.5;
    Eigen::VectorXd v6 = (1 - alpha) * v4 + alpha * v5;
    std::cout << "v6: \n" << v6 << std::endl;
    // rowwise：返回一个行迭代器，可以对每一行进行操作
    Eigen::MatrixXd m12(2, 3);
    m12 << 1, 2, 3,
           4, 5, 6;
    Eigen::VectorXd v7(3);
    v7 << 10, 20, 30;
    Eigen::MatrixXd m13 = m12.rowwise() + v7.transpose(); // 每一行加 v
    std::cout << "m13: \n" << m13 << std::endl;
    // colwise：返回一个列迭代器，可以对每一列进行操作
    Eigen::VectorXd v8(2);
    v8 << 10, 20;
    Eigen::MatrixXd m14 = m12.colwise() + v8; // 每一列加 v
    std::cout << "m14: \n" << m14 << std::endl;
}

/**
 * @brief block
 * @note 一般Eigen通过调用.block()方法进行分块运算。生成一个从第i行j列(i,j)开始，具有p行q列(p,q)的块可以有如下的两种形式:
 */
void Block() {
    Eigen::Matrix4d m1;
    m1 << 1, 2, 3, 4,
          5, 6, 7, 8,
          9, 10, 11, 12,
          13, 14, 15, 16;
    // dynamic-size
    Eigen::MatrixXd m2 = m1.block(1, 1, 2, 2);
    std::cout << "m2: \n" << m2 << std::endl;
    // fixed-size
    Eigen::Matrix2d m3 = m1.block<2, 2>(0, 0);
    std::cout << "m3: \n" << m3 << std::endl;
    // oversized，超出范围的给随机值
    Eigen::Matrix3d m4 = m1.block(2, 2, 3, 3);
    std::cout << "m4: \n" << m4 << std::endl;
    // adjust
    Eigen::Matrix4d m5 = m1;
    m5.block(1, 1, 3, 3) = Eigen::Matrix3d::Identity();
    std::cout << "m5: \n" << m5 << std::endl;
    Eigen::Matrix<double, 3, 2> m6;
    m6 << 0, 0,
          0, 0,
          0, 0;
    m5.block<3, 2>(0, 0) = m6;
    std::cout << "m5: \n" << m5 << std::endl;
}

int main()
{
    std::cout << "================ConFun================" << std::endl;
    ConFun();
    std::cout << "================Init================" << std::endl;
    Init();
    std::cout << "================Runtime================" << std::endl;
    Runtime();
    std::cout << "================Access================" << std::endl;
    Access();
    std::cout << "================Predefined================" << std::endl;
    Predefined();
    std::cout << "================Math================" << std::endl;
    Math();
    std::cout << "================Block================" << std::endl;
    Block();
    return 0;
}