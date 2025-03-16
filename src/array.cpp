#include <iostream>
#include <Eigen/Dense>

int main()
{
  Eigen::ArrayXXf  m(2,2);
  
  // assign some values coefficient by coefficient
  m(0,0) = 1.0; m(0,1) = 2.0;
  m(1,0) = 3.0; m(1,1) = m(0,1) + m(1,0);
  
  // print values to standard output
  std::cout << m << std::endl << std::endl;
 
  // using the comma-initializer is also allowed
  m << 1.0,2.0,
       3.0,4.0;
     
  // print values to standard output
  std::cout << m << std::endl;

  Eigen::Array2d m2;
  m2 << 1.0, 2.0;
  std::cout << "m2: \n" << m2 << std::endl;

  Eigen::Array<double, 3, 2> m3;
  m3 << 1, 2,
  		3, 4,
		5, 6;
  std::cout << "m3: \n" << m3 << std::endl;

  auto ret = m3 * m3;
  std::cout << "ret: \n" << ret << std::endl;

  m3 *= -2;
  std::cout << "m3: \n" << m3 << std::endl;
  std::cout << "m3.abs: \n" << m3.abs() << std::endl;
  std::cout << "m3.sqrt: \n" << m3.sqrt() << std::endl;
}