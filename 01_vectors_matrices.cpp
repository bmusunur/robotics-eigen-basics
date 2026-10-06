// Lesson 1: Vectors & Matrices create cheyyadam, access cheyyadam
// Robotics lo: joint angles = vector, rotation = 3x3 matrix, Jacobian = 6xN matrix
#include <iostream>
#include <Eigen/Dense>   // Eigen lo almost anni (Matrix, Vector, inverse, solve) ee okka header lo

int main()
{
  // ---------- 1. Fixed-size vectors (size compile time lo telusu -> FAST) ----------
  Eigen::Vector3d p(1.0, 2.0, 3.0);   // "3d" = 3 elements, double type. Oka 3D point (x, y, z)
  std::cout << "p =\n" << p << "\n\n";  // Eigen vectors COLUMN vectors (nilu vuga print avtayi)

  std::cout << "p.x() = " << p.x() << ", p(1) = " << p(1) << "\n";  // index 0 nunchi start
  p(2) = 10.0;                                                       // value maarchadam
  std::cout << "p after p(2)=10:  " << p.transpose() << "\n";      // transpose -> okka line lo print

  // ---------- 2. Dynamic-size vector (size run time lo decide) ----------
  // Joints count URDF nunchi vastundi -> compile time lo teliyadu -> VectorXd
  Eigen::VectorXd q(6);                 // 6-DOF arm joint angles
  q << 0.0, -0.5, 1.0, 0.0, 0.3, 0.0;   // "<<" = comma initializer, values fill cheyyadam
  std::cout << "q (joint angles) = " << q.transpose() << "\n";
  std::cout << "q.size() = " << q.size() << "\n\n";
  std::cout << "q = " << q << "\n\n";

  // ---------- 3. Matrices ----------
  Eigen::Matrix3d A;                    // 3x3 double
  A << 1, 2, 3,
       4, 5, 6,
       7, 8, 10;
  std::cout << "A =\n" << A << "\n";
  std::cout << "A(1,2) [row 1, col 2] = " << A(1, 2) << "\n";
  std::cout << "A.rows()=" << A.rows() << " A.cols()=" << A.cols() << "\n\n";

  // Jacobian of a 6-DOF arm = 6 rows (vx,vy,vz,wx,wy,wz) x 6 cols (joints)
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
  std::cout << "J (6x6 zeros) size: " << J.rows() << "x" << J.cols() << "\n\n";

  // ---------- 4. Special matrices ----------
  std::cout << "Identity 3x3:\n" << Eigen::Matrix3d::Identity() << "\n";
  std::cout << "Ones vector: " << Eigen::Vector3d::Ones().transpose() << "\n";
  std::cout << "Unit Z axis: " << Eigen::Vector3d::UnitZ().transpose() << "\n\n";

  // ---------- 5. Block access (big matrix lo chinna part) ----------
  // Transform matrix (4x4) lo top-left 3x3 = rotation, top-right 3x1 = translation
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 1>(0, 3) = Eigen::Vector3d(0.5, 0.0, 0.2);  // (row 0, col 3) nunchi 3x1 block
  std::cout << "T with translation:\n" << T << "\n";
  std::cout << "Translation part: " << T.block<3, 1>(0, 3).transpose() << "\n";

  // TRY: q.head(3) (first 3 joints), q.tail(2), A.row(0), A.col(2) print cheyyandi
  
  return 0;
}
