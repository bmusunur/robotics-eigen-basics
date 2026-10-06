// Lesson 2: Multiply, transpose, inverse, solve, dot/cross
// Robotics lo: v = J * qdot (joint speed -> tool speed), IK lo J inverse, torque = J^T * F
#include <iostream>
#include <Eigen/Dense>

int main()
{
  Eigen::Matrix3d A;
  A << 2, 0, 0,
       0, 3, 0,
       1, 0, 4;
  Eigen::Vector3d x(1, 1, 1);

  // ---------- 1. Matrix * vector ----------
  // Rule: (3x3) * (3x1) = (3x1). Inner sizes match avvali, lekapothe compile error
  Eigen::Vector3d b = A * x;
  std::cout << "A * x = " << b.transpose() << "\n";

  // ---------- 2. Matrix * matrix ----------
  Eigen::Matrix3d AA = A * A;
  std::cout << "A * A =\n" << AA << "\n";

  // Element-wise multiply (matrix multiply KAADU) -> .cwiseProduct()
  std::cout << "A .* A (element-wise) =\n" << A.cwiseProduct(A) << "\n\n";

  // ---------- 3. Transpose ----------
  // Robotics: joint torques = J^T * F (tool daggara force -> prati joint ki torque)
  std::cout << "A^T =\n" << A.transpose() << "\n\n";

  // ---------- 4. Inverse ----------
  Eigen::Matrix3d Ainv = A.inverse();
  std::cout << "A^-1 =\n" << Ainv << "\n";
  std::cout << "A * A^-1 (should be Identity) =\n" << A * Ainv << "\n";
  std::cout << "det(A) = " << A.determinant() << "  (0 aithe inverse ledu = singular)\n\n";

  // ---------- 5. Solve: A x = b  (inverse kante BETTER) ----------
  // Inverse calculate cheyyadam slow + numerically unstable.
  // "x = A^-1 b" kavali ante, direct ga solve() vaadandi.
  Eigen::Vector3d x_solved = A.colPivHouseholderQr().solve(b);
  std::cout << "Solved x from A x = b: " << x_solved.transpose() << " (original: 1 1 1)\n\n";

  // ---------- 6. Pseudo-inverse (non-square matrix ki) ----------
  // 7-DOF arm: Jacobian 6x7 -> square kaadu -> normal inverse ledu -> pseudo-inverse
  Eigen::MatrixXd J(2, 3);
  J << 1, 0, 1,
       0, 1, 1;
  Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
  std::cout << "J (2x3) pseudo-inverse (3x2) =\n" << J_pinv << "\n";
  std::cout << "J * J_pinv (should be 2x2 Identity) =\n" << J * J_pinv << "\n\n";

  // ---------- 7. Dot, cross, norm ----------
  Eigen::Vector3d u(1, 0, 0), v(0, 1, 0);
  std::cout << "u . v (dot)   = " << u.dot(v) << "  (0 = perpendicular)\n";
  std::cout << "u x v (cross) = " << u.cross(v).transpose() << "  (Z axis!)\n";
  Eigen::Vector3d err(0.003, -0.004, 0.0);
  std::cout << "position error norm = " << err.norm() * 1000 << " mm\n";
  std::cout << "u + v normalized = " << (u + v).normalized().transpose() << "\n";

  // TRY: A ni singular cheyyandi (row 3 = row 1 * 2), det & inverse em avtayo chudandi
  return 0;
}
