// Lesson 3: Rotations - 4 ways to represent same rotation
// Rotation matrix, Angle-Axis, Quaternion, Euler (roll-pitch-yaw)
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>  // rotations & transforms ikkada (Dense lo kuda vastundi, explicit ga pettam)

int main()
{
  const double deg = M_PI / 180.0;

  // ---------- 1. Rotation matrix by hand: Z axis chuttu 90 degrees ----------
  double th = 90 * deg;
  Eigen::Matrix3d Rz;
  Rz << std::cos(th), -std::sin(th), 0,
        std::sin(th),  std::cos(th), 0,
        0,             0,            1;
  std::cout << "Rz(90) =\n" << Rz << "\n";

  // X axis point (1,0,0) ni Z chuttu 90 tippithe -> Y axis (0,1,0) avvali
  Eigen::Vector3d p(0, 1, 0);
  std::cout << "Rz * (1,0,0) = " << (Rz * p).transpose() << "\n\n";

  // ---------- 2. Rotation matrix properties (interview question!) ----------
  // R^T * R = I  and  det(R) = +1  ->  so R inverse = R transpose (cheap!)
  std::cout << "Rz^T * Rz =\n" << Rz.transpose() * Rz << "\n";
  std::cout << "det(Rz) = " << Rz.determinant() << "\n\n";

  // ---------- 3. Angle-Axis: "ee axis chuttu inni degrees" ----------
  Eigen::AngleAxisd aa(90 * deg, Eigen::Vector3d::UnitX());
  Eigen::Matrix3d R_from_aa = aa.toRotationMatrix();
  std::cout << "AngleAxis -> matrix (same as RX):\n" << R_from_aa << "\n\n";

  // ---------- 4. Quaternion: ROS lo default (geometry_msgs/Quaternion) ----------
  // 4 numbers (x,y,z,w). Gimbal lock ledu, interpolation (slerp) easy
  Eigen::Quaterniond q(aa);
  std::cout << "Quaternion (x y z w) = " << q.coeffs().transpose() << "\n";
  std::cout << "q.norm() = " << q.norm() << "  (rotation quaternion eppudu unit length)\n";
  std::cout << "q * (0,1,0) = " << (q * p).transpose() << "\n\n";

  // ---------- 5. Euler / RPY: humans ki easy, math ki kaadu ----------
  // Roll (X), Pitch (Y), Yaw (Z). ROS convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  double roll = 10 * deg, pitch = 20 * deg, yaw = 30 * deg;
  Eigen::Matrix3d R_rpy =
      (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();

  Eigen::Vector3d ypr = R_rpy.eulerAngles(2, 1, 0);  // back to angles (Z, Y, X order)
  std::cout << "Back to yaw/pitch/roll (deg) = " << (ypr / deg).transpose() << "\n\n";

  // ---------- 6. Order MATTERS (rotations commute avvavu) ----------
  Eigen::Matrix3d Rx = Eigen::AngleAxisd(90 * deg, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Matrix3d Ry = Eigen::AngleAxisd(90 * deg, Eigen::Vector3d::UnitY()).toRotationMatrix();
  std::cout << "Rx*Ry == Ry*Rx ? " << ((Rx * Ry).isApprox(Ry * Rx) ? "yes" : "NO") << "\n\n";

  // ---------- 7. Slerp: rendu orientations madhya smooth ga ----------
  // Gripper ni smooth ga tippadaniki (trajectory lo vaadataru)
  Eigen::Quaterniond q_start = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_end(Eigen::AngleAxisd(90 * deg, Eigen::Vector3d::UnitZ()));
  for (double t : {0.0, 0.5, 1.0}) {
    Eigen::AngleAxisd mid(q_start.slerp(t, q_end));
    std::cout << "slerp t=" << t << " -> angle " << mid.angle() / deg << " deg\n";
  }

  // TRY: Rx(90) * (0,1,0) ela avtundi? Paper meeda guess chesi, taruvata run cheyyandi
  Eigen::Matrix3d R_x = Eigen::AngleAxisd(90*deg, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Vector3d p_x(0,1,0);
  Eigen::Vector3d p_r = R_x * p_x;
  std::cout<<p_r.transpose()<<"\n";
  return 0;
}
