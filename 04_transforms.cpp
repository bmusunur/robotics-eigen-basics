// Lesson 4: Homogeneous transforms (rotation + translation together)
// Mee harvesting project lo chesindi idhe: camera lo point -> robot base frame
#include <iostream>
#include <cmath>
#include <Eigen/Geometry>

int main()
{
  const double deg = M_PI / 180.0;

  // Notation: T_base_cam = "camera frame ni BASE frame lo describe chestundi"
  // Use: p_base = T_base_cam * p_cam   (inner names match: cam-cam)

  // ---------- 1. Camera pose in base frame ----------
  // Camera base nunchi x=0.5m, z=0.8m daggara undi, kindaki chustundi (X chuttu 180 deg)
  Eigen::Isometry3d T_base_cam = Eigen::Isometry3d::Identity();  // Isometry = rotation + translation only
  T_base_cam.translation() = Eigen::Vector3d(0.5, 0.0, 0.8);
  T_base_cam.linear() = Eigen::AngleAxisd(180 * deg, Eigen::Vector3d::UnitX()).toRotationMatrix();

  std::cout << "T_base_cam (4x4) =\n" << T_base_cam.matrix() << "\n\n";

  // ---------- 2. Camera chusina apple -> base frame ----------
  // Camera frame lo apple: 10cm right, 60cm mundu (camera Z = depth)
  Eigen::Vector3d p_cam(0.1, 0.0, 0.6);
  Eigen::Vector3d p_base = T_base_cam * p_cam;
  std::cout << "Apple in camera frame: " << p_cam.transpose() << "\n";
  std::cout << "Apple in base frame  : " << p_base.transpose() << "\n";
  std::cout << "  (z = 0.8 - 0.6 = 0.2m -> table meeda, correct!)\n\n";

  // ---------- 3. Inverse transform: base -> camera ----------
  Eigen::Isometry3d T_cam_base = T_base_cam.inverse();
  std::cout << "Back to camera frame : " << (T_cam_base * p_base).transpose() << "\n\n";

  // ---------- 4. Chaining: base -> link1 -> tool ----------
  // T_base_tool = T_base_link1 * T_link1_tool   (madhyalo names cancel avtayi)
  Eigen::Isometry3d T_base_link1 = Eigen::Isometry3d::Identity();
  T_base_link1.translate(Eigen::Vector3d(0, 0, 0.3));               // 30cm paiki
  T_base_link1.rotate(Eigen::AngleAxisd(45 * deg, Eigen::Vector3d::UnitZ()));  // 45 deg tirigindi

  Eigen::Isometry3d T_link1_tool = Eigen::Isometry3d::Identity();
  T_link1_tool.translate(Eigen::Vector3d(0.4, 0, 0));               // link1 X lo 40cm

  Eigen::Isometry3d T_base_tool = T_base_link1 * T_link1_tool;
  std::cout << "Tool position in base: " << T_base_tool.translation().transpose() << "\n";
  std::cout << "  (expected: 0.4*cos45 = 0.283, 0.4*sin45 = 0.283, 0.3)\n\n";

  // ---------- 5. Point vs direction (common bug!) ----------
  // Point ki translation apply avvali. Direction/velocity ki rotation matrame.
  Eigen::Vector3d v_cam(0, 0, 1);  // camera forward direction
  std::cout << "Direction in base (rotation only): "
            << (T_base_cam.linear() * v_cam).transpose() << "  -> kindaki\n";

  // TRY: camera ni X chuttu 180 kaakunda 150 deg (konchem mundu ki tilt) pettandi.
  //      Apple base position ela maarutundo chudandi.
  return 0;
}
