// Mini project: 2-link planar arm Forward Kinematics, rendu methods lo
// (Mana simple_hw robot laage: link1 = 0.4m, link2 = 0.3m)
// Week chivari lo idi meeru SOLO ga raayagalagali -> appudu Step 1 complete.
#include <iostream>
#include <cmath>
#include <Eigen/Geometry>

const double L1 = 0.4, L2 = 0.3;

// Method A: textbook formula (trigonometry)
Eigen::Vector2d fk_formula(double q1, double q2)
{
  return {L1 * std::cos(q1) + L2 * std::cos(q1 + q2),
          L1 * std::sin(q1) + L2 * std::sin(q1 + q2)};
}

// Method B: transforms chain (6-DOF arm ki scale avvedi idhe!)
// Prati joint: "joint chuttu rotate" -> "link length translate"
Eigen::Isometry3d fk_transforms(double q1, double q2)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()));  // joint 1
  T.translate(Eigen::Vector3d(L1, 0, 0));                     // link 1
  T.rotate(Eigen::AngleAxisd(q2, Eigen::Vector3d::UnitZ()));  // joint 2
  T.translate(Eigen::Vector3d(L2, 0, 0));                     // link 2
  return T;
}

int main()
{
  const double deg = M_PI / 180.0;
  double tests[][2] = {{0, 0}, {90, 0}, {30, 45}, {45, -90}};

  for (auto & t : tests) {
    double q1 = t[0] * deg, q2 = t[1] * deg;
    Eigen::Vector2d a = fk_formula(q1, q2);
    Eigen::Isometry3d T = fk_transforms(q1, q2);
    Eigen::Vector2d b = T.translation().head<2>();

    // Tool orientation = q1 + q2 (Z chuttu total rotation)
    // atan2(R(1,0), R(0,0)) = Z chuttu angle, sign tho saha (AngleAxis angle eppudu positive)
    double tool_angle = std::atan2(T.linear()(1, 0), T.linear()(0, 0)) / deg;

    std::cout << "q = (" << t[0] << ", " << t[1] << ") deg\n"
              << "  formula    : " << a.transpose() << "\n"
              << "  transforms : " << b.transpose() << "\n"
              << "  match? " << (a.isApprox(b, 1e-9) ? "YES" : "NO")
              << "   tool angle = " << tool_angle << " deg\n\n";
  }

  // CHALLENGE (Step 4 ki preparation):
  // 1. Workspace: q1, q2 ni -90..90 loop chesi tool positions print cheyyandi (matplotlib lo plot)
  // 2. Numerical Jacobian: q1 ni chinna ga (1e-6) maarchi, position entha maarindo divide cheyyandi
  //    J(:,0) = (fk(q1+h, q2) - fk(q1, q2)) / h
  return 0;
}
