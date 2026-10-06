#include <iostream>
#include <cmath>
#include <Eigen/Geometry>

const double l1 =0.4, l2 = 0.3,l3=0.2;


// method 1formulae

Eigen::Vector2d fk_formula(double q1 , double q2){
    return {l1 * std::cos(q1)+ l2* std::cos(q1+q2), l1*std::sin(q1)+l2*std::sin(q1+q2)};
}
Eigen::Vector2d fk_formula_3j(double q1, double q2, double q3){
    return {
        l1*std::cos(q1)+l2*std::cos(q1+q2)+l3*std::cos(q1+q2+q3),
        l1*std::sin(q1)+l2*std::sin(q1+q2)+l3*std::sin(q1+q2+q3)
    };
}

// DH parameter version

Eigen::Isometry3d fk_transform(double q1, double q2){
    //alpha,theta,a,d
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.rotate(Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()));
    t.translate(Eigen::Vector3d(l1,0,0));
    t.rotate(Eigen::AngleAxisd(q2, Eigen::Vector3d::UnitZ()));
    t.translate(Eigen::Vector3d(l2,0,0));
    return t;
}
Eigen::Vector2d fk_2d(double q1, double q2){
    //alpha,theta,a,d
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.rotate(Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()));
    t.translate(Eigen::Vector3d(l1,0,0));
    t.rotate(Eigen::AngleAxisd(q2, Eigen::Vector3d::UnitZ()));
    t.translate(Eigen::Vector3d(l2,0,0));
    return t.translation().head<2>();
}
Eigen::Isometry3d fk_transform_3d(double q1, double q2,double q3){
    //alpha,theta,a,d
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.rotate(Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()));
    t.translate(Eigen::Vector3d(l1,0,0));
    t.rotate(Eigen::AngleAxisd(q2, Eigen::Vector3d::UnitZ()));
    t.translate(Eigen::Vector3d(l2,0,0));
    t.rotate(Eigen::AngleAxisd(q3, Eigen::Vector3d::UnitZ()));
    t.translate(Eigen::Vector3d(l3,0,0));
    return t;
}
Eigen::Matrix2d anal_jacobian(double q1, double q2){
    Eigen::Matrix2d j;
    j<<-l1*std::sin(q1)-l2*std::sin(q1+q2), -l2*std::sin(q1+q2),
        l1*std::cos(q1)+l2*std::cos(q1+q2), l2*std::cos(q1+q2);
    return j;
    
}
Eigen::Matrix2d num_jacobian(double q1, double q2){
    const double h = 1e-6;
    Eigen::Matrix2d j;
    Eigen::Vector2d p = fk_2d(q1,q2);
    j.col(0)= (fk_2d(q1+h,q2)-p)/h;
    j.col(1)= (fk_2d(q1,q2+h)-p)/h;
    return j;

}

int main(){
    const double deg = M_PI/180;
    // double test[][2]={{0,0},{90,0},{-90,0},{30,45}};
    // double q[]={30,45,-30};
    // for(auto &t : test){
    //     double q1 = t[0]*deg,q2 = t[1]*deg;
    //     Eigen::Vector2d a = fk_formula(q1,q2);
    //     Eigen::Isometry3d T = fk_transform(q1,q2);
    //     Eigen::Vector2d T_t = T.translation().head<2>();

    //     //Tool angle R(1,0) and R(0,0)
    //     double tool_angle = std::atan2(T.linear()(1,0),T.linear()(0,0)) /deg;
    //     std::cout<<"a ="<<a<<"\n";
    //     std::cout<<"T_t="<<T_t<<"\n";

    // }
    // Eigen::Vector2d p_n = fk_formula_3j(q[0]*deg,q[1]*deg,q[2]*deg);
    // Eigen::Isometry3d t = fk_transform_3d(q[0]*deg,q[1]*deg,q[2]*deg);
    // Eigen::Vector2d t_t = t.translation().head<2>();
    // std::cout<<"p_n =" <<p_n.transpose()<<"\n";
    // std::cout<<"t_t ="<<t_t.transpose()<<"\n";
    // for (double q1 = -90 ; q1<=90 ; q1+=5){
    //     for(double q2 =-90 ; q2<=90;q2+=5){
    //         Eigen::Isometry3d T = fk_transform(q1 * deg, q2 * deg);
    //         Eigen::Vector2d p = T.translation().head<2>();
    //         std::cout << p.x() << "," << p.y() << "\n";
    //     }
    // }
    //jacobian
    Eigen::Matrix2d j_n = num_jacobian(30*deg, 45*deg);
    Eigen::Matrix2d j_a = anal_jacobian(30*deg, 45*deg);
    for (double q2_deg : {90.0, 45.0, 0.0}) {
        Eigen::Matrix2d J = anal_jacobian(30*deg, q2_deg*deg);
        std::cout << "q2 = " << q2_deg << "  det = " << J.determinant() << "\n";
    }
    std::cout<<j_n<<"\n";
    std::cout<<j_a<<"\n";


}