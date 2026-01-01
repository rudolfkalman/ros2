#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

std::vector<Eigen::Vector2d> Points = {Eigen::Vector2d(-0.990066, -0.502765), Eigen::Vector2d(-0.946230, -0.449131), Eigen::Vector2d(-0.923050, -0.463866),
    Eigen::Vector2d(-0.845967, -0.423427), Eigen::Vector2d(-0.846124, -0.407516), Eigen::Vector2d(-0.794727, -0.360431),
    Eigen::Vector2d(-0.783218, -0.348086), Eigen::Vector2d(-0.727840, -0.319847), Eigen::Vector2d(-0.723345, -0.327545),
    Eigen::Vector2d(-0.664758, -0.271922), Eigen::Vector2d(-0.652772, -0.269313), Eigen::Vector2d(-0.611586, -0.217798),
    Eigen::Vector2d(-0.580298, -0.223027), Eigen::Vector2d(-0.555132, -0.169916), Eigen::Vector2d(-0.507548, -0.171740),
    Eigen::Vector2d(-0.484088, -0.152736), Eigen::Vector2d(-0.450418, -0.120520), Eigen::Vector2d(-0.404360, -0.106692),
    Eigen::Vector2d(-0.374848, -0.072905), Eigen::Vector2d(-0.339986, -0.065370), Eigen::Vector2d(-0.308692, -0.033436),
    Eigen::Vector2d(-0.261322, -0.013986), Eigen::Vector2d(-0.232925, 0.018149), Eigen::Vector2d(-0.197544, 0.033949),
    Eigen::Vector2d(-0.167206, 0.061422), Eigen::Vector2d(-0.127771, 0.087433), Eigen::Vector2d(-0.100848, 0.112879),
    Eigen::Vector2d(-0.058377, 0.126223), Eigen::Vector2d(-0.034330, 0.152678), Eigen::Vector2d(0.009517, 0.172289),
    Eigen::Vector2d(0.037507, 0.194863), Eigen::Vector2d(0.074056, 0.215773), Eigen::Vector2d(0.102762, 0.243694),
    Eigen::Vector2d(0.143857, 0.256642), Eigen::Vector2d(0.179043, 0.279898), Eigen::Vector2d(0.207044, 0.296611),
    Eigen::Vector2d(0.250845, 0.326826), Eigen::Vector2d(0.285383, 0.339882), Eigen::Vector2d(0.313045, 0.368843),
    Eigen::Vector2d(0.353731, 0.377079), Eigen::Vector2d(0.387503, 0.406056), Eigen::Vector2d(0.414120, 0.421939),
    Eigen::Vector2d(0.454254, 0.454354), Eigen::Vector2d(0.489371, 0.464529), Eigen::Vector2d(0.520586, 0.487899),
    Eigen::Vector2d(0.564335, 0.511315), Eigen::Vector2d(0.593220, 0.527446), Eigen::Vector2d(0.627013, 0.556547),
    Eigen::Vector2d(0.665273, 0.565593), Eigen::Vector2d(0.694222, 0.593949), Eigen::Vector2d(0.729257, 0.617268),
    Eigen::Vector2d(0.827804, 0.849781), Eigen::Vector2d(0.881833, 0.799575), Eigen::Vector2d(0.921831, 0.824033),
    Eigen::Vector2d(0.941507, 0.811238), Eigen::Vector2d(1.001164, 0.777141)};

std::vector<Eigen::Vector2d> Rotated_Points;

Eigen::Vector2d Calc_Affine(Eigen::Vector2d Vec2, double Theta, Eigen::Vector2d Displacement){
  Eigen::Matrix3d Affine;
  Affine << std::cos(Theta), -std::sin(Theta), Displacement.x,
            std::sin(Theta),  std::cos(Theta), Displacement.y,
            0              ,  0              , 1;

  Eigen::Vector3d Vec3;
  Vec3 << Vec2.x, Vec2.y, 1;
  Eigen::Vector3d Rotated_Vec3 = Affine * Vec3;
  return Eigen::Vector2d(Rotated_Vec3.x, Rotated_Vec3.y)
}

int main(){

  return 0;
}
