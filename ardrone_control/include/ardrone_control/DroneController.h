#pragma once
#define DRoneController

#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>
#include <chrono>

class DroneController{
public:
  DroneController(double limAngleCamY, double limAngleCamZ, double Kx, double Ky, double Kz,double Kix, double Kiy, double Kiz, double YSizeCam, double ZSizeCam);
  std::vector<float> getVectorControl(std::vector<double> xyzrObject, double alphaDrone, double phiDrone, std::vector<double> desiredDiffVector,std::vector<double> desiredVector);
private:
  double _limAngleCamY;
  double _limAngleCamZ;
  double _Kx;
  double _Ky;
  double _Kz;
  double _Kix;
  double _Kiy;
  double _Kiz;
  double _alpha0;
  double _ball_radius;
  double _YSizeCam;
  double _ZSizeCam;
  double _xOffset;
  double _lastTime=0;
  Eigen::Vector3d _integralError;
  Eigen::Vector3d _lastPosition;
  Eigen::Matrix3d _controlMatrix;
  Eigen::Matrix3d _controlIntegralMatrix;
  Eigen::Vector3d _maxControlVector;
  Eigen::Vector3d _getCurrentVector(Eigen::Vector4d xyzrObject, double alphaDrone, double phiDrone);
  Eigen::Matrix3d _getTransformRotMatrix(double alpha, double phi);
};
