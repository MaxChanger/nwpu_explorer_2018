
//=================================================================================================

#ifndef drawinterface_h__
#define drawinterface_h__

#include <Eigen/Core>
#include <string>

class DrawInterface{
public:
  virtual void setNamespace(const std::string& ns) = 0;

  virtual void drawPoint(const Eigen::Vector2f& pointWorldFrame) = 0;
  virtual void drawArrow(const Eigen::Vector3f& poseWorld) = 0;
  virtual void drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& cov) = 0;
  virtual void drawCovariance(const Eigen::Vector3f& mean, const Eigen::Matrix3f& covMatrix) = 0;

  virtual void setScale(double scale) = 0;
  virtual void setColor(double r, double g, double b, double a = 1.0) = 0;

  virtual void sendAndResetData() = 0;
};

#endif
