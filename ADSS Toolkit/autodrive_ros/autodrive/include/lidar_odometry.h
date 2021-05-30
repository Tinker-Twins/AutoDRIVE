#ifndef LIDAR_Odometry_H
#define LIDAR_Odometry_H

// Standard headers
#include <iostream>
#include <fstream>
#include <numeric>

// ROS headers
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

namespace lidar_odometry {

template <typename T>
inline T sign(const T x) { return x<T(0) ? -1:1; }

template <typename Derived>
inline typename Eigen::MatrixBase<Derived>::Scalar
getYaw(const Eigen::MatrixBase<Derived>& r)
{
  return std::atan2( r(1, 0), r(0, 0) );
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> matrixRollPitchYaw(const T roll,
                                                 const T pitch,
                                                 const T yaw)
{
  const Eigen::AngleAxis<T> ax = Eigen::AngleAxis<T>(roll,  Eigen::Matrix<T, 3, 1>::UnitX());
  const Eigen::AngleAxis<T> ay = Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY());
  const Eigen::AngleAxis<T> az = Eigen::AngleAxis<T>(yaw,   Eigen::Matrix<T, 3, 1>::UnitZ());

  return (az * ay * ax).toRotationMatrix().matrix();
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> matrixYaw(const T yaw)
{
  return matrixRollPitchYaw<T>(0, 0, yaw);
}

class LIDAR_Odometry
{
public:

  using Scalar = float;
  using Pose2d = Eigen::Isometry2d;
  using Pose3d = Eigen::Isometry3d;
  using MatrixS31 = Eigen::Matrix<Scalar, 3, 1>;
  using IncrementCov = Eigen::Matrix<Scalar, 3, 3>;

  LIDAR_Odometry();
  virtual ~LIDAR_Odometry() = default;

  void init(const sensor_msgs::LaserScan& scan,
            const geometry_msgs::Pose& initial_robot_pose);

  bool is_initialized();

  bool odometryCalculation(const sensor_msgs::LaserScan& scan);

  void setLaserPose(const Pose3d& laser_pose);

  const Pose3d& getIncrement() const;

  const IncrementCov& getIncrementCovariance() const;

  Pose3d& getPose();
  const Pose3d& getPose() const;

protected:

  bool verbose, module_initialized, first_laser_scan;

  // Internal data
  std::vector<Eigen::MatrixXf> range;
  std::vector<Eigen::MatrixXf> range_old;
  std::vector<Eigen::MatrixXf> range_inter;
  std::vector<Eigen::MatrixXf> range_warped;
  std::vector<Eigen::MatrixXf> xx;
  std::vector<Eigen::MatrixXf> xx_inter;
  std::vector<Eigen::MatrixXf> xx_old;
  std::vector<Eigen::MatrixXf> xx_warped;
  std::vector<Eigen::MatrixXf> yy;
  std::vector<Eigen::MatrixXf> yy_inter;
  std::vector<Eigen::MatrixXf> yy_old;
  std::vector<Eigen::MatrixXf> yy_warped;
  std::vector<Eigen::MatrixXf> transformations;

  Eigen::MatrixXf range_wf;
  Eigen::MatrixXf dtita;
  Eigen::MatrixXf dt;
  Eigen::MatrixXf rtita;
  Eigen::MatrixXf normx, normy, norm_ang;
  Eigen::MatrixXf weights;
  Eigen::MatrixXi null;
  Eigen::MatrixXf A,Aw;
  Eigen::MatrixXf B,Bw;

  MatrixS31 Var;	// Three unknowns: vx, vy, w
  IncrementCov cov_odo;

  float fps;								// In Hz
  float fovh;								// Horizontal FOV
  unsigned int cols;
  unsigned int cols_i;
  unsigned int width;
  unsigned int ctf_levels;
  unsigned int image_level, level;
  unsigned int num_valid_range;
  unsigned int iter_irls;
  float g_mask[5];
  double lin_speed, ang_speed;

  ros::WallDuration	m_runtime;
  ros::Time last_odom_time, current_scan_time;

  MatrixS31 kai_abs_;
  MatrixS31 kai_loc_;
  MatrixS31 kai_loc_old_;
  MatrixS31 kai_loc_level_;

  Pose3d last_increment_;
  Pose3d laser_pose_on_robot_;
  Pose3d laser_pose_on_robot_inv_;
  Pose3d laser_pose_;
  Pose3d laser_oldpose_;
  Pose3d robot_pose_;
  Pose3d robot_oldpose_;

  bool test;
  std::vector<double> last_m_lin_speeds;
  std::vector<double> last_m_ang_speeds;

  // Methods
  void createImagePyramid();
  void calculateCoord();
  void performWarping();
  void calculateRangeDerivativesSurface();
  void computeNormals();
  void computeWeights();
  void findNullPoints();
  void solveSystemOneLevel();
  void solveSystemNonLinear();
  bool filterLevelSolution();
  void PoseUpdate();
  void Reset(const Pose3d& ini_pose);
};

} /* namespace lidar_odometry */

#endif // LIDAR_Odometry_H
