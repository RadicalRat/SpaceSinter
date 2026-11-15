#include <gtest/gtest.h>

#include <cmath>

#include <sun_pose_finder/dexterity.hpp>
#include <sun_pose_finder/track_raster_with_sun.hpp>

namespace sun_pose_finder
{
TEST(PolylineReference, PointsAndTangentsAlongAZigZag)
{
  const PolylineReference ref({ { 0, 0, 0 }, { 0.04, 0, 0 }, { 0.04, 0.004, 0 }, { 0, 0.004, 0 } });
  EXPECT_NEAR(ref.length(), 0.084, 1e-12);
  EXPECT_TRUE(ref.point(0.02).isApprox(Eigen::Vector3d(0.02, 0, 0)));
  EXPECT_TRUE(ref.point(0.042).isApprox(Eigen::Vector3d(0.04, 0.002, 0)));
  EXPECT_TRUE(ref.point(1.0).isApprox(Eigen::Vector3d(0, 0.004, 0)));   // clamped to the end
  EXPECT_TRUE(ref.point(-1.0).isApprox(Eigen::Vector3d(0, 0, 0)));      // clamped to the start
  EXPECT_TRUE(ref.tangent(0.01).isApprox(Eigen::Vector3d(1, 0, 0)));
  EXPECT_TRUE(ref.tangent(0.042).isApprox(Eigen::Vector3d(0, 1, 0)));
  EXPECT_TRUE(ref.tangent(0.06).isApprox(Eigen::Vector3d(-1, 0, 0)));
}

TEST(PolylineReference, DuplicatePointsAndSinglePoint)
{
  const PolylineReference dup({ { 0, 0, 0 }, { 0, 0, 0 }, { 0.01, 0, 0 } });
  EXPECT_NEAR(dup.length(), 0.01, 1e-12);
  EXPECT_TRUE(dup.tangent(0.0).isApprox(Eigen::Vector3d(1, 0, 0)));
  const PolylineReference one({ { 1, 2, 3 } });
  EXPECT_DOUBLE_EQ(one.length(), 0.0);
  EXPECT_TRUE(one.point(0.5).isApprox(Eigen::Vector3d(1, 2, 3)));
  EXPECT_TRUE(one.tangent(0.0).isZero());
}

TEST(SunCorrection, RotatesTheLensAxisTowardTheSun)
{
  // The sun seen at tan offsets (vx, vy) lies along (vx, vy, 1) in the focal frame (optical axes == focal axes).
  for (const auto& [vx, vy] : { std::pair{ 0.01, 0.0 }, { 0.0, 0.01 }, { -0.007, 0.004 }, { 0.003, -0.009 } })
  {
    const Eigen::Vector3d sun = Eigen::Vector3d(vx, vy, 1.0).normalized();
    const Eigen::Vector3d w = sunErrorToFocalAngularVelocity(vx, vy, 1.0);
    EXPECT_DOUBLE_EQ(w.z(), 0.0);  // roll held
    const double dt = 0.1;
    const Eigen::Vector3d axis_after = Eigen::AngleAxisd(w.norm() * dt, w.normalized()) * Eigen::Vector3d::UnitZ();
    const double before = std::acos(Eigen::Vector3d::UnitZ().dot(sun));
    const double after = std::acos(std::clamp(axis_after.dot(sun), -1.0, 1.0));
    EXPECT_LT(after, before) << "vx " << vx << " vy " << vy;
    EXPECT_NEAR(before - after, w.norm() * dt, 1e-4);  // all of the rotation goes toward the sun
  }
}

TEST(ClampNorm, LimitsOnlyWhenNeeded)
{
  EXPECT_TRUE(clampNorm({ 3, 4, 0 }, 1.0).isApprox(Eigen::Vector3d(0.6, 0.8, 0)));
  EXPECT_TRUE(clampNorm({ 0.1, 0, 0 }, 1.0).isApprox(Eigen::Vector3d(0.1, 0, 0)));
}
TEST(Dexterity, IsotropicSingularAndScaled)
{
  // Identity (with translation scaled to match) is perfectly conditioned.
  Eigen::MatrixXd j = Eigen::MatrixXd::Identity(6, 6);
  j.topRows(3) *= 0.4;
  EXPECT_NEAR(dexterityFromJacobian(j, 0.4), 1.0, 1e-12);
  // A lost direction (column collapse) is singular.
  Eigen::MatrixXd s = j;
  s.col(5) = s.col(4);
  EXPECT_NEAR(dexterityFromJacobian(s, 0.4), 0.0, 1e-12);
  // Halving one direction halves the dexterity.
  Eigen::MatrixXd h = j;
  h(5, 5) = 0.5;
  EXPECT_NEAR(dexterityFromJacobian(h, 0.4), 0.5, 1e-12);
  // Not 6 x n: zero.
  EXPECT_DOUBLE_EQ(dexterityFromJacobian(Eigen::MatrixXd::Identity(5, 5), 0.4), 0.0);
}
}  // namespace sun_pose_finder
