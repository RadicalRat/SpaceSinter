#include <gtest/gtest.h>

#include <cmath>

#include <algorithm>

#include <sun_pose_finder/dexterity.hpp>
#include <sun_pose_finder/raster_scan_sim.hpp>
#include <sun_pose_finder/solar_position.hpp>
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

TEST(SunFeedforward, MatchesTheEphemerisMotion)
{
  // Rotating the sun direction by the feedforward for 60 s must land on the ephemeris sun 60 s later.
  const double lat = 39.7555, lon = -105.2211, heading = 125.0;
  const Eigen::Vector3d w = sunFeedforwardWorld(lat, heading);
  EXPECT_NEAR(w.norm(), 2.0 * M_PI / 86400.0, 1e-12);  // 15 deg/h
  for (const double t0 : { 1793740000.0, 1793755000.0, 1718841600.0 + 18 * 3600.0 })
  {
    const Eigen::Vector3d s0 = sunDirection(computeSolarPosition(t0, lat, lon), heading);
    const Eigen::Vector3d s1 = sunDirection(computeSolarPosition(t0 + 60.0, lat, lon), heading);
    const Eigen::Vector3d pred = Eigen::AngleAxisd(w.norm() * 60.0, w.normalized()) * s0;
    const double moved = std::acos(std::clamp(s0.dot(s1), -1.0, 1.0));
    const double miss = std::acos(std::clamp(pred.dot(s1), -1.0, 1.0));
    EXPECT_GT(moved, 1e-3);         // the sun moved ~0.25 deg
    EXPECT_LT(miss, 0.02 * moved);  // predicted to within 2 % (the sun's own drift along the ecliptic is ~0.3 %)
  }
  // Heading error: a 10 deg error leaves under 15 % of the rate to the feedback loop.
  const Eigen::Vector3d off = sunFeedforwardWorld(lat, heading + 10.0);
  EXPECT_LT((off - w).norm() / w.norm(), 0.15);
}

namespace
{
// Toy 6-DOF "robot": q[0..2] = focal position, q[3..5] = rotation vector. J = identity (rows [v; w]) only
// approximately for the rotation part, which the Newton steps absorb.
Eigen::Isometry3d toyFk(const Eigen::VectorXd& q)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = q.head<3>();
  const Eigen::Vector3d rv = q.tail<3>();
  if (rv.norm() > 0)
  {
    T.linear() = Eigen::AngleAxisd(rv.norm(), rv.normalized()).toRotationMatrix();
  }
  return T;
}
std::optional<Eigen::MatrixXd> toyJac(const Eigen::VectorXd&) { return Eigen::MatrixXd::Identity(6, 6); }
}  // namespace

TEST(SimulateRasterScan, FollowsDescentRasterAndLift)
{
  const std::vector<Eigen::Vector3d> path{ { 0, 0, 0 }, { 0.03, 0, 0 }, { 0.03, 0.004, 0 }, { 0, 0.004, 0 } };
  ScanSimParams p;
  p.lift_m = 0.05;
  p.damping = 1e-6;
  p.acquire_margin_s = 0.0;
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
  q0(2) = p.lift_m;  // lifted start along +Z (R0 = identity, lens axis +Z)
  std::vector<std::tuple<double, double, bool>> bounds(6, { -10.0, 10.0, true });
  const auto r = simulateRasterScan(q0, path, Eigen::Matrix3d::Identity(), p, toyFk, toyJac, bounds);
  ASSERT_TRUE(r.ok) << r.why;
  EXPECT_LT(r.max_residual, 1e-5);
  EXPECT_LT(r.max_rot_residual, 1e-5);
  EXPECT_NEAR(r.min_dexterity, 0.4, 1e-9);  // identity J with translation rows / 0.4 -> sigma ratio 0.4
  // Ends lifted above the last point.
  EXPECT_TRUE(r.samples.back().head<3>().isApprox(Eigen::Vector3d(0, 0.004, p.lift_m), 1e-4));
  // Joint speed at 1 mm/s along a unit Jacobian = 0.001 rad/s per m/s ... lift at 3 mm/s dominates.
  EXPECT_NEAR(r.max_joint_speed, p.lift_speed_mps, 1e-4);
}

TEST(SimulateRasterScan, DuplicatePointsGiveIncreasingTimes)
{
  const std::vector<Eigen::Vector3d> path{ { 0, 0, 0 }, { 0, 0, 0 }, { 0.01, 0, 0 }, { 0.01, 0, 0 } };
  ScanSimParams p;
  p.lift_m = 0.01;
  p.damping = 1e-6;
  p.acquire_margin_s = 0.0;
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
  q0(2) = p.lift_m;
  std::vector<std::tuple<double, double, bool>> bounds(6, { -10.0, 10.0, true });
  const auto r = simulateRasterScan(q0, path, Eigen::Matrix3d::Identity(), p, toyFk, toyJac, bounds);
  ASSERT_TRUE(r.ok) << r.why;
  for (size_t i = 1; i < r.times.size(); ++i)
  {
    EXPECT_GT(r.times[i], r.times[i - 1]) << "sample " << i;
  }
}

TEST(SimulateRasterScan, FlagsLimitsAndSpeed)
{
  const std::vector<Eigen::Vector3d> path{ { 0, 0, 0 }, { 0.03, 0, 0 } };
  ScanSimParams p;
  p.lift_m = 0.02;
  p.damping = 1e-6;
  p.acquire_margin_s = 0.0;
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
  q0(2) = p.lift_m;
  std::vector<std::tuple<double, double, bool>> tight(6, { -10.0, 10.0, true });
  tight[0] = { -1.0, 0.01, true };  // x limited to 10 mm, the path needs 30 mm
  auto r = simulateRasterScan(q0, path, Eigen::Matrix3d::Identity(), p, toyFk, toyJac, tight);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.why.find("joint limit"), std::string::npos) << r.why;
  std::vector<std::tuple<double, double, bool>> loose(6, { -10.0, 10.0, true });
  p.max_joint_speed_rps = 0.002;  // below the 3 mm/s lift
  r = simulateRasterScan(q0, path, Eigen::Matrix3d::Identity(), p, toyFk, toyJac, loose);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.why.find("rad/s"), std::string::npos) << r.why;
  // Continuous joints are not limit-checked but their travel is reported.
  std::vector<std::tuple<double, double, bool>> cont(6, { 0.0, 0.0, false });
  p.max_joint_speed_rps = 1.0;
  r = simulateRasterScan(q0, path, Eigen::Matrix3d::Identity(), p, toyFk, toyJac, cont);
  EXPECT_TRUE(r.ok) << r.why;
  EXPECT_NEAR(r.max_continuous_travel, 0.03, 1e-4);  // x swept 30 mm (largest range)
}

TEST(SimulateRasterScan, OrientationTurnsWithTheSunWithoutRoll)
{
  // With a sun feedforward, the simulated orientation turns; its lens axis must not roll (projection).
  const std::vector<Eigen::Vector3d> path{ { 0, 0, 0 }, { 0.01, 0, 0 } };
  ScanSimParams p;
  p.lift_m = 0.0;
  p.damping = 1e-6;
  p.acquire_margin_s = 600.0;  // 10 minutes of drift before the descent
  p.omega_ff_world = sunFeedforwardWorld(39.7555, 125.0);
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
  std::vector<std::tuple<double, double, bool>> bounds(6, { -10.0, 10.0, true });
  const auto r = simulateRasterScan(q0, path, Eigen::Matrix3d::Identity(), p, toyFk, toyJac, bounds);
  ASSERT_TRUE(r.ok) << r.why;
  const Eigen::Matrix3d R = toyFk(r.samples.back()).linear();
  const double turned = std::acos(std::clamp(R.col(2).dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0));
  EXPECT_GT(turned, 0.02);  // ~2.5 deg in 10 min (0.25 deg/min, minus the along-axis part)
  // No roll: the frame's x axis stays perpendicular-transported (twist about z ~ 0).
  const Eigen::AngleAxisd aa(R);
  EXPECT_LT(std::abs((aa.angle() * aa.axis()).z()), 1e-3);
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
