#include <gtest/gtest.h>

#include <sun_pose_finder/fit_plane_raster_path.hpp>
#include <sun_pose_finder/solar_position.hpp>

namespace sun_pose_finder
{
TEST(SolarPosition, SolsticeNoonInGolden)
{
  // 2024-06-20, Golden CO: noon elevation = 90 - latitude + obliquity.
  double best_elevation = -90.0;
  double best_azimuth = 0.0;
  for (int s = 0; s < 86400; s += 30)
  {
    const auto sun = computeSolarPosition(1718841600.0 + s, 39.7555, -105.2211);
    if (sun.elevation_deg > best_elevation)
    {
      best_elevation = sun.elevation_deg;
      best_azimuth = sun.azimuth_deg;
    }
  }
  EXPECT_NEAR(best_elevation, 90.0 - 39.7555 + 23.44, 0.1);
  EXPECT_NEAR(best_azimuth, 180.0, 1.0);
}

TEST(SolarPosition, MorningSunIsInTheEast)
{
  const auto sun = computeSolarPosition(1710950400.0, 39.7555, -105.2211);  // 2024-03-20 16:00Z = 10:00 MDT
  EXPECT_GT(sun.azimuth_deg, 90.0);
  EXPECT_LT(sun.azimuth_deg, 180.0);
  EXPECT_GT(sun.elevation_deg, 20.0);
}

TEST(FitPlaneRaster, TiltedRectangle)
{
  // 0.10 x 0.04 m grid on a plane tilted 30 deg about X.
  const Eigen::AngleAxisd tilt(M_PI / 6.0, Eigen::Vector3d::UnitX());
  std::vector<Eigen::Vector3d> points;
  for (double x = -0.05; x <= 0.05 + 1e-9; x += 0.005)
  {
    for (double y = -0.02; y <= 0.02 + 1e-9; y += 0.005)
    {
      points.push_back(tilt * Eigen::Vector3d(x, y, 0.0) + Eigen::Vector3d(0.1, -0.6, 0.2));
    }
  }

  const auto raster = fitPlaneRaster(points, 0.01, 0.05, 0.0, 0.0);
  ASSERT_TRUE(raster.has_value()) << raster.error();
  EXPECT_NEAR(raster->rms_m, 0.0, 1e-9);

  const Eigen::Vector3d normal = raster->plane.linear().col(2);
  EXPECT_NEAR(normal.dot(tilt * Eigen::Vector3d::UnitZ()), 1.0, 1e-6);  // points toward +Z
  EXPECT_NEAR(std::abs(raster->plane.linear().col(0).dot(tilt * Eigen::Vector3d::UnitX())), 1.0, 1e-6);

  // 5 lines (0.04 / 0.01 + 1), each 0.10 m split into 2 segments -> 3 points per line.
  ASSERT_EQ(raster->poses.size(), 15u);
  for (const auto& pose : raster->poses)
  {
    EXPECT_NEAR((pose.translation() - raster->plane.translation()).dot(normal), 0.0, 1e-9);
  }
  // Zig-zag: the second line starts where the first ended.
  EXPECT_LT((raster->poses[2].translation() - raster->poses[3].translation()).norm(), 0.0101);
}

TEST(FitPlaneRaster, SubdividesLines)
{
  // 0.08 x 0.02 m flat grid: 3 lines, each split into quarters -> 5 evenly spaced points per line.
  std::vector<Eigen::Vector3d> points;
  for (double x = -0.04; x <= 0.04 + 1e-9; x += 0.002)
  {
    for (double y = -0.01; y <= 0.01 + 1e-9; y += 0.002)
    {
      points.emplace_back(x, y, 0.0);
    }
  }

  const auto raster = fitPlaneRaster(points, 0.01, 0.0, 0.0, 0.0, 4);
  ASSERT_TRUE(raster.has_value()) << raster.error();
  ASSERT_EQ(raster->poses.size(), 15u);
  for (size_t i = 0; i + 1 < 5; ++i)
  {
    EXPECT_NEAR((raster->poses[i + 1].translation() - raster->poses[i].translation()).norm(), 0.02, 1e-6);
  }
  // A coarser point_spacing does not undo the subdivisions; a finer one wins.
  EXPECT_EQ(fitPlaneRaster(points, 0.01, 0.05, 0.0, 0.0, 4)->poses.size(), 15u);
  EXPECT_EQ(fitPlaneRaster(points, 0.01, 0.01, 0.0, 0.0, 4)->poses.size(), 27u);
  EXPECT_FALSE(fitPlaneRaster(points, 0.01, 0.0, 0.0, 0.0, 0).has_value());
}

TEST(FitPlaneRaster, DropsOutliers)
{
  // 0.10 x 0.10 m flat grid at z = 0.1 with noisy points floating above and below it, and one far off to the side.
  std::vector<Eigen::Vector3d> points;
  for (double x = -0.05; x <= 0.05 + 1e-9; x += 0.005)
  {
    for (double y = -0.05; y <= 0.05 + 1e-9; y += 0.005)
    {
      points.emplace_back(x, y, 0.1);
    }
  }
  const size_t num_surface = points.size();
  for (int i = 0; i < 40; ++i)
  {
    points.emplace_back(-0.04 + 0.002 * i, 0.03, i % 2 == 0 ? 0.25 : -0.05);
  }
  points.emplace_back(0.5, 0.5, 0.4);

  const auto raw = fitPlaneRaster(points, 0.01, 0.0, 0.0, 0.0);
  ASSERT_TRUE(raw.has_value()) << raw.error();
  EXPECT_GT(raw->rms_m, 0.01);  // outliers wreck the plain least-squares fit

  const auto raster = fitPlaneRaster(points, 0.01, 0.0, 0.0, 0.005);
  ASSERT_TRUE(raster.has_value()) << raster.error();
  EXPECT_EQ(raster->num_inliers, num_surface);
  EXPECT_EQ(raster->num_outliers, 41u);
  EXPECT_NEAR(raster->rms_m, 0.0, 1e-9);
  EXPECT_NEAR(raster->plane.linear().col(2).z(), 1.0, 1e-9);
  EXPECT_NEAR(raster->plane.translation().z(), 0.1, 1e-9);
  for (const auto& pose : raster->poses)
  {
    EXPECT_LE(pose.translation().head<2>().cwiseAbs().maxCoeff(), 0.05 + 1e-9);  // extent from the inliers only
  }
}

TEST(FitPlaneRaster, StaysOnRotatedSquare)
{
  // 0.20 x 0.20 m square rotated 30 deg about Z: a square's long axis is arbitrary, so the scan lines run at some angle
  // to its edges. Every waypoint must still lie on the square (within half a line spacing at a slanted edge).
  const Eigen::AngleAxisd yaw(M_PI / 6.0, Eigen::Vector3d::UnitZ());
  std::vector<Eigen::Vector3d> points;
  for (double x = -0.10; x <= 0.10 + 1e-9; x += 0.002)
  {
    for (double y = -0.10; y <= 0.10 + 1e-9; y += 0.002)
    {
      points.push_back(yaw * Eigen::Vector3d(x, y, 0.1));
    }
  }

  const auto raster = fitPlaneRaster(points, 0.01, 0.0, 0.005, 0.0);
  ASSERT_TRUE(raster.has_value()) << raster.error();
  ASSERT_FALSE(raster->poses.empty());
  for (const auto& pose : raster->poses)
  {
    const Eigen::Vector3d in_square = yaw.inverse() * pose.translation();
    EXPECT_LE(in_square.head<2>().cwiseAbs().maxCoeff(), 0.10 + 0.005 + 1e-9);
  }
}

TEST(FitPlaneRaster, RejectsBadInput)
{
  EXPECT_FALSE(fitPlaneRaster({ Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX() }, 0.01, 0.0, 0.0, 0.0).has_value());
  const std::vector<Eigen::Vector3d> square{ { 0, 0, 0 }, { 0.02, 0, 0 }, { 0, 0.02, 0 }, { 0.02, 0.02, 0 } };
  EXPECT_FALSE(fitPlaneRaster(square, 0.0, 0.0, 0.0, 0.0).has_value());
  EXPECT_FALSE(fitPlaneRaster(square, 0.01, 0.0, 0.02, 0.0).has_value());
}
}  // namespace sun_pose_finder
