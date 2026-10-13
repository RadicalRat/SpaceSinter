// Copyright 2024 Space Sinter Team, Colorado School of Mines
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "kinova_driver/joint_math.hpp"

using kinova_driver::kDegToRad;

TEST(WrapToPi, StaysInHalfOpenRange)
{
  EXPECT_NEAR(kinova_driver::wrapToPi(0.0), 0.0, 1e-12);
  EXPECT_NEAR(kinova_driver::wrapToPi(M_PI), M_PI, 1e-12);
  EXPECT_NEAR(kinova_driver::wrapToPi(-M_PI), M_PI, 1e-12);  // −π maps to +π
  EXPECT_NEAR(kinova_driver::wrapToPi(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(kinova_driver::wrapToPi(-3.0 * M_PI / 2.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(kinova_driver::wrapToPi(10.0 * M_PI + 0.1), 0.1, 1e-9);
}

TEST(KinovaPosition, ContinuousJointsWrap)
{
  EXPECT_NEAR(kinova_driver::kinovaPositionToRad(270.0f, true), -90.0 * kDegToRad, 1e-6);
  EXPECT_NEAR(kinova_driver::kinovaPositionToRad(90.0f, true), 90.0 * kDegToRad, 1e-6);
}

TEST(KinovaPosition, RevoluteJointsKeepFirmwareRange)
{
  EXPECT_NEAR(kinova_driver::kinovaPositionToRad(270.0f, false), 270.0 * kDegToRad, 1e-6);
}

TEST(KinovaVelocity, DecodesNegativeAndDoubles)
{
  EXPECT_NEAR(kinova_driver::kinovaVelocityToRad(10.0f), 20.0 * kDegToRad, 1e-6);
  EXPECT_NEAR(kinova_driver::kinovaVelocityToRad(350.0f), -20.0 * kDegToRad, 1e-6);
  EXPECT_NEAR(kinova_driver::kinovaVelocityToRad(0.0f), 0.0, 1e-12);
}

TEST(PositionError, ContinuousTakesShortestPath)
{
  const double cmd = 179.0 * kDegToRad;
  const double pos = -179.0 * kDegToRad;
  EXPECT_NEAR(kinova_driver::positionError(cmd, pos, true), -2.0 * kDegToRad, 1e-9);
  EXPECT_NEAR(kinova_driver::positionError(cmd, pos, false), 358.0 * kDegToRad, 1e-9);
}

TEST(PController, ClampsToMaxVelocity)
{
  EXPECT_FLOAT_EQ(kinova_driver::positionErrorToVelocityDeg(1.0, 5.0, 100.0), 100.0f);
  EXPECT_FLOAT_EQ(kinova_driver::positionErrorToVelocityDeg(-1.0, 5.0, 100.0), -100.0f);
  EXPECT_NEAR(kinova_driver::positionErrorToVelocityDeg(1.0 * kDegToRad, 5.0, 100.0), 5.0f, 1e-4);
}

TEST(VelocityPassthrough, ConvertsAndClamps)
{
  EXPECT_NEAR(kinova_driver::velocityCommandToDeg(10.0 * kDegToRad, 100.0), 10.0f, 1e-4);
  EXPECT_FLOAT_EQ(kinova_driver::velocityCommandToDeg(10.0, 100.0), 100.0f);
}
