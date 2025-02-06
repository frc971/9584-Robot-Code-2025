#include "RobotContainer.h"

#include "gtest/gtest.h"
namespace {
const float ERROR_ALLOWANCE = 0.000001;
TEST(ExponentialConvert, Positive) {
  EXPECT_NEAR(0.008, RobotContainer::ExponentialConvert(0.2, 3),
              ERROR_ALLOWANCE);
}
TEST(ExponentialConvert, Negative) {
  EXPECT_NEAR(-0.008, RobotContainer::ExponentialConvert(-0.2, 3),
              ERROR_ALLOWANCE);
}
TEST(ExponentialConvert, Zero) {
  EXPECT_NEAR(0, RobotContainer::ExponentialConvert(0, 3), ERROR_ALLOWANCE);
}
}  // namespace