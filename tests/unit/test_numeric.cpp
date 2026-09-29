#include "autoaim/math/numeric.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim::math;

  return test::run([] {
    Eigen::Matrix2d matrix = Eigen::Matrix2d::Identity() * 2;
    auto solved = solve_positive_definite(matrix, Eigen::Matrix2d::Identity());
    CHECK(solved);
    CHECK((matrix * solved.value()).isApprox(Eigen::Matrix2d::Identity()));
    CHECK(covariance_valid(matrix));
    matrix(1, 1) = 0;
    CHECK(covariance_valid(matrix));
    CHECK(!solve_positive_definite(matrix, Eigen::Matrix2d::Identity()));
    matrix(1, 1) = -1;
    CHECK(!covariance_valid(matrix));
    matrix(1, 1) = NAN;
    CHECK(!covariance_valid(matrix));
    CHECK(!solve_positive_definite(matrix, Eigen::Matrix2d::Identity()));
    CHECK(!solve_positive_definite(Eigen::Matrix2d::Identity(), Eigen::Matrix3d::Identity()));
  });
}
