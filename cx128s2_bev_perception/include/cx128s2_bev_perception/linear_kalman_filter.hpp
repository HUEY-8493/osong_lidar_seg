#pragma once

struct TrackingRegressionAccess;

#include <algorithm>
#include <array>
#include <cmath>

namespace cx128s2_bev_perception {

// Linear constant-velocity KF: state [x, y, vx, vy], measurements [x, y].
// Box orientation belongs to geometry memory; yaw is never a filter state.
class LinearKalmanFilter {
public:
  void initialize(double x, double y, double position_std, double velocity_std) {
    x_ = {x, y, 0.0, 0.0};
    p_.fill({0.0, 0.0, 0.0, 0.0});
    const double pp = position_std * position_std;
    const double pv = velocity_std * velocity_std;
    p_[0][0] = p_[1][1] = pp;
    p_[2][2] = p_[3][3] = pv;
    initialized_ = true;
  }

  bool initialized() const { return initialized_; }
  // Changing the tracked reference point (centroid -> observed box center)
  // is a translation, not a new stationary object. Keep velocity/covariance.
  void reanchorPosition(double x, double y) {
    if (!initialized_) return;
    x_[0] = x;
    x_[1] = y;
  }
  double x() const { return x_[0]; }
  double y() const { return x_[1]; }
  double vx() const { return x_[2]; }
  double vy() const { return x_[3]; }
  double positionVarianceX() const { return p_[0][0]; }
  double positionVarianceY() const { return p_[1][1]; }
  std::array<double, 2> innovation() const { return innovation_; }

  void predict(double dt, double accel_std) {
    if (!initialized_ || !(dt > 0.0)) return;
    x_[0] += x_[2] * dt;
    x_[1] += x_[3] * dt;
    Matrix f{{{1,0,dt,0}, {0,1,0,dt}, {0,0,1,0}, {0,0,0,1}}};
    p_ = multiply(multiply(f, p_), transpose(f));
    const double q = accel_std * accel_std;
    const double d2 = dt * dt, d3 = d2 * dt, d4 = d2 * d2;
    p_[0][0] += q * d4 / 4.0; p_[1][1] += q * d4 / 4.0;
    p_[0][2] += q * d3 / 2.0; p_[2][0] += q * d3 / 2.0;
    p_[1][3] += q * d3 / 2.0; p_[3][1] += q * d3 / 2.0;
    p_[2][2] += q * d2;       p_[3][3] += q * d2;
  }

  void update(double measured_x, double measured_y, double measurement_std) {
    update(measured_x, measured_y, measurement_std, measurement_std);
  }

  void update(double measured_x, double measured_y,
              double measurement_std_x, double measurement_std_y) {
    if (!initialized_) return;
    innovation_ = {measured_x - x_[0], measured_y - x_[1]};
    const double rx = measurement_std_x * measurement_std_x;
    const double ry = measurement_std_y * measurement_std_y;
    const double s00 = p_[0][0] + rx, s01 = p_[0][1];
    const double s10 = p_[1][0], s11 = p_[1][1] + ry;
    const double det = s00 * s11 - s01 * s10;
    if (!(det > 1e-12) || !std::isfinite(det)) return;
    const double i00 = s11 / det, i01 = -s01 / det, i10 = -s10 / det, i11 = s00 / det;
    std::array<std::array<double, 2>, 4> k{};
    for (int i = 0; i < 4; ++i) {
      k[i][0] = p_[i][0] * i00 + p_[i][1] * i10;
      k[i][1] = p_[i][0] * i01 + p_[i][1] * i11;
      x_[i] += k[i][0] * innovation_[0] + k[i][1] * innovation_[1];
    }
    // Joseph form: (I-KH)P(I-KH)' + KRK', preserving positive covariance.
    Matrix ikh = identity();
    for (int i = 0; i < 4; ++i) { ikh[i][0] -= k[i][0]; ikh[i][1] -= k[i][1]; }
    p_ = multiply(multiply(ikh, p_), transpose(ikh));
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j)
      p_[i][j] += rx * k[i][0] * k[j][0] + ry * k[i][1] * k[j][1];
  }

  // Soft Y-only observation for a confirmed vehicle's lateral anchor.
  // It is not called in prediction-only frames.
  void updateLateralAnchor(double anchor_y, double anchor_std) {
    if (!initialized_ || !(anchor_std > 0.0)) return;
    const double r = anchor_std * anchor_std;
    const double s = p_[1][1] + r;
    if (!(s > 1e-12) || !std::isfinite(s)) return;
    const double residual = anchor_y - x_[1];
    std::array<double, 4> k{};
    for (int i = 0; i < 4; ++i) {
      k[i] = p_[i][1] / s;
      x_[i] += k[i] * residual;
    }
    Matrix ikh = identity();
    for (int i = 0; i < 4; ++i) ikh[i][1] -= k[i];
    p_ = multiply(multiply(ikh, p_), transpose(ikh));
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) p_[i][j] += r * k[i] * k[j];
  }

  // Decay lateral velocity as part of prediction while retaining uncertainty.
  void dampLateralVelocity(double factor) {
    if (!initialized_) return;
    factor = std::clamp(factor, 0.0, 1.0);
    x_[3] *= factor;
    for (int i = 0; i < 4; ++i) p_[3][i] *= factor;
    for (int i = 0; i < 4; ++i) p_[i][3] *= factor;
    p_[3][3] += (1.0 - factor * factor) * 0.04;
  }

private:
  friend struct ::TrackingRegressionAccess;
  using Matrix = std::array<std::array<double, 4>, 4>;
  static Matrix identity() { Matrix m{}; for (int i = 0; i < 4; ++i) m[i][i] = 1.0; return m; }
  static Matrix transpose(const Matrix & a) { Matrix b{}; for (int i=0;i<4;++i) for(int j=0;j<4;++j) b[i][j]=a[j][i]; return b; }
  static Matrix multiply(const Matrix & a, const Matrix & b) { Matrix c{}; for(int i=0;i<4;++i) for(int j=0;j<4;++j) for(int k=0;k<4;++k) c[i][j]+=a[i][k]*b[k][j]; return c; }
  bool initialized_ = false;
  std::array<double, 4> x_{};
  Matrix p_{};
  std::array<double, 2> innovation_{};
};
}  // namespace cx128s2_bev_perception
