#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cx128s2_bev_perception/linear_kalman_filter.hpp>
using cx128s2_bev_perception::LinearKalmanFilter;
int main() {
  LinearKalmanFilter k;
  k.initialize(1.0, 2.0, 0.1, 1.0);
  assert(k.initialized() && std::abs(k.x()-1.0) < 1e-12);
  k.predict(0.1, 1.5); k.update(1.1, 2.0, 0.05);
  assert(k.x() > 1.05 && k.positionVarianceX() > 0.0);
  const double before = k.x(); k.predict(0.0, 1.5); assert(k.x() == before);
  for (int i=0; i<30; ++i) { k.predict(i % 2 ? 0.09 : 0.12, 1.5); k.update(1.2 + 0.1*i, 2.0, 0.05); }
  assert(std::isfinite(k.x()) && std::isfinite(k.vx()) && k.vx() > 0.3);
  // A geometry reference change preserves motion and the next prediction.
  const auto vx = k.vx(), vy = k.vy();
  const auto px = k.positionVarianceX(), py = k.positionVarianceY();
  k.reanchorPosition(-7.0, 4.0);
  assert(k.x() == -7.0 && k.y() == 4.0);
  assert(k.vx() == vx && k.vy() == vy);
  assert(k.positionVarianceX() == px && k.positionVarianceY() == py);
  k.predict(0.1, 1.5);
  assert(std::abs(k.x() - (-7.0 + vx * 0.1)) < 1e-12);
  k.predict(0.3, 1.5);  // dropout: prediction only
  assert(std::isfinite(k.x()));
  LinearKalmanFilter recreated; recreated.initialize(9.0, -1.0, .1, 1.0);
  assert(std::abs(recreated.x()-9.0) < 1e-12);  // no state contamination

  // A distant, biased partial face must not pull the lateral state with it.
  LinearKalmanFilter far;
  far.initialize(0.0, 2.0, 0.1, 1.0);
  const double biased_y[] = {2.02, 2.05, 2.15, 2.28, 2.35};
  for (int i = 0; i < 5; ++i) {
    far.predict(0.1, 1.5);
    far.dampLateralVelocity(std::exp(-0.1));
    far.update(0.1 * (i + 1), biased_y[i], 0.4, 3.2);
    far.updateLateralAnchor(2.0, 0.4);
  }
  assert(far.y() < 2.1);
  assert(far.x() > 0.2);

  // Repeated mid-range FULL bias is also bounded over a long run.
  LinearKalmanFilter mid;
  mid.initialize(0.0, 2.0, 0.1, 1.0);
  for (int i = 0; i < 100; ++i) {
    mid.predict(0.1, 1.5);
    mid.dampLateralVelocity(std::exp(-0.1));
    mid.update(0.1 * (i + 1), 2.35, 0.1, 0.6);
    mid.updateLateralAnchor(2.0, 0.4);
  }
  assert(mid.y() < 2.2);

  // A near vehicle can still make a sustained, genuine lateral move.
  LinearKalmanFilter moving;
  moving.initialize(0.0, 2.0, 0.1, 1.0);
  for (int i = 0; i < 12; ++i) {
    moving.predict(0.1, 1.5);
    moving.dampLateralVelocity(std::exp(-0.1));
    moving.update(0.1 * (i + 1), 2.0 + 0.05 * (i + 1), 0.1, 0.2);
    moving.updateLateralAnchor(2.0, 0.8);
  }
  assert(moving.y() > 2.3);
  assert(moving.y() < 2.7);

  // Large Y noise must not degrade the independent X measurement update.
  LinearKalmanFilter separate;
  separate.initialize(0.0, 0.0, 0.1, 1.0);
  separate.predict(0.1, 1.5);
  separate.update(1.0, 1.0, 0.1, 2.0);
  assert(separate.x() > 0.5);
  assert(separate.y() < 0.1);

  // LOST coast predicts motion without a measurement and grows uncertainty.
  LinearKalmanFilter lost;
  lost.initialize(0.0, 3.0, 0.1, 1.0);
  const double initial_variance = lost.positionVarianceX();
  for (int i = 0; i < 30; ++i) {
    lost.predict(0.1, 1.5);
    lost.dampLateralVelocity(std::exp(-0.1));
  }
  assert(lost.positionVarianceX() > initial_variance);
  assert(lost.positionVarianceY() > initial_variance);
  assert(std::isfinite(lost.x()) && std::isfinite(lost.y()));
}
