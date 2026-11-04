#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>
namespace crane_v2 {
// Initially keep the largest accepted observation in the sensor-time window.
// Once locked, never shrink for the same load, including a temporary IDLE.
class LengthLock {
  uint64_t id_ = 0;
  std::deque<std::pair<double, double>> samples_;
  double length_ = 0, pending_peak_ = 0;
public:
  void reset() { *this = LengthLock{}; }
  void confirmObservedPeak(uint64_t id, double peak, double evidence_sec, double required_sec) {
    if(!id||!std::isfinite(peak)||peak<=0||evidence_sec+1e-8<required_sec)return;
    if(id_!=id) {reset();id_=id;}
    if(length_<=0)length_=peak; // Reuse actual same-Cargo evidence, never saved ROS state.
  }
  double update(uint64_t id, bool direct, bool confirmed, double measured,
                double stamp, bool continuous, bool completed, double seconds,
                bool working = true, bool near_hook_connected = false)
  {
    if (completed || !id) { reset(); return 0; }
    if (id != id_) { reset(); id_ = id; }
    // Initial IDLE is live and supplies no lock evidence. After a work-cycle
    // lock, temporary IDLE must not replace it with a smaller observed fragment.
    if (!working) {
      samples_.clear();
      if (length_ > 0 && direct && confirmed && near_hook_connected &&
          std::isfinite(measured) && measured > length_)
        length_ = measured;
      return length_;
    }
    if (!continuous || !direct || !std::isfinite(measured) || measured <= 0)
      samples_.clear();
    if (!direct || !std::isfinite(measured) || measured <= 0) return length_;
    if (length_ > 0) {
      if (confirmed && near_hook_connected)
        length_ = std::max(length_, measured);
      samples_.clear();
      return length_;
    }
    // A brief missed frame restarts the confirmation clock, not the largest
    // accepted same-Cargo observation. Actual relation loss resets the object.
    pending_peak_ = std::max(pending_peak_, measured);
    samples_.emplace_back(stamp, measured);
    const double cutoff = stamp - seconds;
    // Keep the boundary sample so elapsed time is never rounded up.
    while (samples_.size() > 1 && samples_[1].first <= cutoff + 1e-8)
      samples_.pop_front();
    if (confirmed && stamp - samples_.front().first + 1e-8 >= seconds) {
      double peak = measured;
      for (const auto& sample : samples_) peak = std::max(peak, sample.second);
      length_ = std::max(pending_peak_, peak);
    }
    return length_;
  }
};
}
