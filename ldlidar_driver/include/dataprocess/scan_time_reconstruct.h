/**
 * @file scan_time_reconstruct.h
 * @brief Rotation-model measurement-time reconstruction for the LD19 LaserScan.
 *
 * Tombo fork addition (BUG-0050 / Story 059.3). Pure, header-only, no ROS/hardware
 * dependency so it is unit-testable in isolation. Two concerns:
 *
 *  1. ScanStartReconstructor — reconstruct the frame scan-start stamp in host
 *     CLOCK_REALTIME from the LiDAR's reported spin rate instead of serial-arrival
 *     time, so a batched read() that collapses per-packet arrival stamps cannot
 *     reach the published scan.
 *
 *  2. ComputeScanAngleTiming — publish forward-in-time per-point encoding
 *     (non-negative time_increment) with the laser_scan_dir counterclockwise
 *     reflection carried by the signed angle fields, not by reversing the range
 *     array. This is Story 059.3 encoding (b); it keeps the delivered geometry
 *     byte-identical while making per-point time honest for Phase-B deskew.
 */
#ifndef LDLIDAR_SCAN_TIME_RECONSTRUCT_H
#define LDLIDAR_SCAN_TIME_RECONSTRUCT_H

#include <cstdint>
#include <cmath>

namespace ldlidar {

/**
 * Reconstruct the frame scan-start stamp (CLOCK_REALTIME nanoseconds) with a
 * rotation-cadence-locked clock that is bounded-slewed toward real time.
 *
 * For frame k: predict stamp_k = stamp_{k-1} + period (cadence lock), then correct
 * toward the fresh real anchor (fresh_now_k - period) by at most max_slew per
 * frame:
 *     stamp_k = (stamp_{k-1} + period) + clamp(anchor - (stamp_{k-1}+period), +-max_slew)
 * where fresh_now_k is a real clock sample taken at publish (never the collapsed
 * per-packet arrival) and period is one rotation.
 *
 * This is drift-free AND collapse-immune:
 *   - Normal cadence: anchor ~= prediction, correction ~= 0 -> stamp advances by
 *     one rotation, ~period behind fresh now (the AC1 ~112 ms structure).
 *   - Batched-read burst / arrival spike (two frames ms apart, or a 675 ms late
 *     read): the anchor is far from the prediction, correction is clamped to
 *     +-max_slew -> stamp stays on cadence, spacing ~= period (no SLAM-001
 *     collapse). AC2.
 *   - Slow real drift (reported spin period vs true publish cadence, ~0.1 ms/
 *     frame): the small per-frame error is well within max_slew, so the clock
 *     tracks real time and the stamp cannot ratchet into the future. AC1.
 * A plain max(prediction, anchor) was rejected: it ratchets forward whenever the
 * reported-spin period exceeds the true cadence and never re-anchors, drifting the
 * stamp into the future (observed live: ~0.08 ms/frame). max_slew must exceed the
 * real per-frame cadence error but stay far below a burst/spike so those are
 * rejected; 2 ms/frame satisfies both for a ~100 ms rotation.
 */
struct ScanStartReconstructor {
  bool initialized = false;
  int64_t last_scan_start_ns = 0;
  int64_t max_slew_ns = 2'000'000;  // 2 ms/frame

  int64_t Update(int64_t publish_now_ns, int64_t period_ns) {
    const int64_t anchor_ns = publish_now_ns - period_ns;
    if (!initialized) {
      initialized = true;
      last_scan_start_ns = anchor_ns;
      return last_scan_start_ns;
    }
    const int64_t predicted_ns = last_scan_start_ns + period_ns;
    int64_t correction_ns = anchor_ns - predicted_ns;
    if (correction_ns > max_slew_ns) {
      correction_ns = max_slew_ns;
    } else if (correction_ns < -max_slew_ns) {
      correction_ns = -max_slew_ns;
    }
    last_scan_start_ns = predicted_ns + correction_ns;
    return last_scan_start_ns;
  }
};

/**
 * Published per-point angle/time encoding for one scan (Story 059.3 encoding (b)).
 *
 * angle_increment sign carries the scan direction; time_increment is always >= 0
 * (points are stored in forward measurement order). rtabmap_conversions 0.22.1
 * rejects a scan whose angle_increment sign disagrees with (angle_max - angle_min),
 * so for the counterclockwise case the triple is (angle_min=2pi, angle_max=0,
 * angle_increment=-delta) — a mutually consistent negative-increment triple.
 */
struct ScanAngleTiming {
  double angle_min;
  double angle_max;
  double angle_increment;  // signed: <0 for counterclockwise
  double time_increment;   // >= 0 (forward measurement order)
  double scan_time;        // one rotation period (s)
};

inline ScanAngleTiming ComputeScanAngleTiming(int beam_size,
                                              double rotation_period_s,
                                              bool laser_scan_dir_ccw) {
  const double two_pi = 2.0 * M_PI;
  const double delta =
      (beam_size > 1) ? two_pi / static_cast<double>(beam_size - 1) : 0.0;

  ScanAngleTiming t;
  t.scan_time = rotation_period_s;
  t.time_increment =
      (beam_size > 1) ? rotation_period_s / static_cast<double>(beam_size - 1) : 0.0;

  if (laser_scan_dir_ccw) {
    // Counterclockwise reflection expressed in the angle fields (consistent
    // negative-increment triple for rtabmap's guard), not by reversing the array.
    t.angle_min = two_pi;
    t.angle_max = 0.0;
    t.angle_increment = -delta;
  } else {
    t.angle_min = 0.0;
    t.angle_max = two_pi;
    t.angle_increment = delta;
  }
  return t;
}

}  // namespace ldlidar

#endif  // LDLIDAR_SCAN_TIME_RECONSTRUCT_H
