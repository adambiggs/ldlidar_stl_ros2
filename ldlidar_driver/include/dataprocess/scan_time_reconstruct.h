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
 * Reconstruct the frame scan-start stamp (CLOCK_REALTIME nanoseconds).
 *
 * The published stamp for frame k is
 *     stamp_k = max(stamp_{k-1} + period,  fresh_now_k - period)
 * where fresh_now_k is a real clock sample taken at publish (never the collapsed
 * per-packet arrival) and period is one rotation. This single cross-frame value
 * (no tuning parameter):
 *   - holds ~rotation-period spacing across a batched-read burst where two frames
 *     publish milliseconds apart (the cadence-locked prediction wins), and
 *   - follows real time after a genuine multi-rotation drop (the fresh anchor wins),
 * and never stamps a scan in the future (stamp_k <= fresh_now_k always, since
 * period > 0 and the anchor is fresh_now_k - period).
 */
struct ScanStartReconstructor {
  bool initialized = false;
  int64_t last_scan_start_ns = 0;

  int64_t Update(int64_t publish_now_ns, int64_t period_ns) {
    const int64_t anchor_ns = publish_now_ns - period_ns;
    if (!initialized) {
      initialized = true;
      last_scan_start_ns = anchor_ns;
      return last_scan_start_ns;
    }
    const int64_t predicted_ns = last_scan_start_ns + period_ns;
    last_scan_start_ns = (predicted_ns > anchor_ns) ? predicted_ns : anchor_ns;
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
