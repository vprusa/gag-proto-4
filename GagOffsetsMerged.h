#pragma once
/**
 * GagOffsets.h
 *
 * Single-file offset store + calibration helpers.
 *
 * "HW offsets" here are stored in the same 6-component layout used by the old
 * firmware: ax, ay, az, gx, gy, gz.
 *
 * For MPU6050-class sensors these values can be written into device registers.
 * For the wrist MPU9250 path in the merged sketch, the same values are applied
 * as pre-fusion raw-data biases because that code path uses direct register
 * reads instead of the DMP helper stack.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "GagRecogMerged.h"

namespace gag {
namespace offsets {

struct HwOffset6 {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;

  HwOffset6()
    : ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {}

  HwOffset6(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_)
    : ax(ax_), ay(ay_), az(az_), gx(gx_), gy(gy_), gz(gz_) {}
};

struct RawImuSample {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;

  RawImuSample()
    : ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {}

  RawImuSample(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_)
    : ax(ax_), ay(ay_), az(az_), gx(gx_), gy(gy_), gz(gz_) {}
};

struct StableWindowConfig {
  uint16_t required_samples = 200;
  int16_t accel_abs_delta_limit = 180;
  int16_t gyro_abs_delta_limit = 25;
};

struct SampleStats {
  bool valid = false;
  uint16_t count = 0;
  RawImuSample mean{};
  RawImuSample min{};
  RawImuSample max{};
};

class StablePoseAccumulator {
public:
  void reset() {
    _count = 0;
    _sumAx = _sumAy = _sumAz = 0;
    _sumGx = _sumGy = _sumGz = 0;
    _min.ax = _min.ay = _min.az = _min.gx = _min.gy = _min.gz = 32767;
    _max.ax = _max.ay = _max.az = _max.gx = _max.gy = _max.gz = -32768;
  }

  StablePoseAccumulator() { reset(); }

  void push(const RawImuSample& s) {
    ++_count;
    _sumAx += s.ax; _sumAy += s.ay; _sumAz += s.az;
    _sumGx += s.gx; _sumGy += s.gy; _sumGz += s.gz;
    updateMinMax(_min.ax, _max.ax, s.ax);
    updateMinMax(_min.ay, _max.ay, s.ay);
    updateMinMax(_min.az, _max.az, s.az);
    updateMinMax(_min.gx, _max.gx, s.gx);
    updateMinMax(_min.gy, _max.gy, s.gy);
    updateMinMax(_min.gz, _max.gz, s.gz);
  }

  SampleStats stats() const {
    SampleStats out;
    if (_count == 0) return out;
    out.valid = true;
    out.count = _count;
    out.mean.ax = (int16_t)(_sumAx / (long)_count);
    out.mean.ay = (int16_t)(_sumAy / (long)_count);
    out.mean.az = (int16_t)(_sumAz / (long)_count);
    out.mean.gx = (int16_t)(_sumGx / (long)_count);
    out.mean.gy = (int16_t)(_sumGy / (long)_count);
    out.mean.gz = (int16_t)(_sumGz / (long)_count);
    out.min = _min;
    out.max = _max;
    return out;
  }

  bool isStable(const StableWindowConfig& cfg) const {
    if (_count < cfg.required_samples) return false;
    return (absInt(_max.ax - _min.ax) <= cfg.accel_abs_delta_limit) &&
           (absInt(_max.ay - _min.ay) <= cfg.accel_abs_delta_limit) &&
           (absInt(_max.az - _min.az) <= cfg.accel_abs_delta_limit) &&
           (absInt(_max.gx - _min.gx) <= cfg.gyro_abs_delta_limit) &&
           (absInt(_max.gy - _min.gy) <= cfg.gyro_abs_delta_limit) &&
           (absInt(_max.gz - _min.gz) <= cfg.gyro_abs_delta_limit);
  }

private:
  static inline int absInt(int v) { return (v < 0) ? -v : v; }
  static inline void updateMinMax(int16_t& mn, int16_t& mx, int16_t v) {
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }

  uint16_t _count = 0;
  long _sumAx = 0, _sumAy = 0, _sumAz = 0;
  long _sumGx = 0, _sumGy = 0, _sumGz = 0;
  RawImuSample _min{};
  RawImuSample _max{};
};

static inline HwOffset6 computeHardwareOffsetsFromStablePose(const SampleStats& s,
                                                             int16_t targetAz = 16384) {
  HwOffset6 out;
  if (!s.valid || s.count == 0) return out;
  out.ax = (int16_t)(-s.mean.ax / 8);
  out.ay = (int16_t)(-s.mean.ay / 8);
  out.az = (int16_t)((targetAz - s.mean.az) / 8);
  out.gx = (int16_t)(-s.mean.gx / 4);
  out.gy = (int16_t)(-s.mean.gy / 4);
  out.gz = (int16_t)(-s.mean.gz / 4);
  return out;
}

class OffsetStore {
public:
  static constexpr uint8_t kMaxSensors = 8;

  OffsetStore() {
    resetAll();
  }

  void resetAll() {
    for (uint8_t i = 0; i < kMaxSensors; ++i) {
      _hw[i] = HwOffset6();
      _sw[i] = Quaternion();
    }
  }

  void setHardware(uint8_t sensorIndex, const HwOffset6& hw) {
    if (!validIndex(sensorIndex)) return;
    _hw[sensorIndex] = hw;
  }

  HwOffset6 hardware(uint8_t sensorIndex) const {
    if (!validIndex(sensorIndex)) return HwOffset6();
    return _hw[sensorIndex];
  }

  void setSoftwareQuaternion(uint8_t sensorIndex, const Quaternion& qIn) {
    if (!validIndex(sensorIndex)) return;
    Quaternion q = qIn;
    q.normalizeInPlace();
    _sw[sensorIndex] = q;
  }

  void setSoftwareEulerDeg(uint8_t sensorIndex, float xDeg, float yDeg, float zDeg) {
    setSoftwareQuaternion(sensorIndex, Quaternion::fromEulerZyxDeg(zDeg, yDeg, xDeg));
  }

  Quaternion softwareQuaternion(uint8_t sensorIndex) const {
    if (!validIndex(sensorIndex)) return Quaternion();
    return _sw[sensorIndex];
  }

  Quaternion applySoftwareOffset(uint8_t sensorIndex, const Quaternion& rawIn) const {
    Quaternion raw = rawIn;
    raw.normalizeInPlace();
    if (!validIndex(sensorIndex)) return raw;
    Quaternion out = Quaternion::mul(_sw[sensorIndex].inverseUnit(), raw);
    out.normalizeInPlace();
    return out;
  }

  static Quaternion computeNeutralizingSoftwareOffset(const Quaternion& currentRaw,
                                                      const Quaternion& desiredNeutral = Quaternion()) {
    Quaternion actual = currentRaw;
    actual.normalizeInPlace();
    Quaternion desired = desiredNeutral;
    desired.normalizeInPlace();
    Quaternion diff = Quaternion::mul(actual, desired.inverseUnit());
    diff.normalizeInPlace();
    return diff;
  }

private:
  static inline bool validIndex(uint8_t idx) { return idx < kMaxSensors; }

  HwOffset6 _hw[kMaxSensors];
  Quaternion _sw[kMaxSensors];
};

} // namespace offsets
} // namespace gag
