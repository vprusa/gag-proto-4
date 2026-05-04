#pragma once
/**
 * GagRecog.h
 *
 * Header-only gesture recognizer for the ESP32 glove migration.
 *
 * Design goals:
 *  - retain the old `gag::Recognizer` / `gag::GestureDef` shape where practical
 *  - keep the implementation in one file
 *  - support multiple actions per gesture (viz mode switch, blink, mouse, vibration)
 *  - compile both on Arduino and on a Linux host for unit tests
 */

#ifdef ARDUINO
  #include <Arduino.h>
#else
  #include <cmath>
  #include <cstddef>
  #include <cstdint>
  #include <cstdio>
  #include <cstring>
  #include <cstdlib>
  #include <string>

  class Print {
  public:
    virtual ~Print() = default;
    virtual size_t write(uint8_t) = 0;
    size_t write(const uint8_t* buffer, size_t size) {
      if (!buffer) return 0;
      for (size_t i = 0; i < size; ++i) write(buffer[i]);
      return size;
    }
    size_t print(const char* s) {
      if (!s) return 0;
      return write(reinterpret_cast<const uint8_t*>(s), std::strlen(s));
    }
    size_t println(const char* s = "") {
      size_t n = print(s);
      n += write((uint8_t)'\n');
      return n;
    }
    size_t println(int v) { size_t n = print(v); n += write((uint8_t)'\n'); return n; }
    size_t println(unsigned v) { size_t n = print(v); n += write((uint8_t)'\n'); return n; }
    size_t println(unsigned long v) { size_t n = print(v); n += write((uint8_t)'\n'); return n; }
    size_t println(float v, int digits = 3) { size_t n = print(v, digits); n += write((uint8_t)'\n'); return n; }
    size_t print(int v) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%d", v);
      return print(buf);
    }
    size_t print(unsigned v) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%u", v);
      return print(buf);
    }
    size_t print(unsigned long v) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%lu", v);
      return print(buf);
    }
    size_t print(float v, int digits = 3) {
      char fmt[16];
      char buf[64];
      std::snprintf(fmt, sizeof(fmt), "%%.%df", digits);
      std::snprintf(buf, sizeof(buf), fmt, (double)v);
      return print(buf);
    }
  };

  class Stream : public Print {
  public:
    int available() { return 0; }
    int read() { return -1; }
    int peek() { return -1; }
  };

  class NullConsolePrint : public Print {
  public:
    size_t write(uint8_t) override { return 1; }
  };

  class NullConsoleStream : public Stream {
  public:
    size_t write(uint8_t) override { return 1; }
  };

  static NullConsolePrint Serial;
  static NullConsoleStream GAG_DUMMY_STREAM;
#endif

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#ifndef GAG_RECOG_MAX_GESTURES
#define GAG_RECOG_MAX_GESTURES 24
#endif

#ifndef GAG_RECOG_MAX_NAME_LEN
#define GAG_RECOG_MAX_NAME_LEN 32
#endif

#ifndef GAG_RECOG_MAX_CMD_LEN
#define GAG_RECOG_MAX_CMD_LEN 48
#endif

#ifndef GAG_RECOG_MAX_LABEL_LEN
#define GAG_RECOG_MAX_LABEL_LEN 12
#endif

#ifndef GAG_RECOG_MAX_QUATS_PER_SENSOR
#define GAG_RECOG_MAX_QUATS_PER_SENSOR 8
#endif

#ifndef GAG_RECOG_MAX_MATCHERS_PER_SENSOR
#define GAG_RECOG_MAX_MATCHERS_PER_SENSOR 6
#endif

#ifndef GAG_RECOG_USE_JAVA_DISTANCE
#define GAG_RECOG_USE_JAVA_DISTANCE 0
#endif

namespace gag {

inline static bool streq(const char* a, const char* b) {
  if (a == nullptr || b == nullptr) return false;
  return ::strcmp(a, b) == 0;
}

enum class Sensor : uint8_t {
  WRIST  = 0,
  THUMB  = 1,
  INDEX  = 2,
  MIDDLE = 3,
  RING   = 4,
  LITTLE = 5,
  COUNT  = 6
};

static inline const char* sensorToString(Sensor s) {
  switch (s) {
    case Sensor::WRIST:  return "WRIST";
    case Sensor::THUMB:  return "THUMB";
    case Sensor::INDEX:  return "INDEX";
    case Sensor::MIDDLE: return "MIDDLE";
    case Sensor::RING:   return "RING";
    case Sensor::LITTLE: return "LITTLE";
    default:             return "UNKNOWN";
  }
}

static inline uint8_t sensorBit(Sensor s) { return static_cast<uint8_t>(1u << static_cast<uint8_t>(s)); }

struct Quaternion {
  float w;
  float x;
  float y;
  float z;

  Quaternion() : w(1), x(0), y(0), z(0) {}
  Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

  inline void normalizeInPlace() {
    const float n2 = w*w + x*x + y*y + z*z;
    if (n2 <= 0.0f) { w = 1.0f; x = y = z = 0.0f; return; }
    const float inv = 1.0f / sqrtf(n2);
    w *= inv; x *= inv; y *= inv; z *= inv;
  }

  inline Quaternion normalized() const {
    Quaternion q(*this);
    q.normalizeInPlace();
    return q;
  }

  inline Quaternion conjugate() const {
    return Quaternion(w, -x, -y, -z);
  }

  inline Quaternion inverseUnit() const {
    return conjugate();
  }

  static inline Quaternion mul(const Quaternion& a, const Quaternion& b) {
    return Quaternion(
      a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
      a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
      a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
      a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    );
  }

  static inline Quaternion fromAxisAngleDeg(float ax, float ay, float az, float deg) {
    const float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm <= 0.0f) return Quaternion();
    const float half = (deg * 0.01745329251994329577f) * 0.5f;
    const float s = sinf(half);
    Quaternion q(cosf(half), (ax/norm)*s, (ay/norm)*s, (az/norm)*s);
    q.normalizeInPlace();
    return q;
  }

  // Z * Y * X order; args are degrees.
  static inline Quaternion fromEulerZyxDeg(float zDeg, float yDeg, float xDeg) {
    Quaternion qx = fromAxisAngleDeg(1.0f, 0.0f, 0.0f, xDeg);
    Quaternion qy = fromAxisAngleDeg(0.0f, 1.0f, 0.0f, yDeg);
    Quaternion qz = fromAxisAngleDeg(0.0f, 0.0f, 1.0f, zDeg);
    Quaternion out = mul(qz, mul(qy, qx));
    out.normalizeInPlace();
    return out;
  }

  static inline float dot(const Quaternion& a, const Quaternion& b) {
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
  }

  static inline float absDistJavaStyle(const Quaternion& aNorm, const Quaternion& bNorm) {
    float dotv = dot(aNorm, bNorm);
    float v = 2.0f * dotv - 1.0f;
    if (v < -1.0f) v = -1.0f;
    if (v > 1.0f) v = 1.0f;
    float d = acosf(v);
    if (isnan(d)) return 0.0f;
    return d;
  }

  static inline float angularDistance(const Quaternion& aIn, const Quaternion& bIn) {
    Quaternion a = aIn.normalized();
    Quaternion b = bIn.normalized();
#if GAG_RECOG_USE_JAVA_DISTANCE
    return absDistJavaStyle(a, b);
#else
    float d = fabsf(dot(a, b));
    if (d > 1.0f) d = 1.0f;
    return 2.0f * acosf(d);
#endif
  }
};

enum class RotationAxis : uint8_t {
  X = 0,
  Y = 1,
  Z = 2
};

struct RotationVectorDeg {
  float x;
  float y;
  float z;

  RotationVectorDeg() : x(0), y(0), z(0) {}
  RotationVectorDeg(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float signf_or(float v, float fallback) {
  if (v > 0.0f) return 1.0f;
  if (v < 0.0f) return -1.0f;
  return (fallback < 0.0f) ? -1.0f : 1.0f;
}

static inline float rotationVectorComponent(const RotationVectorDeg& v, RotationAxis axis) {
  switch (axis) {
    case RotationAxis::X: return v.x;
    case RotationAxis::Y: return v.y;
    case RotationAxis::Z: return v.z;
    default: return 0.0f;
  }
}

static inline void setRotationVectorComponent(RotationVectorDeg& v, RotationAxis axis, float value) {
  switch (axis) {
    case RotationAxis::X: v.x = value; break;
    case RotationAxis::Y: v.y = value; break;
    case RotationAxis::Z: v.z = value; break;
    default: break;
  }
}

static inline RotationVectorDeg rotationVectorDegFromQuaternion(const Quaternion& qIn) {
  Quaternion q = qIn.normalized();
  if (q.w < 0.0f) {
    q.w = -q.w;
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
  }

  float w = clampf(q.w, -1.0f, 1.0f);
  const float halfAngle = acosf(w);
  const float sinHalf = sqrtf(fmaxf(0.0f, 1.0f - w * w));
  if (halfAngle <= 1e-6f || sinHalf <= 1e-6f) return RotationVectorDeg();

  const float angleDeg = halfAngle * 114.59155902616465f;
  const float axisScale = angleDeg / sinHalf;
  return RotationVectorDeg(q.x * axisScale, q.y * axisScale, q.z * axisScale);
}

static inline Quaternion quaternionFromRotationVectorDeg(const RotationVectorDeg& rvDeg) {
  const float angleDeg = sqrtf(rvDeg.x * rvDeg.x + rvDeg.y * rvDeg.y + rvDeg.z * rvDeg.z);
  if (angleDeg <= 1e-6f) return Quaternion();
  return Quaternion::fromAxisAngleDeg(rvDeg.x / angleDeg,
                                      rvDeg.y / angleDeg,
                                      rvDeg.z / angleDeg,
                                      angleDeg);
}

static inline RotationVectorDeg projectRotationVectorToPrimaryAxis(const RotationVectorDeg& in,
                                                                   RotationAxis primaryAxis,
                                                                   float secondaryLimitDeg,
                                                                   float snapDominanceRatio,
                                                                   float snapPrimaryBelowDeg,
                                                                   float primarySignHint = 1.0f) {
  RotationVectorDeg out = in;
  const float primaryRaw = rotationVectorComponent(in, primaryAxis);

  RotationAxis axisA = RotationAxis::X;
  RotationAxis axisB = RotationAxis::Y;
  switch (primaryAxis) {
    case RotationAxis::X: axisA = RotationAxis::Y; axisB = RotationAxis::Z; break;
    case RotationAxis::Y: axisA = RotationAxis::X; axisB = RotationAxis::Z; break;
    case RotationAxis::Z: axisA = RotationAxis::X; axisB = RotationAxis::Y; break;
    default: break;
  }

  const float offARaw = rotationVectorComponent(in, axisA);
  const float offBRaw = rotationVectorComponent(in, axisB);
  const float offMagRaw = sqrtf(offARaw * offARaw + offBRaw * offBRaw);
  const bool impossibleAxis =
    (offMagRaw > secondaryLimitDeg) &&
    (fabsf(primaryRaw) < snapPrimaryBelowDeg || fabsf(primaryRaw) < offMagRaw * snapDominanceRatio);

  if (impossibleAxis) {
    const float totalMag = sqrtf(primaryRaw * primaryRaw + offARaw * offARaw + offBRaw * offBRaw);
    out = RotationVectorDeg();
    setRotationVectorComponent(out, primaryAxis, signf_or(primaryRaw, primarySignHint) * totalMag);
    return out;
  }

  setRotationVectorComponent(out, axisA, clampf(offARaw, -secondaryLimitDeg, secondaryLimitDeg));
  setRotationVectorComponent(out, axisB, clampf(offBRaw, -secondaryLimitDeg, secondaryLimitDeg));
  return out;
}


struct AccelData {
  float x;
  float y;
  float z;

  AccelData() : x(0), y(0), z(0) {}
  AccelData(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct RecogData {
  const Quaternion* q = nullptr;
  const AccelData* a = nullptr;

  static inline RecogData fromQuat(const Quaternion& qq) {
    RecogData d; d.q = &qq; return d;
  }
  static inline RecogData fromAccel(const AccelData& aa) {
    RecogData d; d.a = &aa; return d;
  }
  inline bool isQuat() const { return q != nullptr && a == nullptr; }
  inline bool isAccel() const { return a != nullptr && q == nullptr; }
};

struct SensorGestureData {
  uint8_t len = 0;
  Quaternion q[GAG_RECOG_MAX_QUATS_PER_SENSOR];
};

struct SensorAccelGestureData {
  uint8_t len = 0;
  AccelData a[GAG_RECOG_MAX_QUATS_PER_SENSOR];
};

enum class MouseActionType : uint8_t {
  NONE = 0,
  MOVE,
  CLICK,
  PRESS,
  RELEASE,
  SCROLL
};

struct MouseAction {
  MouseActionType type = MouseActionType::NONE;
  int8_t dx = 0;
  int8_t dy = 0;
  int8_t wheel = 0;
  int8_t hWheel = 0;
  uint8_t button = 0;
};

struct GestureAction {
  bool switch_visualization_mode = false;
  bool blink_visualization = false;
  bool toggle_wrist_mouse_emulation = false;
  bool start_finger_drift_reset_sequence = false;
  uint16_t blink_color565 = 0xFFFF;

  bool vibrate = false;
  uint8_t vibrate_sensor_mask = 0;
  uint16_t vibrate_duration_ms = 0;

  MouseAction mouse;

  bool log_to_history = true;
};

struct GestureDef {
  char name[GAG_RECOG_MAX_NAME_LEN] = {0};
  char command[GAG_RECOG_MAX_CMD_LEN] = {0};
  char label[GAG_RECOG_MAX_LABEL_LEN] = {0};

  float threshold_rad = 0.0f;
  float threshold_accel = 0.0f;
  uint32_t recognition_delay_ms = 0;
  uint32_t max_time_ms = 0;
  bool relative = false;
  bool active = true;

  SensorGestureData perSensor[static_cast<uint8_t>(Sensor::COUNT)];
  SensorAccelGestureData perSensorAccel[static_cast<uint8_t>(Sensor::COUNT)];
  GestureAction action;

  inline uint8_t requiredSensorMask() const {
    uint8_t m = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(Sensor::COUNT); ++i) {
      const bool hasQuat = perSensor[i].len > 0;
      const bool hasAcc  = perSensorAccel[i].len > 0;
      if (!hasQuat && !hasAcc) continue;
      if (relative && i == static_cast<uint8_t>(Sensor::WRIST) && hasQuat && !hasAcc) continue;
      m |= (1u << i);
    }
    return m;
  }

  inline uint16_t requiredTrackMask() const {
    uint16_t m = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(Sensor::COUNT); ++i) {
      if (perSensor[i].len > 0) {
        if (!(relative && i == static_cast<uint8_t>(Sensor::WRIST))) {
          m |= (uint16_t)(1u << i);
        }
      }
      if (perSensorAccel[i].len > 0) {
        m |= (uint16_t)(1u << (i + static_cast<uint8_t>(Sensor::COUNT)));
      }
    }
    return m;
  }

  inline uint8_t sensorMask() const {
    uint8_t m = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(Sensor::COUNT); ++i) {
      if (perSensor[i].len > 0 || perSensorAccel[i].len > 0) m |= (1u << i);
    }
    return m;
  }

  inline void normalizeAllKeyframes() {
    for (uint8_t i = 0; i < static_cast<uint8_t>(Sensor::COUNT); ++i) {
      for (uint8_t k = 0; k < perSensor[i].len; ++k) {
        perSensor[i].q[k].normalizeInPlace();
      }
    }
  }
};

struct RecognizedGesture {
  const char* name = nullptr;
  const char* command = nullptr;
  const char* label = nullptr;
  const GestureAction* action = nullptr;
  uint8_t sensor_mask = 0;
  uint32_t start_ms = 0;
  uint32_t end_ms = 0;
  uint32_t duration_ms = 0;
};

typedef void (*OnRecognizedCallback)(const RecognizedGesture& g);

class Recognizer {
public:
  Recognizer() { clear(); }

  void begin(Print& out = Serial) {
    _out = &out;
  }

  void clear() {
    _count = 0;
    _haveWrist = false;
    _lastWristNorm = Quaternion();
    _lastWristTimeMs = 0;
    clearAllRuntimes();
  }

  uint8_t count() const { return _count; }

  bool addGesture(const GestureDef& defIn) {
    if (_count >= GAG_RECOG_MAX_GESTURES) return false;
    if (defIn.requiredTrackMask() == 0) return false;

    GestureDef def = defIn;
    def.normalizeAllKeyframes();
    _gestures[_count] = def;
    clearRuntime(_count);
    ++_count;
    return true;
  }

  void addSampleGestures() {
    // Intentionally empty in the refactored version.
  }

  void setOnRecognized(OnRecognizedCallback cb) {
    _cb = cb;
  }

  void processSample(Sensor sensor, const Quaternion& q, uint32_t now_ms = 0) {
    processSample(sensor, RecogData::fromQuat(q), now_ms);
  }

  void processSample(Sensor sensor, const RecogData& data, uint32_t now_ms = 0) {
    if (data.isQuat()) {
      Quaternion qNorm = *data.q;
      qNorm.normalizeInPlace();
      if (sensor == Sensor::WRIST) {
        _lastWristNorm = qNorm;
        _haveWrist = true;
        _lastWristTimeMs = now_ms;
      }
      for (uint8_t gi = 0; gi < _count; ++gi) {
        if (!_gestures[gi].active) continue;
        handleQuatUpdate(gi, sensor, qNorm, now_ms);
        maybeRecognize(gi, now_ms);
      }
    } else if (data.isAccel()) {
      for (uint8_t gi = 0; gi < _count; ++gi) {
        if (!_gestures[gi].active) continue;
        handleAccelUpdate(gi, sensor, *data.a, now_ms);
        maybeRecognize(gi, now_ms);
      }
    }
  }

  void printGestures() const {
    if (!_out) return;
    _out->println("GAG gestures:");
    for (uint8_t i = 0; i < _count; ++i) {
      _out->print(" - ");
      _out->print(_gestures[i].name);
      _out->print(" cmd=");
      _out->print(_gestures[i].command);
      _out->print(" mask=");
      _out->println((unsigned)_gestures[i].sensorMask());
    }
  }

  int8_t findGestureIndexByName(const char* name) const {
    if (!name) return -1;
    for (uint8_t i = 0; i < _count; ++i) {
      if (streq(_gestures[i].name, name)) return (int8_t)i;
    }
    return -1;
  }

  uint8_t getGestureSensorMaskByName(const char* name) const {
    const int8_t idx = findGestureIndexByName(name);
    return (idx < 0) ? 0 : _gestures[(uint8_t)idx].sensorMask();
  }

private:
  struct PartialMatcher {
    bool active = false;
    uint8_t index = 0;     // next reference index expected
    uint32_t start_ms = 0;
  };

  struct SensorRuntime {
    PartialMatcher m[GAG_RECOG_MAX_MATCHERS_PER_SENSOR];
    bool completed = false;
    uint32_t completed_start_ms = 0;
    uint32_t completed_end_ms = 0;
  };

  struct GestureRuntime {
    uint32_t cooldown_until_ms = 0;
    SensorRuntime qrt[static_cast<uint8_t>(Sensor::COUNT)];
    SensorRuntime art[static_cast<uint8_t>(Sensor::COUNT)];
  };

  GestureDef _gestures[GAG_RECOG_MAX_GESTURES];
  GestureRuntime _rt[GAG_RECOG_MAX_GESTURES];
  uint8_t _count = 0;

  Print* _out = &Serial;
  OnRecognizedCallback _cb = nullptr;

  Quaternion _lastWristNorm;
  bool _haveWrist = false;
  uint32_t _lastWristTimeMs = 0;

  inline bool isCooldownActive(uint8_t gi, uint32_t now) const {
    return (int32_t)(now - _rt[gi].cooldown_until_ms) < 0;
  }

  inline void clearRuntimeState(uint8_t gi) {
    for (uint8_t s = 0; s < static_cast<uint8_t>(Sensor::COUNT); ++s) {
      _rt[gi].qrt[s].completed = false;
      _rt[gi].qrt[s].completed_start_ms = 0;
      _rt[gi].qrt[s].completed_end_ms = 0;
      _rt[gi].art[s].completed = false;
      _rt[gi].art[s].completed_start_ms = 0;
      _rt[gi].art[s].completed_end_ms = 0;
      for (uint8_t m = 0; m < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++m) {
        _rt[gi].qrt[s].m[m] = PartialMatcher();
        _rt[gi].art[s].m[m] = PartialMatcher();
      }
    }
  }

  inline void clearRuntime(uint8_t gi) {
    _rt[gi].cooldown_until_ms = 0;
    clearRuntimeState(gi);
  }

  inline bool isNamedExclusivePair(const GestureDef& a, const GestureDef& b,
                                   const char* lhs, const char* rhs) const {
    return (streq(a.command, lhs) && streq(b.command, rhs)) ||
           (streq(a.command, rhs) && streq(b.command, lhs));
  }

  inline bool areMutuallyExclusive(const GestureDef& a, const GestureDef& b) const {
    return isNamedExclusivePair(a, b, "TOGGLE_WRIST_MOUSE", "START_FINGER_DRIFT_RESET") ||
           isNamedExclusivePair(a, b, "MOUSE_LEFT_DOUBLE_CLICK", "MOUSE_MIDDLE_CLICK");
  }

  inline void suppressMutuallyExclusiveGestures(uint8_t recognizedGi, uint32_t now) {
    const GestureDef& recognized = _gestures[recognizedGi];
    for (uint8_t gi = 0; gi < _count; ++gi) {
      if (gi == recognizedGi || !_gestures[gi].active) continue;
      if (!areMutuallyExclusive(recognized, _gestures[gi])) continue;
      clearRuntimeState(gi);
      _rt[gi].cooldown_until_ms = now + _gestures[gi].recognition_delay_ms;
    }
  }

  inline void clearAllRuntimes() {
    for (uint8_t i = 0; i < GAG_RECOG_MAX_GESTURES; ++i) clearRuntime(i);
  }

  inline bool doesMatchQuat(uint8_t gi, Sensor sensor, uint8_t refIndex,
                            const Quaternion& refNorm, const Quaternion& sampleNorm) const {
    const GestureDef& g = _gestures[gi];
    Quaternion refCmp = refNorm;
    Quaternion sampleCmp = sampleNorm;

    if (g.relative && sensor != Sensor::WRIST && g.perSensor[(uint8_t)Sensor::WRIST].len > 0 && _haveWrist) {
      const SensorGestureData& wristTrack = g.perSensor[(uint8_t)Sensor::WRIST];
      const uint8_t wRefIdx = (refIndex < wristTrack.len) ? refIndex : (uint8_t)(wristTrack.len - 1);
      const Quaternion refWrist = wristTrack.q[wRefIdx].normalized();
      refCmp = Quaternion::mul(refWrist.inverseUnit(), refNorm);
      sampleCmp = Quaternion::mul(_lastWristNorm.inverseUnit(), sampleNorm);
    }

    const float d = Quaternion::angularDistance(refCmp, sampleCmp);
    return d <= g.threshold_rad;
  }

  inline bool doesMatchAccel(uint8_t gi, Sensor /*sensor*/, uint8_t /*refIndex*/,
                             const AccelData& ref, const AccelData& sample) const {
    const GestureDef& g = _gestures[gi];
    const float dx = sample.x - ref.x;
    const float dy = sample.y - ref.y;
    const float dz = sample.z - ref.z;
    const float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    return dist <= g.threshold_accel;
  }

  inline void expireMatchers(PartialMatcher* arr, uint32_t max_time_ms, uint32_t now) {
    if (max_time_ms == 0) return;
    for (uint8_t i = 0; i < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++i) {
      if (!arr[i].active) continue;
      if ((uint32_t)(now - arr[i].start_ms) > max_time_ms) {
        arr[i].active = false;
      }
    }
  }

  inline void markSensorComplete(SensorRuntime& rt, uint32_t start_ms, uint32_t end_ms) {
    if (!rt.completed || (int32_t)(start_ms - rt.completed_start_ms) < 0) {
      rt.completed_start_ms = start_ms;
      rt.completed_end_ms = end_ms;
      rt.completed = true;
    }
  }

  inline void startMatcher(PartialMatcher* arr, uint32_t now) {
    for (uint8_t i = 0; i < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++i) {
      if (!arr[i].active) {
        arr[i].active = true;
        arr[i].index = 1;
        arr[i].start_ms = now;
        return;
      }
    }
    // If full, recycle the oldest matcher.
    uint8_t oldest = 0;
    uint32_t oldestStart = arr[0].start_ms;
    for (uint8_t i = 1; i < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++i) {
      if (arr[i].start_ms < oldestStart) { oldestStart = arr[i].start_ms; oldest = i; }
    }
    arr[oldest].active = true;
    arr[oldest].index = 1;
    arr[oldest].start_ms = now;
  }

  void handleQuatUpdate(uint8_t gi, Sensor sensor, const Quaternion& sampleNorm, uint32_t now) {
    const uint8_t si = static_cast<uint8_t>(sensor);
    const GestureDef& g = _gestures[gi];
    if (g.perSensor[si].len == 0) return;
    if (isCooldownActive(gi, now)) return;

    SensorRuntime& srt = _rt[gi].qrt[si];
    const SensorGestureData& track = g.perSensor[si];

    expireMatchers(srt.m, g.max_time_ms, now);

    // Advance currently active partial matches.
    for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
      PartialMatcher& pm = srt.m[mi];
      if (!pm.active) continue;
      if (pm.index >= track.len) continue;
      if (doesMatchQuat(gi, sensor, pm.index, track.q[pm.index], sampleNorm)) {
        ++pm.index;
        if (pm.index >= track.len) {
          markSensorComplete(srt, pm.start_ms, now);
          pm.active = false;
        }
      }
    }

    // Start new matchers if ref[0] matches the sample.
    if (doesMatchQuat(gi, sensor, 0, track.q[0], sampleNorm)) {
      if (track.len <= 1) {
        markSensorComplete(srt, now, now);
      } else {
        startMatcher(srt.m, now);
      }
    }
  }

  void handleAccelUpdate(uint8_t gi, Sensor sensor, const AccelData& sample, uint32_t now) {
    const uint8_t si = static_cast<uint8_t>(sensor);
    const GestureDef& g = _gestures[gi];
    if (g.perSensorAccel[si].len == 0) return;
    if (isCooldownActive(gi, now)) return;

    SensorRuntime& srt = _rt[gi].art[si];
    const SensorAccelGestureData& track = g.perSensorAccel[si];

    expireMatchers(srt.m, g.max_time_ms, now);

    for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
      PartialMatcher& pm = srt.m[mi];
      if (!pm.active) continue;
      if (pm.index >= track.len) continue;
      if (doesMatchAccel(gi, sensor, pm.index, track.a[pm.index], sample)) {
        ++pm.index;
        if (pm.index >= track.len) {
          markSensorComplete(srt, pm.start_ms, now);
          pm.active = false;
        }
      }
    }

    if (doesMatchAccel(gi, sensor, 0, track.a[0], sample)) {
      if (track.len <= 1) {
        markSensorComplete(srt, now, now);
      } else {
        startMatcher(srt.m, now);
      }
    }
  }

  void maybeRecognize(uint8_t gi, uint32_t now) {
    const GestureDef& g = _gestures[gi];
    if (isCooldownActive(gi, now)) return;

    const uint16_t required = g.requiredTrackMask();
    uint16_t completed = 0;
    uint32_t start_ms = 0xFFFFFFFFu;
    uint32_t end_ms = 0;

    for (uint8_t s = 0; s < static_cast<uint8_t>(Sensor::COUNT); ++s) {
      if (required & (uint16_t)(1u << s)) {
        const SensorRuntime& rt = _rt[gi].qrt[s];
        if (!rt.completed) return;
        completed |= (uint16_t)(1u << s);
        if (rt.completed_start_ms < start_ms) start_ms = rt.completed_start_ms;
        if (rt.completed_end_ms > end_ms) end_ms = rt.completed_end_ms;
      }
      if (required & (uint16_t)(1u << (s + static_cast<uint8_t>(Sensor::COUNT)))) {
        const SensorRuntime& rt = _rt[gi].art[s];
        if (!rt.completed) return;
        completed |= (uint16_t)(1u << (s + static_cast<uint8_t>(Sensor::COUNT)));
        if (rt.completed_start_ms < start_ms) start_ms = rt.completed_start_ms;
        if (rt.completed_end_ms > end_ms) end_ms = rt.completed_end_ms;
      }
    }

    if (completed != required) return;
    if (g.max_time_ms != 0 && (uint32_t)(end_ms - start_ms) > g.max_time_ms) {
      clearRuntime(gi);
      return;
    }

    RecognizedGesture rg;
    rg.name = g.name;
    rg.command = g.command;
    rg.label = g.label;
    rg.action = &g.action;
    rg.sensor_mask = g.sensorMask();
    rg.start_ms = start_ms;
    rg.end_ms = end_ms;
    rg.duration_ms = end_ms - start_ms;

    printRecognized(rg);
    if (_cb) _cb(rg);

    suppressMutuallyExclusiveGestures(gi, now);
    _rt[gi].cooldown_until_ms = now + g.recognition_delay_ms;
    clearRuntimeState(gi);
  }

  void printRecognized(const RecognizedGesture& rg) {
    if (!_out) return;
    _out->print("GAG:RECOG name=");
    _out->print(rg.name ? rg.name : "");
    _out->print(" cmd=");
    _out->print(rg.command ? rg.command : "");
    _out->print(" start_ms=");
    _out->print((unsigned long)rg.start_ms);
    _out->print(" end_ms=");
    _out->print((unsigned long)rg.end_ms);
    _out->print(" dur_ms=");
    _out->println((unsigned long)rg.duration_ms);
  }
};

class SerialLoader {
public:
  explicit SerialLoader(Recognizer& r) : _r(r) {
    _line[0] = '\0';
  }

  #ifdef ARDUINO
  void begin(Stream& in = Serial, Print& out = Serial) {
#else
  void begin(Stream& in = GAG_DUMMY_STREAM, Print& out = Serial) {
#endif
    _in = &in;
    _out = &out;
    _pos = 0;
    _line[0] = '\0';
    _b = Builder();
  }

  void poll() {
    if (!_in) return;
    while (_in->available() > 0) {
      int ch = _in->read();
      if (ch < 0) break;
      if (ch == '\r') continue;
      if (ch == '\n') {
        if (_pos == 0) continue;
        _line[_pos] = '\0';
        processLine(_line);
        _pos = 0;
        _line[0] = '\0';
        continue;
      }
      if (_pos + 1 >= LINE_BUF) {
        _pos = 0;
        _line[0] = '\0';
        _b = Builder();
        return;
      }
      _line[_pos++] = (char)ch;
    }
  }

private:
  Recognizer& _r;
  Stream* _in = nullptr;
  Print* _out = &Serial;

  static constexpr size_t LINE_BUF = 200;
  char _line[LINE_BUF];
  size_t _pos = 0;

  struct Builder {
    bool active = false;
    GestureDef g;
    bool haveSensor = false;
    Sensor currentSensor = Sensor::WRIST;
    enum class Track : uint8_t { NONE = 0, QUAT = 1, ACCEL = 2 };
    Track currentTrack = Track::NONE;
    uint8_t expected = 0;
    uint8_t received = 0;
  } _b;

  static inline void normalizeWhitespace(char* s) {
    if (!s) return;
    char* dst = s;
    bool inSpace = false;
    while (*s) {
      const bool isSpace = (*s == ' ' || *s == '\t');
      if (isSpace) {
        if (!inSpace) *dst++ = ' ';
      } else {
        *dst++ = *s;
      }
      inSpace = isSpace;
      ++s;
    }
    if (dst > s && dst[-1] == ' ') --dst;
    *dst = '\0';
  }

  bool parseSensor(const char* tok, Sensor& s) const {
    if (!tok) return false;
    if (streq(tok, "WRIST") || streq(tok, "0")) { s = Sensor::WRIST; return true; }
    if (streq(tok, "THUMB") || streq(tok, "1")) { s = Sensor::THUMB; return true; }
    if (streq(tok, "INDEX") || streq(tok, "2")) { s = Sensor::INDEX; return true; }
    if (streq(tok, "MIDDLE") || streq(tok, "3")) { s = Sensor::MIDDLE; return true; }
    if (streq(tok, "RING") || streq(tok, "4")) { s = Sensor::RING; return true; }
    if (streq(tok, "LITTLE") || streq(tok, "5")) { s = Sensor::LITTLE; return true; }
    return false;
  }

  void printHelp() const {
    if (!_out) return;
    _out->println("GAG HELP");
    _out->println("GAG BEGIN <name> <cmd> <label> <thr_rad> <delay_ms> <max_time_ms> <active0|1> [relative0|1] [thr_accel]");
    _out->println("GAG SENSOR <WRIST|THUMB|INDEX|MIDDLE|RING|LITTLE|0..5> <count>");
    _out->println("GAG Q <w> <x> <y> <z>");
    _out->println("GAG A <x> <y> <z>");
    _out->println("GAG END | GAG ABORT | GAG LIST | GAG CLEAR");
  }

  bool processLine(char* line) {
    normalizeWhitespace(line);
    if (strncmp(line, "GAG ", 4) != 0 && strcmp(line, "GAG") != 0) return false;

    char* save = nullptr;
    char* tok = strtok_r(line, " ", &save); // GAG
    (void)tok;
    tok = strtok_r(nullptr, " ", &save);
    if (!tok) return false;

    if (streq(tok, "HELP")) { printHelp(); return true; }
    if (streq(tok, "CLEAR")) { _r.clear(); _b = Builder(); return true; }
    if (streq(tok, "LIST")) { _r.printGestures(); return true; }
    if (streq(tok, "ABORT")) { _b = Builder(); return true; }

    if (streq(tok, "BEGIN")) {
      Builder b;
      b.active = true;
      const char* name = strtok_r(nullptr, " ", &save);
      const char* cmd  = strtok_r(nullptr, " ", &save);
      const char* lbl  = strtok_r(nullptr, " ", &save);
      const char* thr  = strtok_r(nullptr, " ", &save);
      const char* delay= strtok_r(nullptr, " ", &save);
      const char* maxT = strtok_r(nullptr, " ", &save);
      const char* active = strtok_r(nullptr, " ", &save);
      const char* relative = strtok_r(nullptr, " ", &save);
      const char* thrAccel = strtok_r(nullptr, " ", &save);
      if (!name || !cmd || !lbl || !thr || !delay || !maxT || !active) {
        _b = Builder();
        return false;
      }
      strncpy(b.g.name, name, sizeof(b.g.name)-1);
      strncpy(b.g.command, cmd, sizeof(b.g.command)-1);
      strncpy(b.g.label, lbl, sizeof(b.g.label)-1);
      b.g.threshold_rad = (float)atof(thr);
      b.g.recognition_delay_ms = (uint32_t)strtoul(delay, nullptr, 10);
      b.g.max_time_ms = (uint32_t)strtoul(maxT, nullptr, 10);
      b.g.active = atoi(active) != 0;
      b.g.relative = (relative ? (atoi(relative) != 0) : false);
      b.g.threshold_accel = (thrAccel ? (float)atof(thrAccel) : 0.0f);
      _b = b;
      return true;
    }

    if (streq(tok, "SENSOR")) {
      if (!_b.active) return false;
      const char* sTok = strtok_r(nullptr, " ", &save);
      const char* cTok = strtok_r(nullptr, " ", &save);
      if (!sTok || !cTok) return false;
      Sensor s;
      if (!parseSensor(sTok, s)) return false;
      _b.currentSensor = s;
      _b.haveSensor = true;
      _b.currentTrack = Builder::Track::NONE;
      _b.expected = (uint8_t)atoi(cTok);
      _b.received = 0;
      return true;
    }

    if (streq(tok, "Q")) {
      if (!_b.active || !_b.haveSensor) return false;
      const char* w = strtok_r(nullptr, " ", &save);
      const char* x = strtok_r(nullptr, " ", &save);
      const char* y = strtok_r(nullptr, " ", &save);
      const char* z = strtok_r(nullptr, " ", &save);
      if (!w || !x || !y || !z) return false;
      SensorGestureData& tr = _b.g.perSensor[(uint8_t)_b.currentSensor];
      if (tr.len >= GAG_RECOG_MAX_QUATS_PER_SENSOR) return false;
      tr.q[tr.len++] = Quaternion((float)atof(w), (float)atof(x), (float)atof(y), (float)atof(z));
      _b.currentTrack = Builder::Track::QUAT;
      ++_b.received;
      return true;
    }

    if (streq(tok, "A")) {
      if (!_b.active || !_b.haveSensor) return false;
      const char* x = strtok_r(nullptr, " ", &save);
      const char* y = strtok_r(nullptr, " ", &save);
      const char* z = strtok_r(nullptr, " ", &save);
      if (!x || !y || !z) return false;
      SensorAccelGestureData& tr = _b.g.perSensorAccel[(uint8_t)_b.currentSensor];
      if (tr.len >= GAG_RECOG_MAX_QUATS_PER_SENSOR) return false;
      tr.a[tr.len++] = AccelData((float)atof(x), (float)atof(y), (float)atof(z));
      _b.currentTrack = Builder::Track::ACCEL;
      ++_b.received;
      return true;
    }

    if (streq(tok, "END")) {
      if (!_b.active) return false;
      const bool ok = _r.addGesture(_b.g);
      _b = Builder();
      return ok;
    }

    return false;
  }
};

} // namespace gag
