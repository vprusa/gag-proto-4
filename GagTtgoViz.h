#pragma once
/**
 * GagTtgoViz.h
 *
 * Single-file TTGO/T-Display visualization:
 *  - upper region: hand skeleton on the left, fingers pointing up
 *  - lower-middle region: 2 rows × 4 sensor cubes
 *  - bottom region: command / recognized-gesture log
 *  - round-robin display modes prepared for future button / gesture switching
 */

#ifdef ARDUINO
#include <Arduino.h>
#include <TFT_eSPI.h>

#ifndef GAG_ENABLE_RAW_CUBES_VISUALIZATION
#define GAG_ENABLE_RAW_CUBES_VISUALIZATION 0
#endif

#include "GagRecog.h"

namespace gag {
namespace viz {

struct Vec3 {
  float x;
  float y;
  float z;
};

static inline Vec3 v3(float x, float y, float z) {
  Vec3 v{x,y,z};
  return v;
}

static inline Vec3 add(const Vec3& a, const Vec3& b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline Vec3 scale(const Vec3& a, float s) { return v3(a.x * s, a.y * s, a.z * s); }

static inline Vec3 rotate(const Quaternion& qIn, const Vec3& v) {
  Quaternion q = qIn;
  q.normalizeInPlace();
  const float qx = q.x, qy = q.y, qz = q.z, qw = q.w;
  const float tx = 2.0f * (qy * v.z - qz * v.y);
  const float ty = 2.0f * (qz * v.x - qx * v.z);
  const float tz = 2.0f * (qx * v.y - qy * v.x);
  return v3(
    v.x + qw * tx + (qy * tz - qz * ty),
    v.y + qw * ty + (qz * tx - qx * tz),
    v.z + qw * tz + (qx * ty - qy * tx)
  );
}

enum Mode : uint8_t {
  MODE_FULL = 0,
  MODE_SKELETON_ONLY = 1,
  MODE_CUBES_ONLY = 2,
  MODE_LOG_ONLY = 3,
  MODE_COUNT = 4
};

struct FrameInput {
  Quaternion sensor_q[8];
  Quaternion raw_sensor_q[8];
  bool present[8];
  uint16_t base_color[8];
  uint8_t sensor_count = 0;

  FrameInput() : sensor_count(0) {
    for (uint8_t i = 0; i < 8; ++i) {
      sensor_q[i] = Quaternion();
      raw_sensor_q[i] = Quaternion();
      present[i] = false;
      base_color[i] = TFT_LIGHTGREY;
    }
  }
};

class TtgoDisplayViz {
public:
  void begin(TFT_eSPI& tft, uint16_t bgColor = TFT_BLACK) {
    _tft = &tft;
    _bg = bgColor;
    _mode = MODE_FULL;
    _flashMask = 0;
    _flashUntil = 0;
    _flashColor = TFT_YELLOW;
    _logCount = 0;
    _logWrite = 0;
    for (uint8_t i = 0; i < kLogLines; ++i) {
      _log[i][0] = '\0';
    }
  }

  void setMode(uint8_t mode) {
    _mode = (uint8_t)(mode % MODE_COUNT);
  }

  uint8_t mode() const {
    return _mode;
  }

  void nextMode() {
    _mode = (uint8_t)((_mode + 1u) % MODE_COUNT);
  }

  void flash(uint8_t sensorMask, uint16_t color565, uint32_t durationMs) {
    _flashMask = sensorMask;
    _flashColor = color565;
    _flashUntil = millis() + durationMs;
  }

  void pushLog(const char* line) {
    if (!line || !line[0]) return;
    strncpy(_log[_logWrite], line, kLogLineLen - 1);
    _log[_logWrite][kLogLineLen - 1] = '\0';
    _logWrite = (uint8_t)((_logWrite + 1u) % kLogLines);
    if (_logCount < kLogLines) ++_logCount;
  }

  void draw(const FrameInput& in) {
    if (!_tft) return;
    const uint16_t W = _tft->width();
    const uint16_t H = _tft->height();

    _tft->fillScreen(_bg);

    const bool flashActive = (_flashUntil != 0u) && ((int32_t)(millis() - _flashUntil) < 0);
    const bool blinkOn = flashActive && (((millis() >> 6) & 1u) == 0u);
    if (!flashActive) _flashMask = 0;

    const int skeletonTop = 0;
    const int skeletonH   = 72;
    const int cubesTop    = skeletonTop + skeletonH;
#if GAG_ENABLE_RAW_CUBES_VISUALIZATION
    const int logMinH     = 32;
    const int cubeGridCount = 2;
    int cubesAreaH = (int)H - cubesTop - logMinH;
    if (cubesAreaH < cubeGridCount * 24) cubesAreaH = cubeGridCount * 24;
    const int cubesH = cubesAreaH / cubeGridCount;
    const int rawCubesTop = cubesTop + cubesH;
    const int rawCubesH = cubesAreaH - cubesH;
    const int logTop = cubesTop + cubesAreaH;
#else
    const int cubesH      = 80;
    const int rawCubesTop = cubesTop + cubesH;
    const int rawCubesH   = 0;
    const int logTop      = cubesTop + cubesH;
#endif
    const int logH        = H - logTop;

    if (_mode == MODE_FULL || _mode == MODE_SKELETON_ONLY) {
      drawSkeleton(in, 0, skeletonTop, W, skeletonH, flashActive, blinkOn);
      drawModeBadge(W - 54, 2);
    }

    if (_mode == MODE_FULL || _mode == MODE_CUBES_ONLY) {
      drawCubeGrid(in, in.sensor_q, 0, cubesTop, W, cubesH, flashActive, blinkOn);
#if GAG_ENABLE_RAW_CUBES_VISUALIZATION
      drawCubeGrid(in, in.raw_sensor_q, 0, rawCubesTop, W, rawCubesH, flashActive, blinkOn, "RAW");
#endif
      drawModeBadge(W - 54, 2);
    }

    if (_mode == MODE_FULL || _mode == MODE_LOG_ONLY || _mode == MODE_SKELETON_ONLY || _mode == MODE_CUBES_ONLY) {
      drawLogArea(0, logTop, W, logH);
    }
  }

private:
  static constexpr uint8_t kLogLines = 5;
  static constexpr size_t kLogLineLen = 28;

  TFT_eSPI* _tft = nullptr;
  uint16_t _bg = TFT_BLACK;
  uint8_t _mode = MODE_FULL;

  uint8_t _flashMask = 0;
  uint16_t _flashColor = TFT_YELLOW;
  uint32_t _flashUntil = 0;

  char _log[kLogLines][kLogLineLen];
  uint8_t _logWrite = 0;
  uint8_t _logCount = 0;

  static const char* modeName(uint8_t mode) {
    switch (mode) {
      case MODE_FULL: return "FULL";
      case MODE_SKELETON_ONLY: return "HAND";
      case MODE_CUBES_ONLY: return "CUBE";
      case MODE_LOG_ONLY: return "LOG";
      default: return "?";
    }
  }

  static const char* sensorLabel(uint8_t idx) {
    switch (idx) {
      case 0: return "W0";
      case 1: return "T";
      case 2: return "I";
      case 3: return "M";
      case 4: return "R";
      case 5: return "L";
      case 6: return "W1";
      case 7: return "W2";
      default: return "-";
    }
  }

  void drawModeBadge(int x, int y) {
    _tft->drawRoundRect(x, y, 50, 12, 2, TFT_DARKGREY);
    _tft->setTextColor(TFT_WHITE, _bg);
    _tft->setTextFont(1);
    _tft->setCursor(x + 4, y + 3);
    _tft->print(modeName(_mode));
  }

  void projectSkeletonPoint(const Vec3& p, int areaX, int areaY, int areaW, int areaH, int& sx, int& sy) {
    const float cx = areaX + areaW * 0.30f;
    const float cy = areaY + areaH * 0.90f;
    const float scale = areaH * 0.75f / 58.0f;
    sx = (int)(cx + scale * (p.x + 0.25f * p.z));
    sy = (int)(cy - scale * (p.y - 0.15f * p.z));
  }

  uint16_t colorForSensor(const FrameInput& in, uint8_t idx, bool flashActive) const {
    if (flashActive && (_flashMask & (1u << idx))) return _flashColor;
    return in.base_color[idx];
  }

  void drawTipBlink(int x, int y, uint16_t color) {
    _tft->drawLine(x - 3, y - 3, x + 3, y + 3, color);
    _tft->drawLine(x - 3, y + 3, x + 3, y - 3, color);
  }

  void drawSkeleton(const FrameInput& in, int x, int y, int w, int h, bool flashActive, bool blinkOn) {
    _tft->drawFastHLine(x, y + h - 1, w, TFT_DARKGREY);
    _tft->setTextFont(1);
    _tft->setTextColor(TFT_LIGHTGREY, _bg);
    _tft->setCursor(x + 4, y + 4);
    _tft->print("HAND");

    const Quaternion qWrist = in.present[0] ? in.sensor_q[0] : Quaternion();

    const Vec3 wrist = v3(0.0f, 0.0f, 0.0f);
    const Vec3 bases[5] = {
      v3(-18.0f, 12.0f, 0.0f),
      v3(-9.0f, 16.0f, 0.0f),
      v3(0.0f, 18.0f, 0.0f),
      v3(9.0f, 16.0f, 0.0f),
      v3(18.0f, 13.0f, 0.0f)
    };
    const float segLen[5][3] = {
      {12.0f, 10.0f, 8.0f},
      {14.0f, 13.0f, 10.0f},
      {16.0f, 14.0f, 10.0f},
      {15.0f, 13.0f, 9.0f},
      {13.0f, 11.0f, 8.0f}
    };
    const Vec3 baseDir[5] = {
      v3(-0.58f, 0.82f, 0.0f),
      v3(0.0f, 1.0f, 0.0f),
      v3(0.0f, 1.0f, 0.0f),
      v3(0.0f, 1.0f, 0.0f),
      v3(0.0f, 1.0f, 0.0f)
    };

    int wx, wy;
    projectSkeletonPoint(wrist, x, y, w, h, wx, wy);
    _tft->fillCircle(wx, wy, 2, colorForSensor(in, 0, flashActive));

    for (uint8_t finger = 0; finger < 5; ++finger) {
      const uint8_t sensorIdx = (uint8_t)(finger + 1u);
      Quaternion qRel = Quaternion();
      if (in.present[sensorIdx]) {
        qRel = Quaternion::mul(qWrist.inverseUnit(), in.sensor_q[sensorIdx]);
        qRel.normalizeInPlace();
      }
      Vec3 dir = rotate(qRel, baseDir[finger]);
      const float norm = sqrtf(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
      if (norm > 0.0001f) {
        dir.x /= norm; dir.y /= norm; dir.z /= norm;
      } else {
        dir = baseDir[finger];
      }

      const Vec3 p0 = bases[finger];
      const Vec3 p1 = add(p0, scale(dir, segLen[finger][0]));
      const Vec3 p2 = add(p1, scale(dir, segLen[finger][1]));
      const Vec3 p3 = add(p2, scale(dir, segLen[finger][2]));
      const uint16_t col = colorForSensor(in, sensorIdx, flashActive);

      int x0,y0,x1,y1,x2,y2,x3,y3;
      projectSkeletonPoint(p0, x, y, w, h, x0, y0);
      projectSkeletonPoint(p1, x, y, w, h, x1, y1);
      projectSkeletonPoint(p2, x, y, w, h, x2, y2);
      projectSkeletonPoint(p3, x, y, w, h, x3, y3);

      _tft->drawLine(wx, wy, x0, y0, TFT_DARKGREY);
      _tft->drawLine(x0, y0, x1, y1, col);
      _tft->drawLine(x1, y1, x2, y2, col);
      _tft->drawLine(x2, y2, x3, y3, col);
      _tft->fillCircle(x0, y0, 1, TFT_DARKGREY);
      if (flashActive && blinkOn && (_flashMask & (1u << sensorIdx))) {
        drawTipBlink(x3, y3, _flashColor);
      }
    }
  }

  void drawCubeWire(int cx, int cy, int halfPx, const Quaternion& q, uint16_t color) {
    const Vec3 verts[8] = {
      v3(-1,-1,-1), v3( 1,-1,-1), v3( 1, 1,-1), v3(-1, 1,-1),
      v3(-1,-1, 1), v3( 1,-1, 1), v3( 1, 1, 1), v3(-1, 1, 1)
    };
    static const uint8_t edges[12][2] = {
      {0,1},{1,2},{2,3},{3,0},
      {4,5},{5,6},{6,7},{7,4},
      {0,4},{1,5},{2,6},{3,7}
    };

    int sx[8], sy[8];
    const float dist = 3.5f;
    for (uint8_t i = 0; i < 8; ++i) {
      Vec3 p = rotate(q, verts[i]);
      const float z = p.z + dist;
      const float px = (p.x / z) * halfPx;
      const float py = (p.y / z) * halfPx;
      sx[i] = cx + (int)px;
      sy[i] = cy - (int)py;
    }

    for (uint8_t e = 0; e < 12; ++e) {
      _tft->drawLine(sx[edges[e][0]], sy[edges[e][0]], sx[edges[e][1]], sy[edges[e][1]], color);
    }
  }

  void drawCubeGrid(const FrameInput& in,
                    const Quaternion* qSet,
                    int x,
                    int y,
                    int w,
                    int h,
                    bool flashActive,
                    bool blinkOn,
                    const char* title = nullptr) {
    const int cols = 4;
    const int rows = 2;
    const int titleH = (title && title[0]) ? 9 : 0;
    const int cellW = w / cols;
    const int cellH = (h - titleH) / rows;
    const uint8_t maxCells = 8;

    if (titleH > 0) {
      _tft->setTextFont(1);
      _tft->setTextColor(TFT_CYAN, _bg);
      _tft->setCursor(x + 2, y + 1);
      _tft->print(title);
    }

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        const uint8_t idx = (uint8_t)(r * cols + c);
        const int x0 = x + c * cellW;
        const int y0 = y + titleH + r * cellH;
        _tft->drawRect(x0, y0, cellW, cellH, TFT_DARKGREY);

        const int cx = x0 + cellW / 2;
        const int cy = y0 + cellH / 2 + 1;
        const int half = ((cellH < cellW ? cellH : cellW) * 2) / 3;

        _tft->setTextFont(1);
        _tft->setTextColor(TFT_LIGHTGREY, _bg);
        _tft->setCursor(x0 + 2, y0 + 2);
        _tft->print(sensorLabel(idx));

        if (idx < in.sensor_count && in.present[idx]) {
          const uint16_t col = colorForSensor(in, idx, flashActive);
          drawCubeWire(cx, cy + 3, half, qSet[idx], col);
          if (flashActive && blinkOn && (_flashMask & (1u << idx))) {
            drawTipBlink(cx, cy + 3, _flashColor);
          }
        } else if (idx == maxCells - 1) {
          _tft->drawFastHLine(cx - 6, cy + 3, 12, TFT_DARKGREY);
        }
      }
    }
  }

  void drawLogArea(int x, int y, int w, int h) {
    _tft->fillRect(x, y, w, h, _bg);
    _tft->drawFastHLine(x, y, w, TFT_DARKGREY);
    _tft->setTextColor(TFT_GREEN, _bg);
    _tft->setTextFont(1);

    const int lineH = 8;
    const int maxVisible = h / lineH;
    const int toShow = (_logCount < (uint8_t)maxVisible) ? _logCount : (uint8_t)maxVisible;

    for (int i = 0; i < toShow; ++i) {
      const int idx = (_logWrite + kLogLines - 1 - i) % kLogLines;
      _tft->setCursor(x + 2, y + 2 + i * lineH);
      _tft->print(_log[idx]);
    }
  }
};

} // namespace viz
} // namespace gag
#endif // ARDUINO
