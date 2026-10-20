#pragma once
#include <stdint.h>
#include <math.h>

// No Arduino dependencies: these functions are also exercised by host tests.
struct ColourModel { uint16_t mean[3]; uint16_t spread[3]; };
inline int classifyColour(const uint16_t rgb[3], const ColourModel models[6],
                          uint8_t mask, float &best, float &second) {
  best = second = 1.0e9f;
  int winner = -1;
  if (mask != 63) return -1; // include white; never invent missing classes
  for (int c = 0; c < 6; ++c) {
    float score = 0;
    for (int k = 0; k < 3; ++k) {
      float scale = models[c].spread[k] < 12 ? 12 : models[c].spread[k];
      float d = (float(rgb[k]) - models[c].mean[k]) / scale;
      score += d * d;
    }
    if (score < best) { second = best; best = score; winner = c; }
    else if (score < second) second = score;
  }
  // Trial rejection limits: reject distant or ambiguous readings.
  return (best <= 27.0f && second > 0.01f && best < 0.60f * second) ? winner : -1;
}
inline float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
// Nonnegative pressure means "steer away from this side". Missing walls
// exert no pull, so openings never command a maze turn.
inline int correctionPWM(float leftPressure, float rightPressure, int limit) {
  return int((clamp01(leftPressure) - clamp01(rightPressure)) * limit);
}
inline bool irCalibrationOrdered(uint16_t nearValue, uint16_t clearValue,
                                  uint16_t noneValue) {
  return nearValue > clearValue + 10 && clearValue > noneValue + 5;
}
