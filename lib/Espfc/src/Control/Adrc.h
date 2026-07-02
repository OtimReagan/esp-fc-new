#pragma once

#include <cstdint>
#include "Utils/Filter.h"
#include "Utils/Math.hpp"

namespace Espfc {

namespace Control {

class Adrc
{
public:
  Adrc();
  void begin();
  float update(float setpoint, float measure);
  void resetIterm();

  float rate;
  float dt;

  // repurposed gain slots — same names Controller.cpp already writes to
  float Kp;   // -> wc: controller bandwidth (rad/s)
  float Ki;   // -> wo: observer bandwidth (rad/s), typically 3-10x wc
  float Kd;   // -> b0: control gain estimate (plant-specific)
  float Kf;   // unused for now, kept for interface parity

  float iLimitLow;
  float iLimitHigh;
  float iReset;
  float oLimitLow;
  float oLimitHigh;

  float pScale;
  float iScale;
  float dScale;
  float fScale;

  // debug/blackbox mirrors — same fields Controller.cpp reads
  float error;       // observer error (measure - z1)
  float iTermError;

  float pTerm;        // = wc * (setpoint - z1), pre-b0-division
  float iTerm;         // mirrors z2 (disturbance estimate) — externally writable, see note below
  float dTerm;        // unused, kept 0
  float fTerm;         // unused, kept 0

  Utils::Filter dtermFilter;
  Utils::Filter dtermFilter2;
  Utils::Filter dtermNotchFilter;
  Utils::Filter ptermFilter;
  Utils::Filter ftermFilter;
  Utils::Filter itermRelaxFilter;

  float prevMeasurement;
  float prevError;
  float prevSetpoint;

  bool ftermDerivative;
  bool outputSaturated;
  int8_t itermRelax;
  float itermRelaxFactor;
  float itermRelaxBase;

private:
  // ESO state (1st-order plant model: y_dot = b0*u + f)
  float _z1;  // estimate of measurement (y)
  float _z2;  // estimate of total disturbance (f), == iTerm mirror
  float _uPrev;
};

}

}
