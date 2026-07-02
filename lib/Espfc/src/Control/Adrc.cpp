#include "Control/Adrc.h"
#include <algorithm>

namespace Espfc::Control {

Adrc::Adrc():
  rate(1000), dt(0.001f),
  Kp(0), Ki(0), Kd(1.f), Kf(0),
  iLimitLow(-1.f), iLimitHigh(1.f), iReset(0.f),
  oLimitLow(-1.f), oLimitHigh(1.f),
  pScale(1.f), iScale(1.f), dScale(1.f), fScale(1.f),
  error(0), iTermError(0), pTerm(0), iTerm(0), dTerm(0), fTerm(0),
  prevMeasurement(0), prevError(0), prevSetpoint(0),
  ftermDerivative(false), outputSaturated(false),
  itermRelax(0), itermRelaxFactor(0), itermRelaxBase(0),
  _z1(0), _z2(0), _uPrev(0)
{}

void Adrc::begin()
{
  dt = 1.0f / rate;
  _z1 = 0.f;
  _z2 = 0.f;
  _uPrev = 0.f;
}

float Adrc::update(float setpoint, float measure)
{
  const float wc = Kp;             // controller bandwidth
  const float wo = Ki;             // observer bandwidth
  const float b0 = (Kd != 0.f) ? Kd : 1.f; // guard divide-by-zero

  // critically-damped ESO gains for 1st-order plant (2-state observer)
  const float beta1 = 2.f * wo;
  const float beta2 = wo * wo;

  // --- ESO update (predict-correct, using previous control output) ---
  const float obsError = measure - _z1;
  _z1 += dt * (_z2 + b0 * _uPrev + beta1 * obsError);
  _z2 += dt * (beta2 * obsError);

  // clamp disturbance estimate to configured iTerm-equivalent limits
  _z2 = Utils::clamp(_z2, iLimitLow, iLimitHigh);

  // --- control law ---
  const float u0 = wc * (setpoint - _z1);
  float u = (u0 - _z2) / b0;

  // output clamp, same convention as Pid
  const float uClamped = Utils::clamp(u, oLimitLow, oLimitHigh);
  outputSaturated = (uClamped != u);
  u = uClamped;

  _uPrev = u;

  // debug/blackbox mirrors
  error = obsError;
  pTerm = u0;
  iTerm = _z2;     // externally-writable disturbance estimate, mirrors Pid's iTerm semantics
  dTerm = 0.f;
  fTerm = 0.f;
  prevMeasurement = measure;
  prevError = error;
  prevSetpoint = setpoint;

  return u;
}

void Adrc::resetIterm()
{
  _z2 = iReset;
  iTerm = _z2;
}

}
