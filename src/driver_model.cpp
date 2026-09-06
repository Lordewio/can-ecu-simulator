#include "driver_model.h"

#include <math.h>

namespace driver {
namespace {

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

struct Phase {
  const char *name;
  float targetKph;
  float durationSec;
  float aggression;  // 0..1, scales how hard the driver works the pedals
  bool keyOn;
  bool starter;
};

// A full trip: cold start, urban work, a highway run, then back into town.
// Index 0..3 run once at power-up; the loop restarts at kTripLoopStart.
constexpr int kTripLoopStart = 4;
const Phase kTrip[] = {
    {"key off",       0.0f,   3.0f, 0.0f, false, false},
    {"key on",        0.0f,   2.5f, 0.0f, true,  false},
    {"cranking",      0.0f,   1.0f, 0.0f, true,  true},
    {"cold idle",     0.0f,  25.0f, 0.0f, true,  false},
    {"pull away",    30.0f,  10.0f, 0.45f, true, false},
    {"city cruise",  50.0f,  25.0f, 0.40f, true, false},
    {"stop light",    0.0f,  14.0f, 0.50f, true, false},
    {"accelerate",   60.0f,  15.0f, 0.60f, true, false},
    {"on-ramp",     100.0f,  22.0f, 0.95f, true, false},
    {"highway",     110.0f,  55.0f, 0.35f, true, false},
    {"overtake",    128.0f,  18.0f, 0.85f, true, false},
    {"highway",     104.0f,  45.0f, 0.30f, true, false},
    {"off-ramp",     55.0f,  16.0f, 0.55f, true, false},
    {"urban",        40.0f,  26.0f, 0.45f, true, false},
    {"stop",          0.0f,  14.0f, 0.55f, true, false},
};
constexpr int kTripCount = static_cast<int>(sizeof(kTrip) / sizeof(kTrip[0]));

const Phase kCity[] = {
    {"pull away",  32.0f, 11.0f, 0.50f, true, false},
    {"cruise",     48.0f, 18.0f, 0.40f, true, false},
    {"slow",       22.0f,  9.0f, 0.45f, true, false},
    {"cruise",     55.0f, 16.0f, 0.55f, true, false},
    {"stop light",  0.0f, 13.0f, 0.55f, true, false},
};
constexpr int kCityCount = static_cast<int>(sizeof(kCity) / sizeof(kCity[0]));

const Phase kHighway[] = {
    {"merge",     100.0f, 20.0f, 0.90f, true, false},
    {"cruise",    112.0f, 70.0f, 0.30f, true, false},
    {"overtake",  126.0f, 20.0f, 0.80f, true, false},
    {"cruise",    108.0f, 60.0f, 0.30f, true, false},
};
constexpr int kHighwayCount = static_cast<int>(sizeof(kHighway) / sizeof(kHighway[0]));

const Phase kIdlePhase = {"idle", 0.0f, 3600.0f, 0.0f, true, false};
const Phase kKeyOffPhase = {"key off", 0.0f, 3600.0f, 0.0f, false, false};

const Phase &phaseAt(Scenario s, int index) {
  switch (s) {
    case Scenario::CITY:
      return kCity[index % kCityCount];
    case Scenario::HIGHWAY:
      return kHighway[index % kHighwayCount];
    case Scenario::IDLE:
      return kIdlePhase;
    case Scenario::KEY_OFF:
      return kKeyOffPhase;
    case Scenario::MANUAL:
      return kIdlePhase;
    case Scenario::TRIP:
    default:
      return kTrip[index % kTripCount];
  }
}

}  // namespace

void Driver::begin(Scenario scenario) {
  scenario_ = scenario;
  phaseIndex_ = 0;
  phaseTimer_ = 0.0f;
  integrator_ = 0.0f;
  pedal_ = 0.0f;
  brake_ = 0.0f;
  const Phase &p = phaseAt(scenario_, phaseIndex_);
  targetKph_ = p.targetKph;
  aggression_ = p.aggression;
  phaseName_ = p.name;
  keyOn_ = p.keyOn;
  starter_ = p.starter;
}

void Driver::setScenario(Scenario scenario) {
  if (scenario == scenario_) return;
  scenario_ = scenario;
  phaseIndex_ = 0;
  phaseTimer_ = 0.0f;
  integrator_ = 0.0f;
  const Phase &p = phaseAt(scenario_, phaseIndex_);
  targetKph_ = p.targetKph;
  aggression_ = p.aggression;
  phaseName_ = p.name;
  keyOn_ = p.keyOn;
  starter_ = p.starter;
}

void Driver::setManualTarget(float kph) {
  manualTargetKph_ = clampf(kph, 0.0f, 180.0f);
  if (scenario_ == Scenario::MANUAL) targetKph_ = manualTargetKph_;
}

void Driver::advancePhase(const vehicle::State &st) {
  phaseTimer_ = 0.0f;
  integrator_ *= 0.5f;

  if (scenario_ == Scenario::TRIP) {
    phaseIndex_++;
    if (phaseIndex_ >= kTripCount) phaseIndex_ = kTripLoopStart;
  } else {
    phaseIndex_++;
  }

  const Phase &p = phaseAt(scenario_, phaseIndex_);
  targetKph_ = p.targetKph;
  aggression_ = p.aggression;
  phaseName_ = p.name;
  keyOn_ = p.keyOn;
  starter_ = p.starter;
  (void)st;
}

vehicle::Input Driver::update(const vehicle::State &st, float dt) {
  const float step = clampf(dt, 0.0f, 0.10f);
  noisePhase_ += step;
  phaseTimer_ += step;

  const Phase &phase = phaseAt(scenario_, phaseIndex_);

  if (scenario_ == Scenario::MANUAL) {
    targetKph_ = manualTargetKph_;
    aggression_ = 0.6f;
    phaseName_ = "manual";
    keyOn_ = true;
    starter_ = !st.engineRunning && st.speedKph < 1.0f;
  } else if (scenario_ == Scenario::KEY_OFF) {
    keyOn_ = false;
    starter_ = false;
    targetKph_ = 0.0f;
  } else {
    keyOn_ = phase.keyOn;
    // Hold the starter until the engine catches. A driver who finds the
    // engine stopped mid-trip restarts it rather than coasting to a halt.
    const bool wantsStart = phase.starter || (st.stalled && phase.keyOn && st.speedKph < 2.0f);
    starter_ = wantsStart && !st.engineRunning;
    targetKph_ = phase.targetKph;
    aggression_ = phase.aggression;
    phaseName_ = phase.name;

    const bool crankDone = phase.starter && st.engineRunning;
    if (phaseTimer_ >= phase.durationSec || crankDone) {
      advancePhase(st);
    }
  }

  vehicle::Input in;
  in.keyOn = keyOn_;
  in.starterEngaged = starter_;

  if (!keyOn_ || !st.engineRunning) {
    // No point working the pedals; just hold the car still if it is moving.
    pedal_ = 0.0f;
    brake_ = (st.speedKph > 0.5f) ? 25.0f : 0.0f;
    integrator_ = 0.0f;
    in.pedalPercent = pedal_;
    in.brakePercent = brake_;
    in.steeringAngleDeg = 0.0f;
    return in;
  }

  // Real drivers do not hold an exact speed; they wander a little.
  const float wander = 1.2f * sinf(noisePhase_ * 0.23f) + 0.6f * sinf(noisePhase_ * 0.71f);
  const float target = clampf(targetKph_ + (targetKph_ > 5.0f ? wander : 0.0f), 0.0f, 180.0f);

  const float error = target - st.speedKph;
  const float gain = 1.4f + 4.6f * aggression_;

  // Anti-windup: stop integrating once the pedal is against a stop.
  const bool saturated = (pedal_ >= 99.5f && error > 0.0f) || (brake_ >= 99.5f && error < 0.0f);
  if (!saturated) {
    integrator_ = clampf(integrator_ + error * step * 0.55f, -60.0f, 90.0f);
  }

  float demand = error * gain + integrator_;

  // Coming to a planned stop, the driver holds the brake rather than
  // hunting around zero.
  if (target < 1.0f) {
    demand = (st.speedKph > 0.4f) ? -22.0f - st.speedKph * 0.9f : -18.0f;
    integrator_ = 0.0f;
  }

  float pedalTarget = 0.0f;
  float brakeTarget = 0.0f;
  if (demand >= 0.0f) {
    pedalTarget = clampf(demand, 0.0f, 100.0f);
  } else {
    // Light overspeed is handled by lifting off, not by braking.
    brakeTarget = clampf((-demand - 6.0f) * 1.4f, 0.0f, 100.0f);
  }

  // A foot has a finite speed. Rate-limiting here is what produces realistic
  // throttle traces instead of instantaneous steps.
  const float pedalRate = 90.0f + 320.0f * aggression_;
  const float brakeRate = 130.0f + 300.0f * aggression_;
  pedal_ += clampf(pedalTarget - pedal_, -pedalRate * step, pedalRate * step);
  brake_ += clampf(brakeTarget - brake_, -brakeRate * step, brakeRate * step);
  pedal_ = clampf(pedal_, 0.0f, 100.0f);
  brake_ = clampf(brake_, 0.0f, 100.0f);

  // Steering: small corrections on the straight, bigger inputs in town.
  const float lowSpeedGain = clampf(1.0f - st.speedKph / 70.0f, 0.15f, 1.0f);
  const float steerTarget = (18.0f * sinf(noisePhase_ * 0.13f) + 8.0f * sinf(noisePhase_ * 0.37f)) *
                            lowSpeedGain;
  steering_ += (steerTarget - steering_) * clampf(step * 1.5f, 0.0f, 1.0f);

  in.pedalPercent = pedal_;
  in.brakePercent = brake_;
  in.steeringAngleDeg = steering_;
  // Gentle rolling terrain; enough to make load vary on a constant-speed cruise.
  in.roadGradePercent = 1.6f * sinf(noisePhase_ * 0.018f);
  return in;
}

}  // namespace driver
