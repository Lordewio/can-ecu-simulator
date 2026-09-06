#include "vehicle_model.h"

#include <math.h>

#include "vehicle_params.h"

namespace vehicle {
namespace {

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Linear interpolation over a monotonically increasing breakpoint table.
float interpolate(const float *x, const float *y, int n, float xi) {
  if (xi <= x[0]) return y[0];
  if (xi >= x[n - 1]) return y[n - 1];
  for (int i = 1; i < n; ++i) {
    if (xi <= x[i]) {
      const float span = x[i] - x[i - 1];
      const float f = (span > 0.0f) ? (xi - x[i - 1]) / span : 0.0f;
      return y[i - 1] + f * (y[i] - y[i - 1]);
    }
  }
  return y[n - 1];
}

// 2ZR-FE wide-open-throttle torque curve. 173 Nm @ 4400 rpm, 98 kW @ 6000 rpm.
constexpr int kTorqueBreakpoints = 13;
const float kTorqueRpm[kTorqueBreakpoints] = {700.0f,  1000.0f, 1500.0f, 2000.0f, 2500.0f,
                                              3000.0f, 3500.0f, 4000.0f, 4400.0f, 5000.0f,
                                              5500.0f, 6000.0f, 6400.0f};
const float kTorqueNm[kTorqueBreakpoints] = {105.0f, 128.0f, 148.0f, 158.0f, 165.0f,
                                             169.0f, 171.0f, 172.0f, 173.0f, 170.0f,
                                             164.0f, 157.0f, 145.0f};

// A throttle plate has most of its flow authority in the first third of its
// travel, so plate angle maps to cylinder charge through a strong curve. This
// is what makes a real car cruise at ~20% TPS instead of ~45%.
float plateToLoad(float throttleFraction) {
  const float t = clampf(throttleFraction, 0.0f, 1.0f);
  return 1.0f - powf(1.0f - t, 2.2f);
}

// Cheap deterministic noise so the bus does not look mathematically perfect.
float wobble(float phase, float a, float b) {
  return sinf(phase * a) * 0.6f + sinf(phase * b) * 0.4f;
}

}  // namespace

float wotTorqueNm(float rpm) {
  return interpolate(kTorqueRpm, kTorqueNm, kTorqueBreakpoints, rpm);
}

float frictionTorqueNm(float rpm) {
  // Rubbing friction plus pumping work; roughly 8 Nm at idle, 27 Nm at redline.
  return 6.0f + 0.0035f * rpm;
}

float rpmForSpeed(float speedKph, uint8_t gear) {
  if (gear < 1 || gear > kGearCount) return 0.0f;
  const float wheelRevPerSec = (speedKph / 3.6f) / kWheelCircumferenceM;
  return wheelRevPerSec * 60.0f * kFinalDriveRatio * kGearRatio[gear];
}

void Model::begin(float ambientC, double odometerKm, float fuelLiters) {
  state_ = State();
  state_.ambientTempC = ambientC;
  state_.coolantTempC = ambientC;
  state_.oilTempC = ambientC;
  state_.intakeTempC = ambientC;
  state_.odometerKm = odometerKm;
  state_.fuelRemainingLiters = clampf(fuelLiters, 0.0f, kFuelTankLiters);
  state_.fuelLevelPercent = (state_.fuelRemainingLiters / kFuelTankLiters) * 100.0f;
  state_.batteryVolts = kBatteryRestingVolts;
  state_.barometricKpa = 101.0f;
  state_.ignition = Ignition::OFF;
}

void Model::clearAdaptations() {
  trimIntegrator_ = 0.0f;
  state_.longTrimBank1 = 0.0f;
  state_.distanceSinceClearKm = 0.0;
  state_.warmupsSinceClear = 0;
}

void Model::step(const Input &in, float dtSeconds) {
  // A stalled scheduler must not blow up the integrator.
  const float dt = clampf(dtSeconds, 0.0f, 0.10f);
  if (dt <= 0.0f) return;

  noisePhase_ += dt;

  updateIgnition(in, dt);
  updateEngine(in, dt);
  updateTransmission(dt);
  updateChassis(in, dt);
  updateAirFuel(dt);
  updateThermal(dt);
  updateElectrical(dt);
  updateAccumulators(dt);

  state_.steeringAngleDeg = in.steeringAngleDeg;
  state_.roadGradePercent = in.roadGradePercent;
}

void Model::updateIgnition(const Input &in, float dt) {
  if (!in.keyOn) {
    state_.ignition = (state_.speedKph > 1.0f) ? Ignition::ACCESSORY : Ignition::OFF;
    state_.engineRunning = false;
    crankTimer_ = 0.0f;
    return;
  }

  if (startGrace_ > 0.0f) startGrace_ -= dt;

  if (state_.engineRunning) {
    state_.ignition = Ignition::RUNNING;
    // Below the stall threshold with no starter the engine dies. Suppressed
    // briefly after a start, while the engine is still coming up to idle.
    if (state_.engineRpm < kStallRpm && !in.starterEngaged && startGrace_ <= 0.0f) {
      state_.engineRunning = false;
      state_.stalled = true;
      state_.ignition = Ignition::ON;
    }
    return;
  }

  if (in.starterEngaged) {
    state_.ignition = Ignition::CRANK;
    crankTimer_ += dt;
    // The starter drags the crank up to cranking speed; the engine catches
    // once it has spun long enough to build oil pressure and fuel rail.
    state_.engineRpm += (kCrankRpm - state_.engineRpm) * clampf(dt * 6.0f, 0.0f, 1.0f);
    if (crankTimer_ > 0.6f) {
      state_.engineRunning = true;
      state_.stalled = false;
      state_.ignition = Ignition::RUNNING;
      crankTimer_ = 0.0f;
      warmupCounted_ = false;
      // An engine that catches flares off the starter before the idle
      // controller pulls it down to the target.
      if (state_.engineRpm < 500.0f) state_.engineRpm = 500.0f;
      startGrace_ = 1.5f;
    }
    return;
  }

  state_.ignition = Ignition::ON;
  crankTimer_ = 0.0f;
}

void Model::updateEngine(const Input &in, float dt) {
  state_.pedalPercent = clampf(in.pedalPercent, 0.0f, 100.0f);
  state_.brakePercent = clampf(in.brakePercent, 0.0f, 100.0f);

  if (!state_.engineRunning) {
    if (state_.ignition != Ignition::CRANK) {
      // Spin down against friction.
      state_.engineRpm = clampf(state_.engineRpm - 900.0f * dt, 0.0f, kRedlineRpm);
    }
    state_.throttlePercent = state_.pedalPercent * 0.9f;
    state_.engineLoad = 0.0f;
    state_.engineTorqueNm = 0.0f;
    state_.fuelCut = false;
    idleIntegrator_ = 0.0f;
    return;
  }

  // Idle air control: a PI loop that holds the target idle speed whenever the
  // driver is off the pedal. Cold engines idle high until the coolant warms.
  const float warmFraction = clampf((state_.coolantTempC - 20.0f) / 60.0f, 0.0f, 1.0f);
  const float targetIdle = kColdIdleRpm + (kIdleRpm - kColdIdleRpm) * warmFraction;

  float commandedPlate = state_.pedalPercent;
  if (state_.pedalPercent < 2.0f && state_.engineRpm < targetIdle + 500.0f) {
    const float error = targetIdle - state_.engineRpm;
    idleIntegrator_ = clampf(idleIntegrator_ + error * dt * 0.010f, 0.0f, 22.0f);
    commandedPlate = clampf(idleIntegrator_ + error * 0.006f, 0.0f, 30.0f);
  } else {
    // Bleed the integrator back toward its idle authority while driving.
    idleIntegrator_ += (10.0f - idleIntegrator_) * clampf(dt * 0.5f, 0.0f, 1.0f);
  }

  // Throttle plate has mass; it cannot step instantly.
  const float plateRate = 400.0f;  // percent per second
  const float delta = clampf(commandedPlate - state_.throttlePercent, -plateRate * dt, plateRate * dt);
  state_.throttlePercent = clampf(state_.throttlePercent + delta, 0.0f, 100.0f);

  // Deceleration fuel cut-off: closed throttle, above idle, in gear.
  const bool dfco = state_.pedalPercent < 1.0f && state_.engineRpm > targetIdle + 400.0f &&
                    state_.gear > 0 && state_.speedKph > 15.0f;
  state_.fuelCut = dfco || state_.engineRpm > kFuelCutRpm;

  state_.engineLoad = plateToLoad(state_.throttlePercent / 100.0f);

  float indicated = wotTorqueNm(state_.engineRpm) * state_.engineLoad;
  if (state_.fuelCut) indicated = 0.0f;

  float net = indicated - frictionTorqueNm(state_.engineRpm);

  // Torque reduction during a shift, as the TCM requests from the ECM.
  if (state_.shifting) {
    const float half = kShiftDurationSec * (1.0f - 0.45f * state_.engineLoad) * 0.5f;
    const float progress = (half > 0.0f) ? 1.0f - fabsf(shiftTimer_ - half) / half : 0.0f;
    net *= 1.0f - (1.0f - kShiftTorqueCut) * clampf(progress, 0.0f, 1.0f);
  }

  state_.engineTorqueNm = net;
}

void Model::updateTransmission(float dt) {
  if (gearHoldTimer_ > 0.0f) gearHoldTimer_ -= dt;

  // Park/neutral when stopped with no engine.
  if (!state_.engineRunning && state_.speedKph < 0.5f) {
    state_.gear = 0;
    state_.shifting = false;
    state_.converterLocked = false;
    return;
  }
  if (state_.gear == 0) {
    state_.gear = 1;
    gearHoldTimer_ = 0.5f;
  }

  // Complete an in-progress shift. Shift time shortens as load rises.
  const float shiftDuration = kShiftDurationSec * (1.0f - 0.45f * state_.engineLoad);
  if (state_.shifting) {
    shiftTimer_ += dt;
    if (shiftTimer_ >= shiftDuration * 0.5f && state_.gear != shiftToGear_) {
      state_.gear = shiftToGear_;
    }
    if (shiftTimer_ >= shiftDuration) {
      state_.shifting = false;
      shiftTimer_ = 0.0f;
      gearHoldTimer_ = 1.2f;
    }
  } else if (gearHoldTimer_ <= 0.0f) {
    // Shift schedule: both thresholds rise with load, which is why a car
    // upshifts at 2000 rpm cruising and holds to 5800 rpm at full throttle.
    const float upshiftRpm = 2000.0f + 3800.0f * state_.engineLoad;
    const float downshiftRpm = 1150.0f + 1900.0f * state_.engineLoad;

    if (state_.gear < kGearCount && state_.engineRpm > upshiftRpm && state_.speedKph > 8.0f) {
      shiftFromGear_ = state_.gear;
      shiftToGear_ = static_cast<uint8_t>(state_.gear + 1);
      state_.shifting = true;
      shiftTimer_ = 0.0f;
    } else if (state_.gear > 1 && state_.engineRpm < downshiftRpm) {
      // Never downshift into an overspeed.
      const float projected = rpmForSpeed(state_.speedKph, static_cast<uint8_t>(state_.gear - 1));
      if (projected < kRedlineRpm) {
        shiftFromGear_ = state_.gear;
        shiftToGear_ = static_cast<uint8_t>(state_.gear - 1);
        state_.shifting = true;
        shiftTimer_ = 0.0f;
      }
    }
  }

  // Torque converter lockup clutch, with hysteresis on road speed.
  const bool lockupAllowed = state_.gear >= 3 && !state_.shifting && state_.engineRunning &&
                             state_.engineLoad < kLockupMaxLoad && state_.brakePercent < 5.0f;
  if (state_.converterLocked) {
    state_.converterLocked = lockupAllowed && state_.speedKph > kLockupReleaseKph;
  } else {
    state_.converterLocked = lockupAllowed && state_.speedKph > kLockupEngageKph;
  }
}

void Model::updateChassis(const Input &in, float dt) {
  // Turbine speed is rigidly tied to road speed through the gearbox.
  const float turbineRpm = rpmForSpeed(state_.speedKph, state_.gear);
  state_.turbineRpm = turbineRpm;

  float turbineTorque = 0.0f;

  if (state_.gear == 0 || !state_.engineRunning) {
    // Coasting out of gear, or engine off: no drive torque.
    if (state_.engineRunning) {
      const float error = kIdleRpm - state_.engineRpm;
      state_.engineRpm += error * clampf(dt * 3.0f, 0.0f, 1.0f);
    }
  } else if (state_.converterLocked) {
    // Locked: crank and turbine are one shaft. Solve them together so engine
    // braking and drive torque both pass straight through.
    state_.engineRpm = turbineRpm;
    turbineTorque = state_.engineTorqueNm;
  } else {
    // Fluid coupling. Torque transfer is quadratic in the speed difference and
    // is signed, so an overrunning turbine drives the engine (engine braking).
    const float ni = state_.engineRpm;
    const float nt = turbineRpm;
    float impellerTorque = kConverterCapacity * (ni * ni - nt * nt);

    // Torque multiplication only exists below the coupling point.
    const float sr = (ni > 1.0f) ? clampf(nt / ni, 0.0f, 1.0f) : 0.0f;
    const float ratio = 1.0f + (kConverterStallTorqueRatio - 1.0f) * (1.0f - sr) * (1.0f - sr);
    turbineTorque = impellerTorque * ((impellerTorque > 0.0f) ? ratio : 1.0f);

    // Integrate the crankshaft against the load the impeller imposes.
    const float alpha = (state_.engineTorqueNm - impellerTorque) / kEngineInertia;  // rad/s^2
    const float dRpm = alpha * dt * (60.0f / (2.0f * 3.14159265f));
    state_.engineRpm = clampf(state_.engineRpm + dRpm, 0.0f, kFuelCutRpm + 200.0f);
  }

  state_.wheelTorqueNm = turbineTorque * kGearRatio[state_.gear > 0 ? state_.gear : 1] *
                         kFinalDriveRatio * kDrivelineEfficiency;

  const float v = state_.speedMps;
  const float tractive = state_.wheelTorqueNm / kWheelRadiusM;
  const float aero = 0.5f * kAirDensityKgM3 * kDragCoefficient * kFrontalAreaM2 * v * v;
  const float rolling = (state_.speedKph > 0.3f) ? kRollingResistance * kMassKg * kGravityMps2 : 0.0f;
  const float grade = kMassKg * kGravityMps2 * (in.roadGradePercent / 100.0f);
  const float braking = (state_.brakePercent / 100.0f) * kMaxBrakeDecelMps2 * kMassKg;

  float netForce = tractive - aero - rolling - grade;
  if (v > 0.05f) {
    netForce -= braking;
  } else if (braking > fabsf(netForce)) {
    netForce = 0.0f;  // held on the brakes
  }

  const float effectiveMass = kMassKg * kRotatingMassFactor;
  state_.accelMps2 = netForce / effectiveMass;

  state_.speedMps = state_.speedMps + state_.accelMps2 * dt;
  if (state_.speedMps < 0.0f) state_.speedMps = 0.0f;  // no reverse in this model
  state_.speedKph = state_.speedMps * 3.6f;

  // Individual wheel speeds: driven wheels carry a little slip under torque,
  // and the outside wheels of a turn travel further than the inside ones.
  const float slip = clampf(state_.wheelTorqueNm / 4000.0f, -0.02f, 0.02f);
  const float turn = clampf(state_.steeringAngleDeg / 600.0f, -0.05f, 0.05f);
  state_.wheelSpeedKph[0] = state_.speedKph * (1.0f + slip - turn);  // FL, driven
  state_.wheelSpeedKph[1] = state_.speedKph * (1.0f + slip + turn);  // FR, driven
  state_.wheelSpeedKph[2] = state_.speedKph * (1.0f - turn);         // RL
  state_.wheelSpeedKph[3] = state_.speedKph * (1.0f + turn);         // RR
  for (int i = 0; i < 4; ++i) {
    if (state_.wheelSpeedKph[i] < 0.0f) state_.wheelSpeedKph[i] = 0.0f;
  }
}

void Model::updateAirFuel(float dt) {
  // Barometric pressure drifts slowly, as weather does.
  state_.barometricKpa = 101.0f + 1.4f * sinf(noisePhase_ * 0.004f);

  if (!state_.engineRunning) {
    state_.mapKpa = state_.barometricKpa;
    state_.mafGramsSec = 0.0f;
    state_.fuelPressureKpa = state_.engineRunning ? 324.0f : 0.0f;
    state_.closedLoop = false;
    state_.lambda = 1.0f;
    state_.shortTrimBank1 = 0.0f;
    state_.o2DownstreamVolts = 0.10f;
    return;
  }

  // Manifold pressure follows cylinder charge; near-vacuum at closed throttle,
  // barometric at wide-open throttle.
  const float targetMap = state_.barometricKpa * (0.12f + 0.88f * state_.engineLoad);
  state_.mapKpa += (targetMap - state_.mapKpa) * clampf(dt * 12.0f, 0.0f, 1.0f);

  // Speed-density airflow: m_dot = (MAP * Vd * N * VE) / (R * T)
  const float intakeKelvin = state_.intakeTempC + 273.15f;
  const float cyclesPerSec = state_.engineRpm / 120.0f;  // 4-stroke
  const float massFlowKgSec = (state_.mapKpa * 1000.0f * kEngineDisplacementM3 * cyclesPerSec *
                               kVolumetricEfficiency) /
                              (287.0f * intakeKelvin);
  state_.mafGramsSec = massFlowKgSec * 1000.0f;

  // Port injection on the 2ZR-FE: rail pressure is regulated, not variable.
  state_.fuelPressureKpa = 324.0f + 6.0f * sinf(noisePhase_ * 1.7f);

  // Closed loop needs a hot engine, a light-to-moderate load and no fuel cut.
  state_.closedLoop = state_.coolantTempC > 60.0f && state_.engineLoad < 0.80f && !state_.fuelCut;

  if (state_.closedLoop) {
    // The fuel controller hunts across stoichiometric a couple of times a
    // second; that oscillation is what the downstream sensor smooths out.
    lambdaPhase_ += dt * 7.5f;
    state_.lambda = 1.0f + 0.025f * sinf(lambdaPhase_);

    // Short-term trim chases the sensor; long-term slowly absorbs the mean.
    const float target = 2.5f * sinf(lambdaPhase_ * 0.9f) + 1.5f * sinf(noisePhase_ * 0.21f);
    state_.shortTrimBank1 += (target - state_.shortTrimBank1) * clampf(dt * 6.0f, 0.0f, 1.0f);
    trimIntegrator_ = clampf(trimIntegrator_ + state_.shortTrimBank1 * dt * 0.02f, -12.0f, 12.0f);
    state_.longTrimBank1 = trimIntegrator_;
  } else if (state_.fuelCut) {
    state_.lambda = 3.0f;  // pure air
    state_.shortTrimBank1 = 0.0f;
  } else {
    // Open loop: cold enrichment, or power enrichment at high load.
    state_.lambda = (state_.coolantTempC < 60.0f) ? 0.90f : 0.85f;
    state_.shortTrimBank1 = 0.0f;
  }

  // Downstream narrowband sensor behind a healthy catalyst: lazy, and parked
  // high because the cat has stripped the oxygen out of the exhaust.
  o2Phase_ += dt;
  float o2Target;
  if (state_.fuelCut) {
    o2Target = 0.08f;
  } else if (state_.lambda < 0.95f) {
    o2Target = 0.82f;
  } else {
    o2Target = 0.68f + 0.06f * sinf(o2Phase_ * 0.35f);
  }
  state_.o2DownstreamVolts += (o2Target - state_.o2DownstreamVolts) * clampf(dt * 1.6f, 0.0f, 1.0f);

  // EGR opens at part load on a warm engine, closed at idle and at full load.
  const bool egrActive = state_.coolantTempC > 70.0f && state_.engineLoad > 0.25f &&
                         state_.engineLoad < 0.75f && state_.engineRpm > 1400.0f;
  const float egrTarget = egrActive ? 18.0f : 0.0f;
  state_.commandedEgrPercent += (egrTarget - state_.commandedEgrPercent) * clampf(dt * 2.0f, 0.0f, 1.0f);
}

void Model::updateThermal(float dt) {
  // Ambient drifts over the day.
  const float ambientTarget = 26.0f + 3.0f * sinf(noisePhase_ * 0.0012f);
  state_.ambientTempC += (ambientTarget - state_.ambientTempC) * clampf(dt * 0.01f, 0.0f, 1.0f);

  // Intake air sits above ambient from underhood heat soak, and the effect
  // shrinks as airflow through the bay increases with road speed.
  const float soak = clampf((state_.coolantTempC - state_.ambientTempC) * 0.30f, 0.0f, 25.0f);
  const float ram = clampf(1.0f - state_.speedKph / 90.0f, 0.25f, 1.0f);
  const float intakeTarget = state_.ambientTempC + soak * ram;
  state_.intakeTempC += (intakeTarget - state_.intakeTempC) * clampf(dt * 0.10f, 0.0f, 1.0f);

  if (!state_.engineRunning) {
    // Soak back toward ambient with the engine off.
    state_.coolantTempC += (state_.ambientTempC - state_.coolantTempC) * clampf(dt * 0.0015f, 0.0f, 1.0f);
    state_.oilTempC += (state_.ambientTempC - state_.oilTempC) * clampf(dt * 0.0010f, 0.0f, 1.0f);
    state_.oilPressureKpa = 0.0f;
    state_.coolingFanOn = false;
    return;
  }

  // Heat into the coolant scales with fuel burned; heat out depends on whether
  // the thermostat has opened and how much air is going through the radiator.
  const float heatIn = (0.030f + 0.55f * state_.engineLoad) * (state_.engineRpm / 2000.0f);

  float coolingCoefficient = 0.004f;  // block losses only, thermostat shut
  if (state_.coolantTempC > kThermostatOpenC) {
    const float open = clampf((state_.coolantTempC - kThermostatOpenC) / 7.0f, 0.0f, 1.0f);
    const float airflow = 0.35f + state_.speedKph / 100.0f + (state_.coolingFanOn ? 0.9f : 0.0f);
    coolingCoefficient += 0.055f * open * airflow;
  }
  const float heatOut = coolingCoefficient * (state_.coolantTempC - state_.ambientTempC);
  state_.coolantTempC = clampf(state_.coolantTempC + (heatIn - heatOut) * dt, -40.0f, 130.0f);

  // Fan cycles on a hysteresis band, as the real one does.
  if (state_.coolingFanOn) {
    if (state_.coolantTempC < kCoolingFanOffC) state_.coolingFanOn = false;
  } else {
    if (state_.coolantTempC > kCoolingFanOnC) state_.coolingFanOn = true;
  }

  // Oil lags coolant and runs hotter under sustained load.
  const float oilTarget = state_.coolantTempC + 6.0f + 14.0f * state_.engineLoad;
  state_.oilTempC += (oilTarget - state_.oilTempC) * clampf(dt * 0.02f, 0.0f, 1.0f);

  // Pressure rises with pump speed and falls as hot oil thins.
  const float viscosity = clampf(1.55f - state_.oilTempC / 150.0f, 0.75f, 1.55f);
  state_.oilPressureKpa = clampf((90.0f + state_.engineRpm * 0.085f) * viscosity, 80.0f, 620.0f);
}

void Model::updateElectrical(float dt) {
  float target;
  if (state_.ignition == Ignition::CRANK) {
    target = kCrankingVolts;  // starter drags the battery down hard
    state_.alternatorCharging = false;
  } else if (state_.engineRunning) {
    // Charging voltage backs off as the battery fills and as the alternator
    // warms; electrical load from the fan pulls it down a little.
    target = kChargingVolts - (state_.coolingFanOn ? 0.25f : 0.0f) +
             0.08f * wobble(noisePhase_, 0.7f, 2.3f);
    state_.alternatorCharging = true;
  } else if (state_.ignition == Ignition::OFF) {
    target = kBatteryRestingVolts;
    state_.alternatorCharging = false;
  } else {
    target = kBatteryRestingVolts - 0.35f;  // key on, loads live, no charge
    state_.alternatorCharging = false;
  }
  state_.batteryVolts += (target - state_.batteryVolts) * clampf(dt * 4.0f, 0.0f, 1.0f);
}

void Model::updateAccumulators(float dt) {
  // Distance is accumulated in metres in a float and only handed to the
  // double odometer a metre at a time, so neither accumulator loses steps.
  distanceAccumulatorM_ += state_.speedMps * dt;
  if (distanceAccumulatorM_ >= 1.0f) {
    const double whole = static_cast<double>(static_cast<int>(distanceAccumulatorM_));
    distanceAccumulatorM_ -= static_cast<float>(whole);
    state_.odometerKm += whole / 1000.0;
    state_.distanceSinceClearKm += whole / 1000.0;
  }

  if (state_.engineRunning) {
    runTimeAccumulator_ += dt;
    while (runTimeAccumulator_ >= 1.0f) {
      runTimeAccumulator_ -= 1.0f;
      state_.engineRunSeconds++;
    }
  } else {
    runTimeAccumulator_ = 0.0f;
    state_.engineRunSeconds = 0;
  }

  // A warmup cycle counts once per run, when the engine crosses 70 C having
  // started cold. The old code re-counted this ~20 times per trigger.
  if (!warmupCounted_ && state_.engineRunning && state_.coolantTempC > 70.0f) {
    warmupCounted_ = true;
    state_.warmupsSinceClear++;
  }

  // Fuel burned follows airflow through the commanded mixture, kept in litres
  // as a float so a 50 ms step is never truncated away.
  if (state_.engineRunning && !state_.fuelCut) {
    const float fuelGramsSec = state_.mafGramsSec / (kStoichAfr * state_.lambda);
    const float litersBurned = (fuelGramsSec * dt) / (kFuelDensityKgL * 1000.0f);
    state_.fuelRemainingLiters = clampf(state_.fuelRemainingLiters - litersBurned, 0.0f, kFuelTankLiters);
    state_.tripFuelUsedLiters += litersBurned;

    if (state_.speedKph > 3.0f) {
      const float lPerHour = (fuelGramsSec * 3600.0f) / (kFuelDensityKgL * 1000.0f);
      const float instant = (lPerHour / state_.speedKph) * 100.0f;
      state_.instantConsumptionLper100km +=
          (instant - state_.instantConsumptionLper100km) * clampf(dt * 0.8f, 0.0f, 1.0f);
    }
  }
  state_.fuelLevelPercent = (state_.fuelRemainingLiters / kFuelTankLiters) * 100.0f;
}

}  // namespace vehicle
