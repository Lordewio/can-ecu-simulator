#pragma once

#include <stdint.h>

namespace vehicle {

enum class Ignition : uint8_t {
  OFF = 0,
  ACCESSORY = 1,
  ON = 2,
  CRANK = 3,
  RUNNING = 4,
};

// Everything the CAN/OBD layer is allowed to read. Produced by Model::step().
struct State {
  Ignition ignition = Ignition::OFF;
  bool engineRunning = false;
  bool stalled = false;

  // Driver inputs actually applied this step (0..100).
  float pedalPercent = 0.0f;
  float brakePercent = 0.0f;
  float steeringAngleDeg = 0.0f;

  // Powertrain.
  float engineRpm = 0.0f;
  float turbineRpm = 0.0f;
  float throttlePercent = 0.0f;      // plate angle, includes idle-air control
  float engineLoad = 0.0f;           // 0..1, normalised cylinder charge
  float engineTorqueNm = 0.0f;
  float wheelTorqueNm = 0.0f;
  uint8_t gear = 0;                  // 0 = neutral/park, 1..4
  bool shifting = false;
  bool converterLocked = false;
  bool fuelCut = false;

  // Motion.
  float speedMps = 0.0f;
  float speedKph = 0.0f;
  float accelMps2 = 0.0f;
  float wheelSpeedKph[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // FL, FR, RL, RR
  float roadGradePercent = 0.0f;

  // Air / fuel.
  float mapKpa = 0.0f;
  float barometricKpa = 101.0f;
  float mafGramsSec = 0.0f;
  float fuelPressureKpa = 0.0f;      // port injection, gauge
  float lambda = 1.0f;
  float o2DownstreamVolts = 0.45f;
  float shortTrimBank1 = 0.0f;
  float longTrimBank1 = 0.0f;
  bool closedLoop = false;
  float commandedEgrPercent = 0.0f;

  // Thermal.
  float coolantTempC = 0.0f;
  float oilTempC = 0.0f;
  float intakeTempC = 0.0f;
  float ambientTempC = 0.0f;
  bool coolingFanOn = false;
  float oilPressureKpa = 0.0f;

  // Electrical.
  float batteryVolts = 0.0f;
  bool alternatorCharging = false;

  // Accumulators. Odometer is a double: a float cannot resolve a 0.8 m step
  // at 124000 km (ULP there is 7.8 m, so the addition is silently discarded).
  double odometerKm = 0.0;
  double distanceSinceClearKm = 0.0;
  float fuelRemainingLiters = 0.0f;
  float fuelLevelPercent = 0.0f;
  float tripFuelUsedLiters = 0.0f;
  float instantConsumptionLper100km = 0.0f;
  uint32_t engineRunSeconds = 0;
  uint32_t warmupsSinceClear = 0;
};

// Commands into the model for one step.
struct Input {
  float pedalPercent = 0.0f;
  float brakePercent = 0.0f;
  float steeringAngleDeg = 0.0f;
  float roadGradePercent = 0.0f;
  bool keyOn = false;
  bool starterEngaged = false;
};

class Model {
 public:
  // ambientC seeds the soak temperature; odometerKm and fuelLiters seed the
  // accumulators so a restart doesn't reset the vehicle's history.
  void begin(float ambientC, double odometerKm, float fuelLiters);

  // Advances the simulation by dtSeconds. Safe to call at any rate; dt is
  // internally clamped so a scheduling hiccup cannot destabilise the integrator.
  void step(const Input &in, float dtSeconds);

  const State &state() const { return state_; }

  // Fuel-trim/adaptation reset, as service 04 would do.
  void clearAdaptations();

 private:
  void updateIgnition(const Input &in, float dt);
  void updateEngine(const Input &in, float dt);
  void updateTransmission(float dt);
  void updateChassis(const Input &in, float dt);
  void updateAirFuel(float dt);
  void updateThermal(float dt);
  void updateElectrical(float dt);
  void updateAccumulators(float dt);

  State state_;

  float idleIntegrator_ = 0.0f;
  float trimIntegrator_ = 0.0f;
  float lambdaPhase_ = 0.0f;
  float shiftTimer_ = 0.0f;
  uint8_t shiftFromGear_ = 0;
  uint8_t shiftToGear_ = 0;
  float gearHoldTimer_ = 0.0f;
  float crankTimer_ = 0.0f;
  float startGrace_ = 0.0f;
  float runTimeAccumulator_ = 0.0f;
  float distanceAccumulatorM_ = 0.0f;
  bool warmupCounted_ = false;
  float o2Phase_ = 0.0f;
  float noisePhase_ = 0.0f;
};

// Wide-open-throttle torque of the 2ZR-FE at a given crank speed, in Nm.
// Exposed for the host test harness.
float wotTorqueNm(float rpm);

// Engine friction and pumping torque at a given crank speed, in Nm (positive
// number, subtracted from indicated torque).
float frictionTorqueNm(float rpm);

// Crank speed implied by road speed in a given gear with the converter locked.
float rpmForSpeed(float speedKph, uint8_t gear);

}  // namespace vehicle
