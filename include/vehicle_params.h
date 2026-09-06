#pragma once

// Physical parameters for a 2012 Toyota Corolla (E140) 1.8L.
// Powertrain: 2ZR-FE inline-4, U340E 4-speed automatic.
//
// These are the published/measured figures for the real car. Everything the
// simulator emits is derived from them, so changing a number here changes the
// behaviour of the whole model consistently.

namespace vehicle {

// ---------------------------------------------------------------- chassis --
constexpr float kCurbMassKg = 1300.0f;
constexpr float kDriverMassKg = 80.0f;
constexpr float kMassKg = kCurbMassKg + kDriverMassKg;

// 205/55R16: diameter = 16*25.4 + 2*205*0.55 = 631.9 mm.
constexpr float kWheelRadiusM = 0.3160f;
constexpr float kWheelCircumferenceM = 2.0f * 3.14159265f * kWheelRadiusM;

// Road load. F = Crr*m*g + 0.5*rho*Cd*A*v^2
constexpr float kDragCoefficient = 0.29f;
constexpr float kFrontalAreaM2 = 2.20f;
constexpr float kAirDensityKgM3 = 1.225f;
constexpr float kRollingResistance = 0.012f;
constexpr float kGravityMps2 = 9.81f;

// Rotating inertia referred to the road, expressed as an effective mass
// increase. Wheels + driveshafts + gearbox internals.
constexpr float kRotatingMassFactor = 1.06f;

// --------------------------------------------------------------- engine ----
constexpr float kEngineDisplacementM3 = 0.001798f;  // 1.798 L
constexpr float kIdleRpm = 700.0f;
constexpr float kColdIdleRpm = 1250.0f;
constexpr float kRedlineRpm = 6400.0f;
constexpr float kFuelCutRpm = 6500.0f;
constexpr float kCrankRpm = 250.0f;
constexpr float kStallRpm = 400.0f;

// Crankshaft + flywheel + torque converter impeller, kg*m^2.
constexpr float kEngineInertia = 0.20f;

// Volumetric efficiency used in the speed-density airflow calculation.
constexpr float kVolumetricEfficiency = 0.85f;

// Stoichiometric air/fuel ratio for gasoline, and fuel density.
constexpr float kStoichAfr = 14.7f;
constexpr float kFuelDensityKgL = 0.745f;

// --------------------------------------------------- torque converter ------
// Fluid-coupling capacity constant, chosen so stall torque at 2200 rpm equals
// the engine's output there (~158 Nm) -> C = 158 / 2200^2.
constexpr float kConverterCapacity = 3.264e-5f;
constexpr float kConverterStallTorqueRatio = 2.00f;
constexpr float kLockupEngageKph = 60.0f;
constexpr float kLockupReleaseKph = 55.0f;
constexpr float kLockupMaxLoad = 0.78f;

// -------------------------------------------------------- transmission -----
// U340E 4-speed automatic.
constexpr int kGearCount = 4;
constexpr float kGearRatio[kGearCount + 1] = {0.0f, 2.847f, 1.552f, 1.000f, 0.700f};
constexpr float kFinalDriveRatio = 3.943f;
constexpr float kDrivelineEfficiency = 0.92f;
constexpr float kShiftDurationSec = 0.40f;
constexpr float kShiftTorqueCut = 0.40f;  // torque multiplier at the peak of a shift

// --------------------------------------------------------------- brakes ----
// Peak deceleration the modelled brake system can produce.
constexpr float kMaxBrakeDecelMps2 = 8.5f;

// ----------------------------------------------------------------- fuel ----
constexpr float kFuelTankLiters = 55.0f;

// --------------------------------------------------------------- thermal ---
constexpr float kThermostatOpenC = 88.0f;
constexpr float kCoolingFanOnC = 100.0f;
constexpr float kCoolingFanOffC = 94.0f;

// ------------------------------------------------------------- electrical --
constexpr float kBatteryRestingVolts = 12.55f;
constexpr float kChargingVolts = 14.20f;
constexpr float kCrankingVolts = 10.20f;

}  // namespace vehicle
