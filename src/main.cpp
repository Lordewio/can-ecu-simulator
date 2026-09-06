// CAN ECU simulator -- 2012 Toyota Corolla (E140) 1.8L, 2ZR-FE / U340E.
//
// The bus traffic here is produced by an actual vehicle model: a driver works
// the pedals to track a target speed, the engine makes torque against a road
// load, and the transmission shifts on a real schedule. Every signal broadcast
// or answered over OBD is read out of that model, so throttle, RPM, speed,
// airflow and fuel consumption stay causally consistent with each other.
//
// Hardware: ESP32 DevKit v1 + MCP2515 (8 MHz crystal) at 500 kbit/s.

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#include "driver_model.h"
#include "obd_server.h"
#include "vehicle_model.h"
#include "vehicle_params.h"

namespace {

// ------------------------------------------------------------------ wiring --
constexpr int CAN_INT_PIN = 4;
constexpr int CAN_SCK_PIN = 18;
constexpr int CAN_MISO_PIN = 19;
constexpr int CAN_MOSI_PIN = 23;
constexpr int CAN_CS_PIN = 5;
constexpr auto MCP2515_CLOCK = MCP_8MHZ;

// --------------------------------------------------------------- broadcast --
// Toyota-style 11-bit identifiers for bench use. Layouts are documented in
// each broadcast function; they are plausible rather than reverse-engineered
// from a real car, and are not claimed to match Toyota's own encoding.
constexpr uint16_t CAN_ENGINE_FAST = 0x0AA;
constexpr uint16_t CAN_WHEEL_SPEEDS = 0x0B4;
constexpr uint16_t CAN_POWERTRAIN = 0x0B0;
constexpr uint16_t CAN_TRANSMISSION = 0x1D2;
constexpr uint16_t CAN_BRAKE_STEER = 0x260;
constexpr uint16_t CAN_FUEL_EMISSIONS = 0x3B3;
constexpr uint16_t CAN_BODY_HVAC = 0x620;
constexpr uint16_t CAN_CHARGING = 0x3C8;
constexpr uint16_t CAN_CRUISE_ODOMETER = 0x2A0;
constexpr uint16_t CAN_TPMS = 0x385;

// Seed values for a car that has been on the road a while.
constexpr double kInitialOdometerKm = 124532.4;
constexpr float kInitialFuelLiters = 41.0f;
constexpr float kInitialAmbientC = 24.0f;

MCP2515 mcp2515(CAN_CS_PIN);
vehicle::Model model;
driver::Driver pilot;
obd::Server diagnostics;

uint32_t lastStepMicros = 0;
float stepRemainder = 0.0f;

// Rolling counters. Only the frames that would carry one on a real bus have
// one -- a uniform counter and checksum on every single message is itself a
// giveaway that traffic is synthetic.
uint8_t ctrEngine = 0;
uint8_t ctrTransmission = 0;
uint8_t ctrBrake = 0;
uint8_t ctrTpms = 0;

// ------------------------------------------------------------------- timing --
struct Schedule {
  uint32_t periodMs;
  uint32_t nextMs;
};

Schedule schEngine = {20, 0};
Schedule schWheels = {20, 0};
Schedule schPowertrain = {50, 0};
Schedule schTransmission = {50, 0};
Schedule schBrake = {25, 0};
Schedule schFuel = {100, 0};
Schedule schBody = {200, 0};
Schedule schCharging = {100, 0};
Schedule schCruise = {100, 0};
Schedule schTpms = {500, 0};

uint32_t rngState = 0x2A1D1234;

uint32_t nextRandom() {
  // xorshift32: cheap, and good enough for timing jitter.
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return rngState;
}

// Real ECUs do not hit their period to the millisecond; scheduler and clock
// drift show up as a percent or two of jitter on every cyclic message.
bool due(Schedule &s, uint32_t now) {
  if (static_cast<int32_t>(now - s.nextMs) < 0) return false;
  const uint32_t jitterSpan = (s.periodMs / 25) + 1;  // about +/-4%
  const int32_t jitter = static_cast<int32_t>(nextRandom() % (2 * jitterSpan + 1)) -
                         static_cast<int32_t>(jitterSpan);
  s.nextMs = now + s.periodMs + jitter;
  return true;
}

// --------------------------------------------------------------- utilities --
uint8_t percentByte(float percent) {
  float v = percent * 255.0f / 100.0f;
  if (v < 0.0f) v = 0.0f;
  if (v > 255.0f) v = 255.0f;
  return static_cast<uint8_t>(v + 0.5f);
}

uint8_t tempByte(float celsius) {
  float v = celsius + 40.0f;
  if (v < 0.0f) v = 0.0f;
  if (v > 255.0f) v = 255.0f;
  return static_cast<uint8_t>(v + 0.5f);
}

uint16_t clampU16(float value) {
  if (value < 0.0f) return 0;
  if (value > 65535.0f) return 65535;
  return static_cast<uint16_t>(value + 0.5f);
}

void put16(uint8_t *out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value >> 8);
  out[1] = static_cast<uint8_t>(value & 0xFF);
}

uint8_t crc8Sae(const uint8_t *data, uint8_t length) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x1D) : static_cast<uint8_t>(crc << 1);
    }
  }
  return static_cast<uint8_t>(crc ^ 0xFF);
}

void sendFrame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  struct can_frame frame {};
  frame.can_id = id;
  frame.can_dlc = dlc;
  for (uint8_t i = 0; i < dlc && i < 8; ++i) frame.data[i] = data[i];
  mcp2515.sendMessage(&frame);
}

// Stamps a 4-bit rolling counter and a CRC over the identifier and payload
// into the last two bytes. Only used on frames that reserve room for it.
void stampProtection(uint16_t id, uint8_t *frame, uint8_t dlc, uint8_t &counter) {
  frame[dlc - 2] = static_cast<uint8_t>(counter & 0x0F);
  uint8_t buffer[10];
  buffer[0] = static_cast<uint8_t>(id >> 8);
  buffer[1] = static_cast<uint8_t>(id & 0xFF);
  for (uint8_t i = 0; i + 1 < dlc; ++i) buffer[2 + i] = frame[i];
  frame[dlc - 1] = crc8Sae(buffer, static_cast<uint8_t>(dlc + 1));
  counter = static_cast<uint8_t>((counter + 1) & 0x0F);
}

// ------------------------------------------------------------- broadcasts ---

// 0x0AA, 20 ms, DLC 8: rpm | throttle | pedal | load | status | ctr | crc
void broadcastEngineFast(const vehicle::State &s) {
  uint8_t f[8];
  put16(&f[0], clampU16(s.engineRpm * 4.0f));  // quarter-rpm per bit
  f[2] = percentByte(s.throttlePercent);
  f[3] = percentByte(s.pedalPercent);
  f[4] = percentByte(s.engineLoad * 100.0f);
  f[5] = static_cast<uint8_t>((static_cast<uint8_t>(s.ignition) & 0x07) |
                              (s.engineRunning ? 0x08 : 0x00) |
                              (s.fuelCut ? 0x10 : 0x00) |
                              (s.closedLoop ? 0x20 : 0x00));
  stampProtection(CAN_ENGINE_FAST, f, 8, ctrEngine);
  sendFrame(CAN_ENGINE_FAST, f, 8);
}

// 0x0B4, 20 ms, DLC 8: four wheel speeds at quarter km/h per bit.
// All eight bytes are wheel data -- there is deliberately no counter or
// checksum here, because there is nowhere to put one without destroying a
// wheel speed. (The previous firmware stamped bytes 6 and 7, which silently
// overwrote the rear-right wheel with counter and CRC values.)
void broadcastWheelSpeeds(const vehicle::State &s) {
  uint8_t f[8];
  for (int w = 0; w < 4; ++w) {
    put16(&f[w * 2], clampU16(s.wheelSpeedKph[w] * 4.0f));
  }
  sendFrame(CAN_WHEEL_SPEEDS, f, 8);
}

// 0x0B0, 50 ms, DLC 7: coolant | intake | MAP | MAF | gear | flags
void broadcastPowertrain(const vehicle::State &s) {
  uint8_t f[7];
  f[0] = tempByte(s.coolantTempC);
  f[1] = tempByte(s.intakeTempC);
  f[2] = static_cast<uint8_t>(s.mapKpa + 0.5f);
  put16(&f[3], clampU16(s.mafGramsSec * 100.0f));
  f[5] = static_cast<uint8_t>((s.gear & 0x0F) | (s.converterLocked ? 0x40 : 0x00) |
                              (s.shifting ? 0x80 : 0x00));
  f[6] = static_cast<uint8_t>((s.coolingFanOn ? 0x01 : 0x00) |
                              (s.alternatorCharging ? 0x02 : 0x00) |
                              (diagnostics.milOn() ? 0x04 : 0x00));
  sendFrame(CAN_POWERTRAIN, f, 7);
}

// 0x1D2, 50 ms, DLC 6: gear | turbine rpm | flags | ctr | crc
void broadcastTransmission(const vehicle::State &s) {
  uint8_t f[6];
  f[0] = static_cast<uint8_t>(s.gear & 0x0F);
  put16(&f[1], clampU16(s.turbineRpm));
  f[3] = static_cast<uint8_t>((s.converterLocked ? 0x01 : 0x00) | (s.shifting ? 0x02 : 0x00));
  stampProtection(CAN_TRANSMISSION, f, 6, ctrTransmission);
  sendFrame(CAN_TRANSMISSION, f, 6);
}

// 0x260, 25 ms, DLC 5: steering angle | brake | ctr | crc
void broadcastBrakeSteer(const vehicle::State &s) {
  uint8_t f[5];
  const int16_t angle = static_cast<int16_t>(s.steeringAngleDeg * 10.0f);
  put16(&f[0], static_cast<uint16_t>(angle));  // tenth of a degree, signed
  f[2] = percentByte(s.brakePercent);
  stampProtection(CAN_BRAKE_STEER, f, 5, ctrBrake);
  sendFrame(CAN_BRAKE_STEER, f, 5);
}

// 0x3B3, 100 ms, DLC 8: fuel level | trims | lambda | O2 | EGR
void broadcastFuelEmissions(const vehicle::State &s) {
  uint8_t f[8];
  f[0] = percentByte(s.fuelLevelPercent);
  f[1] = static_cast<uint8_t>((s.shortTrimBank1 + 100.0f) * 128.0f / 100.0f);
  f[2] = static_cast<uint8_t>((s.longTrimBank1 + 100.0f) * 128.0f / 100.0f);
  put16(&f[3], clampU16(s.lambda * 32768.0f));
  f[5] = static_cast<uint8_t>(s.o2DownstreamVolts / 0.005f);
  f[6] = percentByte(s.commandedEgrPercent);
  f[7] = static_cast<uint8_t>(s.closedLoop ? 0x01 : 0x00);
  sendFrame(CAN_FUEL_EMISSIONS, f, 8);
}

// 0x620, 200 ms, DLC 5: ambient | doors | lights | HVAC
void broadcastBodyHvac(const vehicle::State &s) {
  uint8_t f[5];
  f[0] = tempByte(s.ambientTempC);
  f[1] = 0x00;  // all doors closed
  f[2] = static_cast<uint8_t>(s.ignition >= vehicle::Ignition::ON ? 0x03 : 0x00);
  f[3] = 22;    // HVAC setpoint, degrees C
  f[4] = static_cast<uint8_t>(s.coolingFanOn ? 0x02 : 0x01);
  sendFrame(CAN_BODY_HVAC, f, 5);
}

// 0x3C8, 100 ms, DLC 4: battery millivolts | alternator | load
void broadcastCharging(const vehicle::State &s) {
  uint8_t f[4];
  put16(&f[0], clampU16(s.batteryVolts * 1000.0f));
  f[2] = static_cast<uint8_t>(s.alternatorCharging ? 0x01 : 0x00);
  f[3] = percentByte(s.alternatorCharging ? 45.0f : 0.0f);
  sendFrame(CAN_CHARGING, f, 4);
}

// 0x2A0, 100 ms, DLC 8: odometer | speed | runtime | cruise
void broadcastCruiseOdometer(const vehicle::State &s) {
  uint8_t f[8];
  const uint32_t odo = static_cast<uint32_t>(s.odometerKm);
  f[0] = static_cast<uint8_t>((odo >> 16) & 0xFF);
  f[1] = static_cast<uint8_t>((odo >> 8) & 0xFF);
  f[2] = static_cast<uint8_t>(odo & 0xFF);
  f[3] = static_cast<uint8_t>(s.speedKph);
  put16(&f[4], clampU16(static_cast<float>(s.engineRunSeconds)));
  f[6] = 0x00;  // cruise control not engaged
  f[7] = static_cast<uint8_t>(s.instantConsumptionLper100km * 10.0f);
  sendFrame(CAN_CRUISE_ODOMETER, f, 8);
}

// 0x385, 500 ms, DLC 8: four tyre pressures | temperature | status | ctr | crc
// Pressures and a single shared temperature, so the counter and checksum have
// somewhere to live. (The previous firmware packed four temperatures here and
// then overwrote two of them with the counter and CRC.)
void broadcastTpms(const vehicle::State &s) {
  uint8_t f[8];
  // Cold pressure rises a little as the tyres heat up with speed.
  const float heat = s.speedKph * 0.02f;
  f[0] = static_cast<uint8_t>((35.0f + heat) * 4.0f);  // quarter-psi per bit
  f[1] = static_cast<uint8_t>((35.0f + heat) * 4.0f);
  f[2] = static_cast<uint8_t>((34.0f + heat) * 4.0f);
  f[3] = static_cast<uint8_t>((34.0f + heat) * 4.0f);
  f[4] = tempByte(s.ambientTempC + 6.0f + s.speedKph * 0.06f);
  f[5] = 0x00;  // no pressure warnings
  stampProtection(CAN_TPMS, f, 8, ctrTpms);
  sendFrame(CAN_TPMS, f, 8);
}

// ---------------------------------------------------------- serial console --

void printHelp() {
  Serial.println(F("commands:"));
  Serial.println(F("  status                 current vehicle state"));
  Serial.println(F("  scenario <name>        trip | city | highway | idle | manual | keyoff"));
  Serial.println(F("  speed <kph>            target speed (switches to manual)"));
  Serial.println(F("  dtc set <code>         store a confirmed fault, e.g. dtc set P0171"));
  Serial.println(F("  dtc pending <code>     store a pending fault"));
  Serial.println(F("  dtc list               show stored faults"));
  Serial.println(F("  dtc clear              clear faults (permanent codes survive)"));
  Serial.println(F("  help                   this list"));
}

void printStatus() {
  const vehicle::State &s = model.state();
  static const char *kIgnition[] = {"OFF", "ACC", "ON", "CRANK", "RUN"};
  Serial.println();
  Serial.printf("scenario %s / phase %-12s target %5.1f km/h\n",
                pilot.scenario() == driver::Scenario::TRIP      ? "trip"
                : pilot.scenario() == driver::Scenario::CITY    ? "city"
                : pilot.scenario() == driver::Scenario::HIGHWAY ? "highway"
                : pilot.scenario() == driver::Scenario::IDLE    ? "idle"
                : pilot.scenario() == driver::Scenario::MANUAL  ? "manual"
                                                                : "keyoff",
                pilot.phaseName(), pilot.targetKph());
  Serial.printf("  ignition %-5s  speed %6.1f km/h  rpm %6.0f  gear %d%s%s\n",
                kIgnition[static_cast<int>(s.ignition)], s.speedKph, s.engineRpm, s.gear,
                s.converterLocked ? " LOCK" : "", s.shifting ? " SHIFT" : "");
  Serial.printf("  pedal %5.1f%%  throttle %5.1f%%  brake %5.1f%%  load %4.2f%s\n",
                s.pedalPercent, s.throttlePercent, s.brakePercent, s.engineLoad,
                s.fuelCut ? "  DFCO" : "");
  Serial.printf("  coolant %5.1fC  oil %5.1fC  intake %5.1fC  ambient %5.1fC%s\n",
                s.coolantTempC, s.oilTempC, s.intakeTempC, s.ambientTempC,
                s.coolingFanOn ? "  FAN" : "");
  Serial.printf("  MAP %5.1f kPa  MAF %6.2f g/s  lambda %4.2f  %s  STFT %+5.1f%%  LTFT %+5.1f%%\n",
                s.mapKpa, s.mafGramsSec, s.lambda, s.closedLoop ? "closed-loop" : "open-loop ",
                s.shortTrimBank1, s.longTrimBank1);
  Serial.printf("  battery %5.2f V  oil pressure %5.0f kPa\n", s.batteryVolts, s.oilPressureKpa);
  Serial.printf("  odometer %10.2f km  fuel %5.1f%% (%5.2f L)  %5.2f L/100km  runtime %lus\n",
                s.odometerKm, s.fuelLevelPercent, s.fuelRemainingLiters,
                s.instantConsumptionLper100km, static_cast<unsigned long>(s.engineRunSeconds));
  Serial.printf("  MIL %s  confirmed %u  pending %u\n", diagnostics.milOn() ? "ON " : "off",
                diagnostics.confirmedCount(), diagnostics.pendingCount());
}

void printDtcs() {
  const obd::Dtc *list = diagnostics.dtcs();
  bool any = false;
  Serial.println(F("stored faults:"));
  for (int i = 0; i < obd::kMaxDtcs; ++i) {
    if (!list[i].used) continue;
    char text[6];
    obd::decodeDtc(list[i].code, text);
    Serial.printf("  %s  %-4s %s%s%s\n", text,
                  obd::kModules[static_cast<int>(list[i].module)].name + 4,
                  list[i].confirmed ? "confirmed " : "",
                  list[i].pending ? "pending " : "",
                  list[i].permanent ? "permanent" : "");
    any = true;
  }
  if (!any) Serial.println(F("  (none)"));
}

void handleCommand(char *line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;

  if (strncmp(line, "help", 4) == 0) {
    printHelp();
  } else if (strncmp(line, "status", 6) == 0) {
    printStatus();
  } else if (strncmp(line, "speed ", 6) == 0) {
    const float kph = atof(line + 6);
    pilot.setScenario(driver::Scenario::MANUAL);
    pilot.setManualTarget(kph);
    Serial.printf("target speed %.1f km/h\n", kph);
  } else if (strncmp(line, "scenario ", 9) == 0) {
    const char *name = line + 9;
    driver::Scenario s = driver::Scenario::TRIP;
    if (strncmp(name, "city", 4) == 0) s = driver::Scenario::CITY;
    else if (strncmp(name, "highway", 7) == 0) s = driver::Scenario::HIGHWAY;
    else if (strncmp(name, "idle", 4) == 0) s = driver::Scenario::IDLE;
    else if (strncmp(name, "manual", 6) == 0) s = driver::Scenario::MANUAL;
    else if (strncmp(name, "keyoff", 6) == 0) s = driver::Scenario::KEY_OFF;
    else if (strncmp(name, "trip", 4) != 0) {
      Serial.println(F("unknown scenario"));
      return;
    }
    pilot.setScenario(s);
    Serial.printf("scenario -> %s\n", name);
  } else if (strncmp(line, "dtc ", 4) == 0) {
    const char *arg = line + 4;
    if (strncmp(arg, "list", 4) == 0) {
      printDtcs();
    } else if (strncmp(arg, "clear", 5) == 0) {
      diagnostics.clearAll();
      Serial.println(F("faults cleared (permanent codes retained)"));
    } else if (strncmp(arg, "set ", 4) == 0 || strncmp(arg, "pending ", 8) == 0) {
      const bool confirmed = (arg[0] == 's');
      const char *code = confirmed ? arg + 4 : arg + 8;
      const uint16_t encoded = obd::encodeDtc(code);
      if (encoded == 0) {
        Serial.println(F("bad code, expected something like P0171"));
        return;
      }
      // Chassis codes belong to the ABS module, everything else to the ECM.
      const obd::Module target =
          (((encoded >> 14) & 0x03) == 1) ? obd::Module::ABS : obd::Module::ECM;
      if (diagnostics.setDtc(encoded, target, confirmed)) {
        char text[6];
        obd::decodeDtc(encoded, text);
        Serial.printf("stored %s as %s\n", text, confirmed ? "confirmed" : "pending");
      } else {
        Serial.println(F("fault table full"));
      }
    } else {
      Serial.println(F("dtc set|pending|list|clear"));
    }
  } else {
    Serial.println(F("unknown command, try 'help'"));
  }
}

void pollConsole() {
  static char buffer[64];
  static uint8_t used = 0;
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      buffer[used] = '\0';
      handleCommand(buffer);
      used = 0;
      continue;
    }
    if (used < sizeof(buffer) - 1) buffer[used++] = c;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("\n=== CAN ECU simulator ==="));
  Serial.println(F("2012 Toyota Corolla E140 1.8L -- 2ZR-FE / U340E 4-speed automatic"));
  Serial.println(F("ESP32 + MCP2515, 500 kbit/s"));
  Serial.println();

  rngState ^= static_cast<uint32_t>(esp_random());
  if (rngState == 0) rngState = 0x1234ABCD;

  model.begin(kInitialAmbientC, kInitialOdometerKm, kInitialFuelLiters);
  pilot.begin(driver::Scenario::TRIP);

  pinMode(CAN_INT_PIN, INPUT_PULLUP);
  SPI.begin(CAN_SCK_PIN, CAN_MISO_PIN, CAN_MOSI_PIN, CAN_CS_PIN);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP2515_CLOCK);
  mcp2515.setNormalMode();
  diagnostics.begin(&mcp2515, &model);

  Serial.println(F("MCP2515 up. Type 'help' for the console."));
  lastStepMicros = micros();
}

void loop() {
  const uint32_t nowMicros = micros();
  const uint32_t nowMs = millis();

  // Fixed-step integration. Real elapsed time is accumulated and consumed in
  // 10 ms steps so the physics stays stable regardless of loop timing.
  float elapsed = static_cast<float>(nowMicros - lastStepMicros) / 1e6f;
  lastStepMicros = nowMicros;
  if (elapsed > 0.25f) elapsed = 0.25f;  // never try to catch up after a stall

  stepRemainder += elapsed;
  constexpr float kStep = 0.010f;
  int guard = 0;
  while (stepRemainder >= kStep && guard++ < 32) {
    stepRemainder -= kStep;
    const vehicle::Input in = pilot.update(model.state(), kStep);
    model.step(in, kStep);
  }

  const vehicle::State &s = model.state();

  if (due(schEngine, nowMs)) broadcastEngineFast(s);
  if (due(schWheels, nowMs)) broadcastWheelSpeeds(s);
  if (due(schPowertrain, nowMs)) broadcastPowertrain(s);
  if (due(schTransmission, nowMs)) broadcastTransmission(s);
  if (due(schBrake, nowMs)) broadcastBrakeSteer(s);
  if (due(schFuel, nowMs)) broadcastFuelEmissions(s);
  if (due(schBody, nowMs)) broadcastBodyHvac(s);
  if (due(schCharging, nowMs)) broadcastCharging(s);
  if (due(schCruise, nowMs)) broadcastCruiseOdometer(s);
  if (due(schTpms, nowMs)) broadcastTpms(s);

  // Drain the controller's receive buffers. Reading several frames per pass
  // keeps a scan tool that fires requests back to back from overflowing them.
  struct can_frame frame {};
  for (int i = 0; i < 4; ++i) {
    if (mcp2515.readMessage(&frame) != MCP2515::ERROR_OK) break;
    diagnostics.handleFrame(frame);
  }

  diagnostics.pump();
  pollConsole();
}
