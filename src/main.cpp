#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <cmath>
#include <cstring>

namespace {
constexpr int CAN_INT_PIN = 4;
constexpr int CAN_SCK_PIN = 18;
constexpr int CAN_MISO_PIN = 19;
constexpr int CAN_MOSI_PIN = 23;
constexpr int CAN_CS_PIN = 5;

constexpr uint16_t OBD2_FUNCTIONAL_REQUEST = 0x7DF;
constexpr uint16_t OBD2_TESTER_REQUEST_ECM = 0x7E0;
constexpr uint16_t OBD2_TESTER_REQUEST_TCM = 0x7E1;
constexpr uint16_t OBD2_TESTER_REQUEST_ABS = 0x7E2;
constexpr uint16_t OBD2_TESTER_REQUEST_BCM = 0x7E3;
constexpr uint16_t OBD2_ECU_RESPONSE_ECM = 0x7E8;
constexpr uint16_t OBD2_ECU_RESPONSE_TCM = 0x7E9;
constexpr uint16_t OBD2_ECU_RESPONSE_ABS = 0x7EA;
constexpr uint16_t OBD2_ECU_RESPONSE_BCM = 0x7EB;

// Toyota Corolla E140 (2012) inspired 11-bit IDs for bench simulation.
constexpr uint16_t CAN_ENGINE_FAST = 0x0AA;
constexpr uint16_t CAN_POWERTRAIN = 0x0B0;
constexpr uint16_t CAN_TRANSMISSION = 0x1D2;
constexpr uint16_t CAN_ABS_WHEELS = 0x0B4;
constexpr uint16_t CAN_BRAKE_STEER = 0x260;
constexpr uint16_t CAN_FUEL_EMISSIONS = 0x3B3;
constexpr uint16_t CAN_BODY_HVAC = 0x620;
constexpr uint16_t CAN_CHARGING = 0x3C8;
constexpr uint16_t CAN_CRUISE_ODOMETER = 0x2A0;
constexpr uint16_t CAN_TPMS = 0x385;

constexpr auto MCP2515_CLOCK = MCP_8MHZ;
constexpr uint8_t MODE01_MAX_PID = 0xE0;

constexpr char kVehicleProfile[] = "Toyota Corolla E140 2012 1.8L";
enum class IgnitionState : uint8_t {
  OFF = 0,
  ACCESSORY = 1,
  ON = 2,
  CRANK = 3,
  RUNNING = 4,
};

MCP2515 mcp2515(CAN_CS_PIN);

IgnitionState ignitionState = IgnitionState::RUNNING;
uint16_t engineRpm = 850;
uint8_t vehicleSpeedKph = 0;
uint8_t coolantTempC = 75;
uint8_t intakeTempC = 30;
uint8_t ambientTempC = 27;
uint8_t oilTempC = 82;
uint8_t throttlePercent = 12;
uint16_t manifoldPressureKpa = 35;
uint16_t fuelRailPressureKpa = 420;
uint16_t fuelPressureKpa = 360;
uint16_t oilPressureKpa = 260;
uint16_t mafCentiGramsSec = 1200;
uint8_t o2SensorVoltageRaw = 128;
int8_t shortTermFuelTrimBank1 = 0;
int8_t longTermFuelTrimBank1 = 0;
int8_t shortTermFuelTrimBank2 = 0;
int8_t longTermFuelTrimBank2 = 0;
uint8_t commandedEgrPercent = 0;
uint8_t relativeThrottlePercent = 14;
uint8_t accelPedalPercent = 18;
uint8_t commandedThrottlePercent = 15;
uint8_t transmissionGear = 1;
uint8_t fuelLevelPercent = 82;
uint8_t barometricPressureKpa = 100;
uint16_t moduleVoltageMilliVolts = 13820;
uint16_t engineRuntimeSec = 0;
uint16_t milOnTimeMin = 0;
uint16_t warmupsSinceClear = 3;
float odometerKm = 124532.4f;
float distanceSinceClearKm = 128.0f;
float distanceMilOnKm = 0.0f;
unsigned long lastStateUpdateMs = 0;
unsigned long lastDistanceUpdateMs = 0;
unsigned long lastEngineFastMs = 0;
unsigned long lastPowertrainMs = 0;
unsigned long lastTransmissionMs = 0;
unsigned long lastAbsMs = 0;
unsigned long lastBrakeSteerMs = 0;
unsigned long lastFuelEmissionsMs = 0;
unsigned long lastBodyMs = 0;
unsigned long lastChargingMs = 0;
unsigned long lastCruiseMs = 0;
unsigned long lastTpmsMs = 0;

constexpr char kVin[] = "1HGBH41JXMN109186";
constexpr char kCalId[] = "SIMCAL2026A";
constexpr char kEcuName[] = "SIM-ECM-ESP32";
constexpr char kTcmName[] = "SIM-TCM-ESP32";
constexpr char kAbsName[] = "SIM-ABS-ESP32";
constexpr char kBcmName[] = "SIM-BCM-ESP32";

enum class DiagModule : uint8_t {
  ECM = 0,
  TCM = 1,
  ABS = 2,
  BCM = 3,
};

DiagModule activeDiagModule = DiagModule::ECM;
uint16_t activeObdResponseId = OBD2_ECU_RESPONSE_ECM;

uint8_t ctrEngineFast = 0;
uint8_t ctrPowertrain = 0;
uint8_t ctrTransmission = 0;
uint8_t ctrAbs = 0;
uint8_t ctrBrakeSteer = 0;
uint8_t ctrFuelEmissions = 0;
uint8_t ctrBody = 0;
uint8_t ctrCharging = 0;
uint8_t ctrCruise = 0;
uint8_t ctrTpms = 0;

template <typename T>
T clampValue(T value, T minValue, T maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

uint8_t percentToByte(uint8_t percent) {
  return static_cast<uint8_t>((static_cast<uint16_t>(percent) * 255U) / 100U);
}

uint8_t trimPercentToByte(int8_t trimPercent) {
  const int16_t centered = static_cast<int16_t>(trimPercent) + 100;
  const int16_t scaled = (centered * 128) / 100;
  return static_cast<uint8_t>(clampValue<int16_t>(scaled, 0, 255));
}

uint8_t calcCrc8(const uint8_t *data, uint8_t length, uint8_t initial = 0xFFU, uint8_t poly = 0x1DU) {
  uint8_t crc = initial;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x80U) != 0U) {
        crc = static_cast<uint8_t>((crc << 1U) ^ poly);
      } else {
        crc <<= 1U;
      }
    }
  }
  return crc;
}

uint8_t calcFrameChecksum(uint16_t canId, const uint8_t *frame, uint8_t count) {
  uint8_t data[9] = {
      static_cast<uint8_t>((canId >> 8) & 0xFFU),
      static_cast<uint8_t>(canId & 0xFFU),
      0,
      0,
      0,
      0,
      0,
      0,
      0,
  };
  for (uint8_t i = 0; i < count && (i + 2U) < sizeof(data); ++i) {
    data[i + 2U] = frame[i];
  }
  return static_cast<uint8_t>(calcCrc8(data, static_cast<uint8_t>(count + 2U)) ^ 0xFFU);
}

void setDiagnosticContext(uint16_t requestId) {
  if (requestId == OBD2_TESTER_REQUEST_TCM) {
    activeDiagModule = DiagModule::TCM;
    activeObdResponseId = OBD2_ECU_RESPONSE_TCM;
  } else if (requestId == OBD2_TESTER_REQUEST_ABS) {
    activeDiagModule = DiagModule::ABS;
    activeObdResponseId = OBD2_ECU_RESPONSE_ABS;
  } else if (requestId == OBD2_TESTER_REQUEST_BCM) {
    activeDiagModule = DiagModule::BCM;
    activeObdResponseId = OBD2_ECU_RESPONSE_BCM;
  } else {
    activeDiagModule = DiagModule::ECM;
    activeObdResponseId = OBD2_ECU_RESPONSE_ECM;
  }
}

const char *getActiveEcuName() {
  if (activeDiagModule == DiagModule::TCM) {
    return kTcmName;
  }
  if (activeDiagModule == DiagModule::ABS) {
    return kAbsName;
  }
  if (activeDiagModule == DiagModule::BCM) {
    return kBcmName;
  }
  return kEcuName;
}

void encodeU16(uint16_t value, uint8_t *out) {
  out[0] = static_cast<uint8_t>(value >> 8);
  out[1] = static_cast<uint8_t>(value & 0xFFU);
}

void encodeI16(int16_t value, uint8_t *out) {
  out[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
  out[1] = static_cast<uint8_t>(value & 0xFF);
}

void stampCounterAndChecksum(uint16_t canId, uint8_t *frame, uint8_t &counter) {
  frame[6] = static_cast<uint8_t>(counter & 0x0FU);
  frame[7] = calcFrameChecksum(canId, frame, 7);
  counter = static_cast<uint8_t>((counter + 1U) & 0x0FU);
}

void sendCanFrame(uint16_t id, const uint8_t *data, uint8_t length) {
  struct can_frame frame {};
  frame.can_id = id;
  frame.can_dlc = length;
  for (uint8_t i = 0; i < length; ++i) {
    frame.data[i] = data[i];
  }
  mcp2515.sendMessage(&frame);
}

void sendObdNegativeResponse(uint8_t service, uint8_t reason) {
  const uint8_t payload[3] = {0x7F, service, reason};
  uint8_t frame[8] = {static_cast<uint8_t>(payload[0] ? 3 : 0), payload[0], payload[1], payload[2], 0, 0, 0, 0};
  sendCanFrame(activeObdResponseId, frame, 8);
}

void sendObdSingleFrame(uint8_t responseService, const uint8_t *data, uint8_t dataLength) {
  if (dataLength > 6) {
    return;
  }
  uint8_t frame[8] = {static_cast<uint8_t>(dataLength + 1U), responseService, 0, 0, 0, 0, 0, 0};
  for (uint8_t i = 0; i < dataLength; ++i) {
    frame[2 + i] = data[i];
  }
  sendCanFrame(activeObdResponseId, frame, 8);
}

void sendObdIsoTp(uint8_t responseService, const uint8_t *data, uint16_t dataLength) {
  const uint16_t totalLength = static_cast<uint16_t>(dataLength + 1U);
  if (totalLength <= 7U) {
    sendObdSingleFrame(responseService, data, static_cast<uint8_t>(dataLength));
    return;
  }

  uint8_t firstFrame[8] = {
      static_cast<uint8_t>(0x10U | ((totalLength >> 8) & 0x0FU)),
      static_cast<uint8_t>(totalLength & 0xFFU),
      responseService,
      0,
      0,
      0,
      0,
      0,
  };

  uint16_t index = 0;
  for (uint8_t i = 3; i < 8 && index < dataLength; ++i) {
    firstFrame[i] = data[index++];
  }
  sendCanFrame(activeObdResponseId, firstFrame, 8);

  uint8_t sequence = 1;
  while (index < dataLength) {
    uint8_t cf[8] = {static_cast<uint8_t>(0x20U | (sequence & 0x0FU)), 0, 0, 0, 0, 0, 0, 0};
    for (uint8_t i = 1; i < 8 && index < dataLength; ++i) {
      cf[i] = data[index++];
    }
    sendCanFrame(activeObdResponseId, cf, 8);
    sequence = static_cast<uint8_t>((sequence + 1U) & 0x0FU);
    if (sequence == 0U) {
      sequence = 1U;
    }
    delay(2);
  }
}

void sendMode01Response(uint8_t pid, const uint8_t *payload, uint8_t length) {
  uint8_t data[7] = {pid, 0, 0, 0, 0, 0, 0};
  for (uint8_t i = 0; i < length && (i + 1U) < sizeof(data); ++i) {
    data[i + 1U] = payload[i];
  }
  sendObdSingleFrame(0x41, data, static_cast<uint8_t>(length + 1U));
}

void sendSupportedPidsRange(uint8_t rangePid) {
  if (rangePid > MODE01_MAX_PID || (rangePid % 0x20U) != 0U) {
    return;
  }

  const bool hasNextRange = rangePid < MODE01_MAX_PID;
  const uint8_t payload[4] = {
      static_cast<uint8_t>(hasNextRange ? 0xFFU : 0x00U),
      static_cast<uint8_t>(hasNextRange ? 0xFFU : 0x00U),
      static_cast<uint8_t>(hasNextRange ? 0xFFU : 0x00U),
      static_cast<uint8_t>(hasNextRange ? 0xFFU : 0x00U),
  };
  sendMode01Response(rangePid, payload, sizeof(payload));
}

uint8_t inferMode01Length(uint8_t pid) {
  if (pid >= 0x14U && pid <= 0x1BU) {
    return 2;
  }
  if (pid >= 0x24U && pid <= 0x2BU) {
    return 4;
  }
  if (pid >= 0x34U && pid <= 0x3BU) {
    return 4;
  }
  switch (pid) {
    case 0x01:
    case 0x20:
    case 0x40:
    case 0x60:
    case 0x80:
    case 0xA0:
    case 0xC0:
    case 0xE0:
      return 4;
    case 0x02:
    case 0x03:
    case 0x0C:
    case 0x10:
    case 0x1F:
    case 0x21:
    case 0x22:
    case 0x23:
    case 0x31:
    case 0x42:
    case 0x43:
    case 0x4D:
    case 0x4E:
    case 0x5E:
      return 2;
    default:
      return 1;
  }
}

void fillGenericMode01Payload(uint8_t pid, uint8_t *payload, uint8_t length) {
  for (uint8_t i = 0; i < length; ++i) {
    const uint16_t mixed = static_cast<uint16_t>(pid * 37U) + static_cast<uint16_t>(i * 29U) +
                           static_cast<uint16_t>(vehicleSpeedKph + throttlePercent + transmissionGear * 11U);
    payload[i] = static_cast<uint8_t>(mixed & 0xFFU);
  }
}

bool buildMode01Payload(uint8_t pid, uint8_t *payload, uint8_t &length) {
  if ((pid % 0x20U) == 0U) {
    return false;
  }

  switch (pid) {
    case 0x01: {
      length = 4;
      payload[0] = 0x00;
      payload[1] = 0x07;
      payload[2] = 0xE1;
      payload[3] = 0x00;
      return true;
    }
    case 0x02: {
      length = 2;
      payload[0] = 0x00;
      payload[1] = 0x00;
      return true;
    }
    case 0x03: {
      length = 2;
      payload[0] = 0x01;
      payload[1] = 0x00;
      return true;
    }
    case 0x04: {
      length = 1;
      payload[0] = percentToByte(throttlePercent);
      return true;
    }
    case 0x05: {
      length = 1;
      payload[0] = static_cast<uint8_t>(coolantTempC + 40U);
      return true;
    }
    case 0x06: {
      length = 1;
      payload[0] = trimPercentToByte(shortTermFuelTrimBank1);
      return true;
    }
    case 0x07: {
      length = 1;
      payload[0] = trimPercentToByte(longTermFuelTrimBank1);
      return true;
    }
    case 0x08: {
      length = 1;
      payload[0] = trimPercentToByte(shortTermFuelTrimBank2);
      return true;
    }
    case 0x09: {
      length = 1;
      payload[0] = trimPercentToByte(longTermFuelTrimBank2);
      return true;
    }
    case 0x0A: {
      length = 1;
      payload[0] = static_cast<uint8_t>(clampValue<uint16_t>(fuelPressureKpa / 3U, 0, 255));
      return true;
    }
    case 0x0B: {
      length = 1;
      payload[0] = static_cast<uint8_t>(clampValue<uint16_t>(manifoldPressureKpa, 0, 255));
      return true;
    }
    case 0x0C: {
      length = 2;
      const uint16_t raw = static_cast<uint16_t>(engineRpm * 4U);
      payload[0] = static_cast<uint8_t>(raw >> 8);
      payload[1] = static_cast<uint8_t>(raw & 0xFFU);
      return true;
    }
    case 0x0D: {
      length = 1;
      payload[0] = vehicleSpeedKph;
      return true;
    }
    case 0x0E: {
      length = 1;
      payload[0] = static_cast<uint8_t>(128 + static_cast<int8_t>(4.0f * sinf(millis() / 3000.0f)));
      return true;
    }
    case 0x0F: {
      length = 1;
      payload[0] = static_cast<uint8_t>(intakeTempC + 40U);
      return true;
    }
    case 0x10: {
      length = 2;
      payload[0] = static_cast<uint8_t>(mafCentiGramsSec >> 8);
      payload[1] = static_cast<uint8_t>(mafCentiGramsSec & 0xFFU);
      return true;
    }
    case 0x11: {
      length = 1;
      payload[0] = percentToByte(throttlePercent);
      return true;
    }
    case 0x12: {
      length = 1;
      payload[0] = 0x01;
      return true;
    }
    case 0x13: {
      length = 1;
      payload[0] = 0x03;
      return true;
    }
    case 0x1C: {
      length = 1;
      payload[0] = 0x06;
      return true;
    }
    case 0x1F: {
      length = 2;
      payload[0] = static_cast<uint8_t>(engineRuntimeSec >> 8);
      payload[1] = static_cast<uint8_t>(engineRuntimeSec & 0xFFU);
      return true;
    }
    case 0x21: {
      length = 2;
      const uint16_t distance = static_cast<uint16_t>(clampValue<int32_t>(static_cast<int32_t>(distanceMilOnKm), 0, 65535));
      payload[0] = static_cast<uint8_t>(distance >> 8);
      payload[1] = static_cast<uint8_t>(distance & 0xFFU);
      return true;
    }
    case 0x22: {
      length = 2;
      const uint16_t raw = static_cast<uint16_t>(fuelRailPressureKpa / 0.079f);
      payload[0] = static_cast<uint8_t>(raw >> 8);
      payload[1] = static_cast<uint8_t>(raw & 0xFFU);
      return true;
    }
    case 0x23: {
      length = 2;
      const uint16_t raw = static_cast<uint16_t>(fuelRailPressureKpa / 10U);
      payload[0] = static_cast<uint8_t>(raw >> 8);
      payload[1] = static_cast<uint8_t>(raw & 0xFFU);
      return true;
    }
    case 0x2F: {
      length = 1;
      payload[0] = percentToByte(fuelLevelPercent);
      return true;
    }
    case 0x31: {
      length = 2;
      const uint16_t distance = static_cast<uint16_t>(clampValue<int32_t>(static_cast<int32_t>(distanceSinceClearKm), 0, 65535));
      payload[0] = static_cast<uint8_t>(distance >> 8);
      payload[1] = static_cast<uint8_t>(distance & 0xFFU);
      return true;
    }
    case 0x33: {
      length = 1;
      payload[0] = barometricPressureKpa;
      return true;
    }
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F: {
      length = 2;
      const uint16_t raw = static_cast<uint16_t>((850 + (20.0f * sinf(millis() / 4000.0f))) * 10.0f);
      payload[0] = static_cast<uint8_t>(raw >> 8);
      payload[1] = static_cast<uint8_t>(raw & 0xFFU);
      return true;
    }
    case 0x42: {
      length = 2;
      payload[0] = static_cast<uint8_t>(moduleVoltageMilliVolts >> 8);
      payload[1] = static_cast<uint8_t>(moduleVoltageMilliVolts & 0xFFU);
      return true;
    }
    case 0x43: {
      length = 2;
      const uint16_t absLoadRaw = static_cast<uint16_t>(clampValue<int32_t>(engineRpm * 3, 0, 65535));
      payload[0] = static_cast<uint8_t>(absLoadRaw >> 8);
      payload[1] = static_cast<uint8_t>(absLoadRaw & 0xFFU);
      return true;
    }
    case 0x44: {
      length = 2;
      payload[0] = 0x80;
      payload[1] = 0x00;
      return true;
    }
    case 0x45: {
      length = 1;
      payload[0] = percentToByte(relativeThrottlePercent);
      return true;
    }
    case 0x46: {
      length = 1;
      payload[0] = static_cast<uint8_t>(ambientTempC + 40U);
      return true;
    }
    case 0x47: {
      length = 1;
      payload[0] = percentToByte(throttlePercent);
      return true;
    }
    case 0x49:
    case 0x4A:
    case 0x4B:
    case 0x4C: {
      length = 1;
      payload[0] = percentToByte(accelPedalPercent);
      return true;
    }
    case 0x4D: {
      length = 2;
      payload[0] = static_cast<uint8_t>(milOnTimeMin >> 8);
      payload[1] = static_cast<uint8_t>(milOnTimeMin & 0xFFU);
      return true;
    }
    case 0x4E: {
      length = 2;
      const uint16_t mins = static_cast<uint16_t>((engineRuntimeSec / 60U) & 0xFFFFU);
      payload[0] = static_cast<uint8_t>(mins >> 8);
      payload[1] = static_cast<uint8_t>(mins & 0xFFU);
      return true;
    }
    case 0x51: {
      length = 1;
      payload[0] = 0x01;
      return true;
    }
    case 0x5C: {
      length = 1;
      payload[0] = static_cast<uint8_t>(oilTempC + 40U);
      return true;
    }
    case 0x5E: {
      length = 2;
      const float litersPerHour = 0.6f + (static_cast<float>(mafCentiGramsSec) / 400.0f);
      const uint16_t raw = static_cast<uint16_t>(litersPerHour * 20.0f);
      payload[0] = static_cast<uint8_t>(raw >> 8);
      payload[1] = static_cast<uint8_t>(raw & 0xFFU);
      return true;
    }
    default:
      length = inferMode01Length(pid);
      fillGenericMode01Payload(pid, payload, length);
      return true;
  }
}

void handleService03() {
  const uint8_t response[1] = {0x00};
  sendObdSingleFrame(0x43, response, sizeof(response));
}

void handleService04() {
  sendObdSingleFrame(0x44, nullptr, 0);
}

void handleService07() {
  const uint8_t response[1] = {0x00};
  sendObdSingleFrame(0x47, response, sizeof(response));
}

void handleService09(uint8_t pid) {
  if (pid == 0x00) {
    const uint8_t supportedInfoTypes[4] = {0x55, 0x00, 0x00, 0x00};
    uint8_t data[5] = {pid, supportedInfoTypes[0], supportedInfoTypes[1], supportedInfoTypes[2], supportedInfoTypes[3]};
    sendObdSingleFrame(0x49, data, sizeof(data));
    return;
  }

  if (pid == 0x02) {
    uint8_t data[1 + 1 + sizeof(kVin)] = {0};
    data[0] = pid;
    data[1] = 0x01;
    memcpy(&data[2], kVin, sizeof(kVin) - 1U);
    sendObdIsoTp(0x49, data, static_cast<uint16_t>(2U + sizeof(kVin) - 1U));
    return;
  }

  if (pid == 0x04) {
    uint8_t data[1 + 1 + sizeof(kCalId)] = {0};
    data[0] = pid;
    data[1] = 0x01;
    memcpy(&data[2], kCalId, sizeof(kCalId) - 1U);
    sendObdIsoTp(0x49, data, static_cast<uint16_t>(2U + sizeof(kCalId) - 1U));
    return;
  }

  if (pid == 0x0A) {
    const char *ecuName = getActiveEcuName();
    const uint8_t ecuNameLength = static_cast<uint8_t>(strlen(ecuName));
    uint8_t data[24] = {0};
    data[0] = pid;
    data[1] = 0x01;
    memcpy(&data[2], ecuName, ecuNameLength);
    sendObdIsoTp(0x49, data, static_cast<uint16_t>(2U + ecuNameLength));
    return;
  }

  sendObdNegativeResponse(0x09, 0x12);
}

void sendMode22Response(uint16_t did, const uint8_t *payload, uint8_t payloadLength) {
  uint8_t data[7] = {
      static_cast<uint8_t>(did >> 8),
      static_cast<uint8_t>(did & 0xFFU),
      0,
      0,
      0,
      0,
      0,
  };
  for (uint8_t i = 0; i < payloadLength && (i + 2U) < sizeof(data); ++i) {
    data[i + 2U] = payload[i];
  }
  sendObdSingleFrame(0x62, data, static_cast<uint8_t>(payloadLength + 2U));
}

void fillSnapshotPayload(uint8_t *payload, uint8_t length) {
  if (length < 8U) {
    return;
  }
  const uint16_t rpmRaw = static_cast<uint16_t>(engineRpm * 4U);
  payload[0] = static_cast<uint8_t>(rpmRaw >> 8);
  payload[1] = static_cast<uint8_t>(rpmRaw & 0xFFU);
  payload[2] = vehicleSpeedKph;
  payload[3] = percentToByte(throttlePercent);
  payload[4] = static_cast<uint8_t>(coolantTempC + 40U);
  payload[5] = static_cast<uint8_t>(intakeTempC + 40U);
  payload[6] = transmissionGear;
  payload[7] = static_cast<uint8_t>(moduleVoltageMilliVolts / 100U);
}

void handleService22(const struct can_frame &frame) {
  if (frame.can_dlc < 4) {
    sendObdNegativeResponse(0x22, 0x13);
    return;
  }

  const uint16_t did = static_cast<uint16_t>((frame.data[2] << 8) | frame.data[3]);

  if (activeDiagModule == DiagModule::TCM) {
    switch (did) {
      case 0xF130: {
        const uint8_t payload[1] = {transmissionGear};
        sendMode22Response(did, payload, sizeof(payload));
        return;
      }
      case 0xF18C: {
        const uint8_t payload[2] = {
            static_cast<uint8_t>(oilTempC + 40U),
            static_cast<uint8_t>(vehicleSpeedKph),
        };
        sendMode22Response(did, payload, sizeof(payload));
        return;
      }
      case 0xF190: {
        uint8_t data[2 + sizeof(kVin)] = {static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did & 0xFFU)};
        memcpy(&data[2], kVin, sizeof(kVin) - 1U);
        sendObdIsoTp(0x62, data, static_cast<uint16_t>(2U + sizeof(kVin) - 1U));
        return;
      }
      default:
        sendObdNegativeResponse(0x22, 0x31);
        return;
    }
  }

  if (activeDiagModule == DiagModule::ABS) {
    switch (did) {
      case 0xF121: {
        uint8_t payload[8] = {0};
        const uint16_t baseWheel = static_cast<uint16_t>(vehicleSpeedKph * 16U);
        encodeU16(static_cast<uint16_t>(baseWheel + 2U), &payload[0]);
        encodeU16(static_cast<uint16_t>(baseWheel + 1U), &payload[2]);
        encodeU16(static_cast<uint16_t>(baseWheel + 0U), &payload[4]);
        encodeU16(static_cast<uint16_t>(baseWheel + 1U), &payload[6]);
        uint8_t data[2 + sizeof(payload)] = {static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did & 0xFFU)};
        memcpy(&data[2], payload, sizeof(payload));
        sendObdIsoTp(0x62, data, static_cast<uint16_t>(2U + sizeof(payload)));
        return;
      }
      case 0xF122: {
        int16_t steeringRaw = static_cast<int16_t>(14.0f * sinf(millis() / 2200.0f) * 10.0f);
        uint8_t payload[2] = {0, 0};
        encodeI16(steeringRaw, payload);
        sendMode22Response(did, payload, sizeof(payload));
        return;
      }
      case 0xF123: {
        const uint8_t brakePct = static_cast<uint8_t>(clampValue<int32_t>(
            static_cast<int32_t>(7.0f + 7.0f * fabsf(sinf(millis() / 3000.0f))), 0, 100));
        const uint8_t payload[1] = {brakePct};
        sendMode22Response(did, payload, sizeof(payload));
        return;
      }
      default:
        sendObdNegativeResponse(0x22, 0x31);
        return;
    }
  }

  if (activeDiagModule == DiagModule::BCM) {
    switch (did) {
      case 0xF180: {
        const char *ecuName = getActiveEcuName();
        const uint8_t nameLength = static_cast<uint8_t>(strlen(ecuName));
        uint8_t data[24] = {static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did & 0xFFU)};
        memcpy(&data[2], ecuName, nameLength);
        sendObdIsoTp(0x62, data, static_cast<uint16_t>(2U + nameLength));
        return;
      }
      case 0xF181: {
        const uint8_t payload[3] = {
            static_cast<uint8_t>(ambientTempC + 40U),
            static_cast<uint8_t>(fuelLevelPercent),
            0x01,
        };
        sendMode22Response(did, payload, sizeof(payload));
        return;
      }
      default:
        sendObdNegativeResponse(0x22, 0x31);
        return;
    }
  }

  switch (did) {
    case 0xF40C: {
      const uint16_t raw = static_cast<uint16_t>(engineRpm * 4U);
      const uint8_t payload[2] = {
          static_cast<uint8_t>(raw >> 8),
          static_cast<uint8_t>(raw & 0xFFU),
      };
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    case 0xF40D: {
      const uint8_t payload[1] = {vehicleSpeedKph};
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    case 0xF405: {
      const uint8_t payload[1] = {static_cast<uint8_t>(coolantTempC + 40U)};
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    case 0xF411: {
      const uint8_t payload[1] = {percentToByte(throttlePercent)};
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    case 0xF442: {
      const uint8_t payload[2] = {
          static_cast<uint8_t>(moduleVoltageMilliVolts >> 8),
          static_cast<uint8_t>(moduleVoltageMilliVolts & 0xFFU),
      };
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    case 0xF45C: {
      const uint8_t payload[1] = {static_cast<uint8_t>(oilTempC + 40U)};
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    case 0xF130: {
      const uint8_t payload[1] = {transmissionGear};
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    case 0xF190: {
      uint8_t data[2 + sizeof(kVin)] = {static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did & 0xFFU)};
      memcpy(&data[2], kVin, sizeof(kVin) - 1U);
      sendObdIsoTp(0x62, data, static_cast<uint16_t>(2U + sizeof(kVin) - 1U));
      return;
    }
    case 0xF180: {
      const char *ecuName = getActiveEcuName();
      const uint8_t nameLength = static_cast<uint8_t>(strlen(ecuName));
      uint8_t data[24] = {static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did & 0xFFU)};
      memcpy(&data[2], ecuName, nameLength);
      sendObdIsoTp(0x62, data, static_cast<uint16_t>(2U + nameLength));
      return;
    }
    case 0xF181: {
      uint8_t payload[8] = {0};
      fillSnapshotPayload(payload, sizeof(payload));
      uint8_t data[2 + sizeof(payload)] = {static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did & 0xFFU)};
      memcpy(&data[2], payload, sizeof(payload));
      sendObdIsoTp(0x62, data, static_cast<uint16_t>(2U + sizeof(payload)));
      return;
    }
    case 0xF182: {
      const uint8_t payload[4] = {
          trimPercentToByte(shortTermFuelTrimBank1),
          trimPercentToByte(longTermFuelTrimBank1),
          trimPercentToByte(shortTermFuelTrimBank2),
          trimPercentToByte(longTermFuelTrimBank2),
      };
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    case 0xF187: {
      uint8_t data[18] = {static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did & 0xFFU)};
      const char serial[] = "ECUSN12026A01";
      memcpy(&data[2], serial, sizeof(serial) - 1U);
      sendObdIsoTp(0x62, data, static_cast<uint16_t>(2U + sizeof(serial) - 1U));
      return;
    }
    case 0xF188: {
      const uint8_t payload[4] = {
          static_cast<uint8_t>(warmupsSinceClear & 0xFFU),
          static_cast<uint8_t>(milOnTimeMin >> 8),
          static_cast<uint8_t>(milOnTimeMin & 0xFFU),
          static_cast<uint8_t>(fuelLevelPercent),
      };
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    case 0xF18A: {
      const uint8_t payload[2] = {
          static_cast<uint8_t>(barometricPressureKpa),
          static_cast<uint8_t>(ambientTempC + 40U),
      };
      sendMode22Response(did, payload, sizeof(payload));
      return;
    }
    default:
      sendObdNegativeResponse(0x22, 0x31);
      return;
  }
}

void updateVehicleState() {
  const unsigned long now = millis();
  if ((now - lastStateUpdateMs) < 50U) {
    return;
  }

  const float dtSeconds = static_cast<float>(now - lastStateUpdateMs) / 1000.0f;
  lastStateUpdateMs = now;

  const float t = now / 1000.0f;
  ignitionState = IgnitionState::RUNNING;

  const float speedWave = 58.0f + 34.0f * sinf(t * 0.11f) + 10.0f * sinf(t * 0.035f);
  vehicleSpeedKph = static_cast<uint8_t>(clampValue<int32_t>(static_cast<int32_t>(speedWave), 0, 180));

  const float throttleWave = 18.0f + 15.0f * sinf(t * 0.43f) + 7.0f * sinf(t * 1.31f);
  throttlePercent = static_cast<uint8_t>(clampValue<int32_t>(static_cast<int32_t>(throttleWave), 6, 80));
  accelPedalPercent = static_cast<uint8_t>(clampValue<int32_t>(throttlePercent + 3, 0, 100));
  relativeThrottlePercent = static_cast<uint8_t>(clampValue<int32_t>(throttlePercent - 2, 0, 100));
  commandedThrottlePercent = static_cast<uint8_t>(clampValue<int32_t>(throttlePercent, 0, 100));

  if (vehicleSpeedKph < 12) {
    transmissionGear = 1;
  } else if (vehicleSpeedKph < 30) {
    transmissionGear = 2;
  } else if (vehicleSpeedKph < 52) {
    transmissionGear = 3;
  } else if (vehicleSpeedKph < 78) {
    transmissionGear = 4;
  } else if (vehicleSpeedKph < 110) {
    transmissionGear = 5;
  } else {
    transmissionGear = 6;
  }

  const float gearFactor = 1.2f - (static_cast<float>(transmissionGear) * 0.12f);
  engineRpm = static_cast<uint16_t>(clampValue<int32_t>(
      static_cast<int32_t>(780.0f + vehicleSpeedKph * 42.0f * gearFactor + throttlePercent * 24.0f), 700, 6200));

  coolantTempC = static_cast<uint8_t>(clampValue<int32_t>(
      static_cast<int32_t>(72.0f + 20.0f * (1.0f - expf(-t / 220.0f)) + 2.0f * sinf(t * 0.04f)), 70, 104));
  oilTempC = static_cast<uint8_t>(clampValue<int32_t>(static_cast<int32_t>(coolantTempC + 6.0f), 75, 115));
  ambientTempC = static_cast<uint8_t>(clampValue<int32_t>(
      static_cast<int32_t>(26.0f + 3.0f * sinf(t * 0.005f)), 18, 42));
  intakeTempC = static_cast<uint8_t>(clampValue<int32_t>(
      static_cast<int32_t>(ambientTempC + 5.0f + 7.0f * sinf(t * 0.08f)), 20, 70));

  manifoldPressureKpa = static_cast<uint16_t>(clampValue<int32_t>(
      static_cast<int32_t>(22.0f + throttlePercent * 0.95f), 18, 140));
  fuelPressureKpa = static_cast<uint16_t>(clampValue<int32_t>(
      static_cast<int32_t>(320.0f + throttlePercent * 2.0f + 20.0f * sinf(t * 0.5f)), 280, 650));
  fuelRailPressureKpa = static_cast<uint16_t>(clampValue<int32_t>(
      static_cast<int32_t>(3800.0f + throttlePercent * 24.0f + 140.0f * sinf(t * 0.3f)), 3000, 8000));
  oilPressureKpa = static_cast<uint16_t>(clampValue<int32_t>(
      static_cast<int32_t>(160.0f + engineRpm * 0.12f), 120, 650));

  mafCentiGramsSec = static_cast<uint16_t>(clampValue<int32_t>(
      static_cast<int32_t>(220.0f + engineRpm * 0.32f + throttlePercent * 36.0f), 180, 65535));
  o2SensorVoltageRaw = static_cast<uint8_t>(clampValue<int32_t>(
      static_cast<int32_t>(128.0f + 30.0f * sinf(t * 3.2f)), 10, 245));

  shortTermFuelTrimBank1 = static_cast<int8_t>(clampValue<int32_t>(static_cast<int32_t>(5.0f * sinf(t * 0.9f)), -20, 20));
  longTermFuelTrimBank1 = static_cast<int8_t>(clampValue<int32_t>(static_cast<int32_t>(2.0f * sinf(t * 0.05f)), -10, 10));
  shortTermFuelTrimBank2 = static_cast<int8_t>(clampValue<int32_t>(shortTermFuelTrimBank1 - 1, -20, 20));
  longTermFuelTrimBank2 = static_cast<int8_t>(clampValue<int32_t>(longTermFuelTrimBank1 + 1, -10, 10));

  barometricPressureKpa = static_cast<uint8_t>(clampValue<int32_t>(
      static_cast<int32_t>(100.0f + 1.5f * sinf(t * 0.004f)), 95, 105));
  moduleVoltageMilliVolts = static_cast<uint16_t>(clampValue<int32_t>(
      static_cast<int32_t>(13800.0f + 180.0f * sinf(t * 0.7f)), 13000, 14600));

  if (lastDistanceUpdateMs == 0U) {
    lastDistanceUpdateMs = now;
  }
  const float distanceStepKm = (vehicleSpeedKph * dtSeconds) / 3600.0f;
  odometerKm += distanceStepKm;
  distanceSinceClearKm += distanceStepKm;
  if (milOnTimeMin > 0U) {
    distanceMilOnKm += distanceStepKm;
  }

  const uint32_t runSeconds = now / 1000U;
  engineRuntimeSec = static_cast<uint16_t>(runSeconds & 0xFFFFU);

  const float estimatedFuelUseLiters = distanceStepKm * (6.8f / 100.0f);
  const float tankLiters = 55.0f;
  const float levelDropPct = (estimatedFuelUseLiters / tankLiters) * 100.0f;
  const float rawFuel = static_cast<float>(fuelLevelPercent) - levelDropPct;
  fuelLevelPercent = static_cast<uint8_t>(clampValue<int32_t>(static_cast<int32_t>(rawFuel), 5, 100));

  if ((runSeconds % 1200U) == 0U && runSeconds != 0U) {
    warmupsSinceClear = static_cast<uint16_t>(clampValue<int32_t>(warmupsSinceClear + 1, 0, 255));
  }
}

void broadcastEngineFast() {
  const unsigned long now = millis();
  if ((now - lastEngineFastMs) < 20U) {
    return;
  }
  lastEngineFastMs = now;

  uint8_t frame[8] = {
      static_cast<uint8_t>(engineRpm >> 8),
      static_cast<uint8_t>(engineRpm & 0xFFU),
      vehicleSpeedKph,
      percentToByte(throttlePercent),
      static_cast<uint8_t>(coolantTempC + 40U),
      static_cast<uint8_t>(intakeTempC + 40U),
      transmissionGear,
      static_cast<uint8_t>(ignitionState),
  };
    stampCounterAndChecksum(CAN_ENGINE_FAST, frame, ctrEngineFast);
  sendCanFrame(CAN_ENGINE_FAST, frame, sizeof(frame));
}

void broadcastPowertrain() {
  const unsigned long now = millis();
  if ((now - lastPowertrainMs) < 50U) {
    return;
  }
  lastPowertrainMs = now;

  const uint16_t torqueNm = static_cast<uint16_t>(clampValue<int32_t>(
      static_cast<int32_t>(80 + (throttlePercent * 3) + (engineRpm / 40)), 80, 520));

  uint8_t frame[8] = {
      static_cast<uint8_t>(torqueNm >> 8),
      static_cast<uint8_t>(torqueNm & 0xFFU),
      static_cast<uint8_t>(manifoldPressureKpa),
      static_cast<uint8_t>(barometricPressureKpa),
      static_cast<uint8_t>(oilPressureKpa >> 8),
      static_cast<uint8_t>(oilPressureKpa & 0xFFU),
      static_cast<uint8_t>(fuelPressureKpa >> 8),
      static_cast<uint8_t>(fuelPressureKpa & 0xFFU),
  };
  stampCounterAndChecksum(CAN_POWERTRAIN, frame, ctrPowertrain);
  sendCanFrame(CAN_POWERTRAIN, frame, sizeof(frame));
}

void broadcastTransmission() {
  const unsigned long now = millis();
  if ((now - lastTransmissionMs) < 40U) {
    return;
  }
  lastTransmissionMs = now;

  const uint16_t slipRpm = static_cast<uint16_t>(clampValue<int32_t>(
      static_cast<int32_t>((engineRpm / 15) / transmissionGear), 0, 600));
  uint8_t frame[8] = {
      transmissionGear,
      static_cast<uint8_t>(engineRpm >> 8),
      static_cast<uint8_t>(engineRpm & 0xFFU),
      static_cast<uint8_t>(slipRpm >> 8),
      static_cast<uint8_t>(slipRpm & 0xFFU),
      static_cast<uint8_t>(percentToByte(throttlePercent)),
      static_cast<uint8_t>(vehicleSpeedKph),
      0x00,
  };
  stampCounterAndChecksum(CAN_TRANSMISSION, frame, ctrTransmission);
  sendCanFrame(CAN_TRANSMISSION, frame, sizeof(frame));
}

void broadcastAbsWheels() {
  const unsigned long now = millis();
  if ((now - lastAbsMs) < 20U) {
    return;
  }
  lastAbsMs = now;

  const uint16_t baseWheel = static_cast<uint16_t>(vehicleSpeedKph * 16U);
  const uint16_t fl = static_cast<uint16_t>(baseWheel + 2U);
  const uint16_t fr = static_cast<uint16_t>(baseWheel + 1U);
  const uint16_t rl = static_cast<uint16_t>(baseWheel + 0U);
  const uint16_t rr = static_cast<uint16_t>(baseWheel + 1U);
  uint8_t frame[8] = {
      static_cast<uint8_t>(fl >> 8),
      static_cast<uint8_t>(fl & 0xFFU),
      static_cast<uint8_t>(fr >> 8),
      static_cast<uint8_t>(fr & 0xFFU),
      static_cast<uint8_t>(rl >> 8),
      static_cast<uint8_t>(rl & 0xFFU),
      static_cast<uint8_t>(rr >> 8),
      static_cast<uint8_t>(rr & 0xFFU),
  };
  stampCounterAndChecksum(CAN_ABS_WHEELS, frame, ctrAbs);
  sendCanFrame(CAN_ABS_WHEELS, frame, sizeof(frame));
}

void broadcastBrakeSteer() {
  const unsigned long now = millis();
  if ((now - lastBrakeSteerMs) < 25U) {
    return;
  }
  lastBrakeSteerMs = now;

  const int16_t steeringDeg = static_cast<int16_t>(14.0f * sinf(millis() / 2200.0f));
  const uint8_t brakePct = static_cast<uint8_t>(clampValue<int32_t>(
      static_cast<int32_t>(7.0f + 7.0f * fabsf(sinf(millis() / 3000.0f))), 0, 100));

  uint8_t frame[8] = {
      static_cast<uint8_t>(steeringDeg >> 8),
      static_cast<uint8_t>(steeringDeg & 0xFFU),
      percentToByte(brakePct),
      static_cast<uint8_t>(vehicleSpeedKph),
      static_cast<uint8_t>(0x01),
      0x00,
      0x00,
      0x00,
  };
  stampCounterAndChecksum(CAN_BRAKE_STEER, frame, ctrBrakeSteer);
  sendCanFrame(CAN_BRAKE_STEER, frame, sizeof(frame));
}

void broadcastFuelEmissions() {
  const unsigned long now = millis();
  if ((now - lastFuelEmissionsMs) < 100U) {
    return;
  }
  lastFuelEmissionsMs = now;

  const uint16_t catalystC = static_cast<uint16_t>(850 + 20.0f * sinf(millis() / 4000.0f));
  uint8_t frame[8] = {
      static_cast<uint8_t>(fuelRailPressureKpa >> 8),
      static_cast<uint8_t>(fuelRailPressureKpa & 0xFFU),
      static_cast<uint8_t>(mafCentiGramsSec >> 8),
      static_cast<uint8_t>(mafCentiGramsSec & 0xFFU),
      o2SensorVoltageRaw,
      static_cast<uint8_t>(catalystC >> 8),
      static_cast<uint8_t>(catalystC & 0xFFU),
      percentToByte(fuelLevelPercent),
  };
  stampCounterAndChecksum(CAN_FUEL_EMISSIONS, frame, ctrFuelEmissions);
  sendCanFrame(CAN_FUEL_EMISSIONS, frame, sizeof(frame));
}

void broadcastBodyHvac() {
  const unsigned long now = millis();
  if ((now - lastBodyMs) < 200U) {
    return;
  }
  lastBodyMs = now;

  const uint8_t cabinSetpointC = 22;
  const uint8_t fanPercent = static_cast<uint8_t>(35 + 15 * fabsf(sinf(millis() / 5000.0f)));
  uint8_t frame[8] = {
      static_cast<uint8_t>(ambientTempC + 40U),
      static_cast<uint8_t>(intakeTempC + 40U),
      static_cast<uint8_t>(cabinSetpointC + 40U),
      percentToByte(fanPercent),
      0x01,
      0x01,
      0x00,
      0x00,
  };
  stampCounterAndChecksum(CAN_BODY_HVAC, frame, ctrBody);
  sendCanFrame(CAN_BODY_HVAC, frame, sizeof(frame));
}

void broadcastCharging() {
  const unsigned long now = millis();
  if ((now - lastChargingMs) < 250U) {
    return;
  }
  lastChargingMs = now;

  const int16_t currentA = static_cast<int16_t>(25 + 8.0f * sinf(millis() / 3500.0f));
  uint8_t frame[8] = {
      static_cast<uint8_t>(moduleVoltageMilliVolts >> 8),
      static_cast<uint8_t>(moduleVoltageMilliVolts & 0xFFU),
      static_cast<uint8_t>(currentA >> 8),
      static_cast<uint8_t>(currentA & 0xFFU),
      static_cast<uint8_t>(fuelLevelPercent),
      0x01,
      0x00,
      0x00,
  };
  stampCounterAndChecksum(CAN_CHARGING, frame, ctrCharging);
  sendCanFrame(CAN_CHARGING, frame, sizeof(frame));
}

void broadcastCruiseOdometer() {
  const unsigned long now = millis();
  if ((now - lastCruiseMs) < 100U) {
    return;
  }
  lastCruiseMs = now;

  const uint16_t odometerRaw = static_cast<uint16_t>(fmodf(odometerKm * 10.0f, 65535.0f));
  const uint8_t cruiseSetpoint = static_cast<uint8_t>(clampValue<int32_t>(static_cast<int32_t>(vehicleSpeedKph + 4), 30, 130));

  uint8_t frame[8] = {
      static_cast<uint8_t>(odometerRaw >> 8),
      static_cast<uint8_t>(odometerRaw & 0xFFU),
      vehicleSpeedKph,
      cruiseSetpoint,
      static_cast<uint8_t>(engineRuntimeSec >> 8),
      static_cast<uint8_t>(engineRuntimeSec & 0xFFU),
      0x01,
      0x00,
  };
  stampCounterAndChecksum(CAN_CRUISE_ODOMETER, frame, ctrCruise);
  sendCanFrame(CAN_CRUISE_ODOMETER, frame, sizeof(frame));
}

void broadcastTpms() {
  const unsigned long now = millis();
  if ((now - lastTpmsMs) < 500U) {
    return;
  }
  lastTpmsMs = now;

  const uint8_t psiFl = 35;
  const uint8_t psiFr = 35;
  const uint8_t psiRl = 34;
  const uint8_t psiRr = 34;
  const uint8_t tpmsTemp = static_cast<uint8_t>(ambientTempC + 4U + 40U);

  uint8_t frame[8] = {psiFl, psiFr, psiRl, psiRr, tpmsTemp, tpmsTemp, tpmsTemp, tpmsTemp};
  stampCounterAndChecksum(CAN_TPMS, frame, ctrTpms);
  sendCanFrame(CAN_TPMS, frame, sizeof(frame));
}

void handleObdRequest(const struct can_frame &frame) {
  if (frame.can_dlc < 2) {
    return;
  }

  const uint8_t service = frame.data[1];

  if (service == 0x01) {
    if (activeDiagModule != DiagModule::ECM) {
      sendObdNegativeResponse(service, 0x11);
      return;
    }
    if (frame.can_dlc < 3) {
      sendObdNegativeResponse(service, 0x13);
      return;
    }

    const uint8_t pid = frame.data[2];
    if (pid > MODE01_MAX_PID) {
      sendObdNegativeResponse(service, 0x12);
      return;
    }

    if ((pid % 0x20U) == 0U) {
      sendSupportedPidsRange(pid);
      return;
    }

    uint8_t payload[4] = {0, 0, 0, 0};
    uint8_t payloadLength = 0;
    if (buildMode01Payload(pid, payload, payloadLength)) {
      sendMode01Response(pid, payload, payloadLength);
    } else {
      sendObdNegativeResponse(service, 0x12);
    }
    return;
  }

  if (service == 0x03) {
    if (activeDiagModule != DiagModule::ECM) {
      sendObdNegativeResponse(service, 0x11);
      return;
    }
    handleService03();
    return;
  }

  if (service == 0x04) {
    if (activeDiagModule != DiagModule::ECM) {
      sendObdNegativeResponse(service, 0x11);
      return;
    }
    handleService04();
    return;
  }

  if (service == 0x07) {
    if (activeDiagModule != DiagModule::ECM) {
      sendObdNegativeResponse(service, 0x11);
      return;
    }
    handleService07();
    return;
  }

  if (service == 0x09) {
    if (activeDiagModule != DiagModule::ECM && activeDiagModule != DiagModule::TCM) {
      sendObdNegativeResponse(service, 0x11);
      return;
    }
    if (frame.can_dlc < 3) {
      sendObdNegativeResponse(service, 0x13);
      return;
    }
    handleService09(frame.data[2]);
    return;
  }

  if (service == 0x22) {
    handleService22(frame);
    return;
  }

  sendObdNegativeResponse(service, 0x11);
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== CAN ECU Simulator (Running Vehicle Profile) ===");
  Serial.println("ESP32 + MCP2515");
  Serial.printf("Profile: %s\n", kVehicleProfile);
  Serial.println("Features:");
  Serial.println("  - Continuous running-engine vehicle dynamics");
  Serial.println("  - Broad CAN broadcast set (powertrain, ABS, body, TPMS, charging)");
  Serial.println("  - OBD-II services 01/03/04/07/09 + OEM-style service 22 DIDs");
  Serial.println("  - Multi-frame ISO-TP support for VIN/CALID/ECU name responses");
  Serial.println("");

  pinMode(CAN_INT_PIN, INPUT_PULLUP);
  SPI.begin(CAN_SCK_PIN, CAN_MISO_PIN, CAN_MOSI_PIN, CAN_CS_PIN);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP2515_CLOCK);
  mcp2515.setNormalMode();

  Serial.println("MCP2515 initialized at 500 kbps");
}

void loop() {
  updateVehicleState();

  broadcastEngineFast();
  broadcastPowertrain();
  broadcastTransmission();
  broadcastAbsWheels();
  broadcastBrakeSteer();
  broadcastFuelEmissions();
  broadcastBodyHvac();
  broadcastCharging();
  broadcastCruiseOdometer();
  broadcastTpms();

  struct can_frame frame {};
  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    const uint16_t canId = static_cast<uint16_t>(frame.can_id & 0x7FFU);
    if (canId == OBD2_FUNCTIONAL_REQUEST || canId == OBD2_TESTER_REQUEST_ECM || canId == OBD2_TESTER_REQUEST_TCM ||
        canId == OBD2_TESTER_REQUEST_ABS || canId == OBD2_TESTER_REQUEST_BCM) {
      setDiagnosticContext(canId);
      handleObdRequest(frame);
    }
  }
}