#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <cmath>

namespace {
constexpr int CAN_INT_PIN = 4;
constexpr int CAN_SCK_PIN = 18;
constexpr int CAN_MISO_PIN = 19;
constexpr int CAN_MOSI_PIN = 23;
constexpr int CAN_CS_PIN = 5;

constexpr uint16_t OBD2_FUNCTIONAL_REQUEST = 0x7DF;
constexpr uint16_t OBD2_TESTER_REQUEST = 0x7E0;
constexpr uint16_t OBD2_ECU_RESPONSE = 0x7E8;

// Typical automotive CAN IDs (example vehicle).
constexpr uint16_t CAN_ENGINE_STATUS = 0x100;
constexpr uint16_t CAN_TRANSMISSION = 0x101;
constexpr uint16_t CAN_FUEL_SYSTEM = 0x102;
constexpr uint16_t CAN_EMISSIONS = 0x103;

// Change this to MCP_16MHZ if your MCP2515 module uses a 16 MHz crystal.
constexpr auto MCP2515_CLOCK = MCP_8MHZ;

enum class IgnitionState : uint8_t {
  OFF = 0,
  ACCESSORY = 1,
  ON = 2,
  CRANK = 3,
  RUNNING = 4,
};

MCP2515 mcp2515(CAN_CS_PIN);

// Vehicle state variables.
IgnitionState ignitionState = IgnitionState::OFF;
uint16_t engineRpm = 0;
uint8_t vehicleSpeed = 0;
uint8_t coolantTempC = 20;
uint8_t intakeTempC = 25;
uint8_t throttlePercent = 0;
uint16_t fuelPressureKpa = 0;
uint8_t mafGramsSec = 0;
uint8_t o2SensorVoltage = 0;
uint8_t transmissionGear = 0;  // 0=Park, 1=Reverse, 2=Neutral, 3=Drive
uint16_t oilPressureKpa = 0;
uint8_t batteryVoltageDecimal = 0;  // In 0.1V units (e.g., 130 = 13.0V)

unsigned long lastStateUpdateMs = 0;
unsigned long lastBroadcastMs = 0;
unsigned long lastEngineStatusMs = 0;
unsigned long lastTransmissionMs = 0;
unsigned long lastFuelSystemMs = 0;
unsigned long lastEmissionsMs = 0;

void sendCanFrame(uint32_t id, const uint8_t *data, uint8_t length) {
  struct can_frame frame {};
  frame.can_id = id;
  frame.can_dlc = length;
  for (uint8_t index = 0; index < length; ++index) {
    frame.data[index] = data[index];
  }
  mcp2515.sendMessage(&frame);
}

void sendObdResponse(uint8_t pid, const uint8_t *payload, uint8_t payloadLength) {
  uint8_t response[8] = {static_cast<uint8_t>(payloadLength + 2), 0x41, pid, 0, 0, 0, 0, 0};
  for (uint8_t index = 0; index < payloadLength && (index + 3) < 8; ++index) {
    response[index + 3] = payload[index];
  }
  sendCanFrame(OBD2_ECU_RESPONSE, response, 8);
}

void updateFakeVehicleState() {
  const unsigned long now = millis();
  if (now - lastStateUpdateMs < 100) {
    return;
  }

  lastStateUpdateMs = now;
  const float seconds = now / 1000.0f;

  // Simulate ignition sequence: OFF -> ACCESSORY -> ON -> CRANK -> RUNNING -> OFF.
  // Cycle every 30 seconds for demo purposes.
  const float cyclePhase = fmod(seconds / 30.0f, 1.0f);
  IgnitionState newState = ignitionState;

  if (cyclePhase < 0.1f) {
    newState = IgnitionState::OFF;
  } else if (cyclePhase < 0.2f) {
    newState = IgnitionState::ACCESSORY;
  } else if (cyclePhase < 0.3f) {
    newState = IgnitionState::ON;
  } else if (cyclePhase < 0.4f) {
    newState = IgnitionState::CRANK;
  } else {
    newState = IgnitionState::RUNNING;
  }

  // Log state transitions.
  if (newState != ignitionState) {
    ignitionState = newState;
    const char *stateNames[] = {"OFF", "ACCESSORY", "ON", "CRANK", "RUNNING"};
    Serial.printf("[%lu] Ignition -> %s\n", millis(), stateNames[static_cast<uint8_t>(ignitionState)]);
  } else {
    ignitionState = newState;
  }

  // Update state based on ignition.
  if (ignitionState == IgnitionState::OFF) {
    engineRpm = 0;
    vehicleSpeed = 0;
    throttlePercent = 0;
    coolantTempC = 20;
    mafGramsSec = 0;
    o2SensorVoltage = 0;
    fuelPressureKpa = 0;
    oilPressureKpa = 0;
    batteryVoltageDecimal = 130;  // 13.0V off engine
  } else if (ignitionState == IgnitionState::ACCESSORY) {
    engineRpm = 0;
    vehicleSpeed = 0;
    throttlePercent = 0;
    coolantTempC = 25;
    mafGramsSec = 0;
    o2SensorVoltage = 0;
    fuelPressureKpa = 300;  // Fuel pump on.
    oilPressureKpa = 0;
    batteryVoltageDecimal = 132;  // 13.2V with alternator.
  } else if (ignitionState == IgnitionState::ON) {
    engineRpm = 0;
    vehicleSpeed = 0;
    throttlePercent = 0;
    coolantTempC = 30;
    mafGramsSec = 0;
    fuelPressureKpa = 350;
    oilPressureKpa = 0;
    batteryVoltageDecimal = 133;
  } else if (ignitionState == IgnitionState::CRANK) {
    // Cranking.
    engineRpm = 150;
    vehicleSpeed = 0;
    throttlePercent = 0;
    coolantTempC = 35;
    mafGramsSec = 10;
    fuelPressureKpa = 380;
    oilPressureKpa = 50;
    batteryVoltageDecimal = 125;  // Voltage drop during crank.
  } else {
    // Engine running: simulate light acceleration cycle.
    const float throttleWave = 0.3f + 0.5f * sinf(seconds * 0.6f);
    throttlePercent = static_cast<uint8_t>(10 + throttleWave * 60.0f);
    engineRpm = static_cast<uint16_t>(800 + throttleWave * 4000.0f);
    vehicleSpeed = static_cast<uint8_t>(throttleWave * 120.0f);
    mafGramsSec = static_cast<uint8_t>(5 + throttleWave * 90.0f);
    o2SensorVoltage = static_cast<uint8_t>(400 + sinf(seconds * 0.5f) * 100.0f);
    coolantTempC = static_cast<uint8_t>(88 + 8.0f * sinf(seconds * 0.15f));
    intakeTempC = static_cast<uint8_t>(45 + 10.0f * sinf(seconds * 0.3f));
    fuelPressureKpa = static_cast<uint16_t>(380 + throttleWave * 80.0f);
    oilPressureKpa = static_cast<uint16_t>(300 + throttleWave * 400.0f);
    batteryVoltageDecimal = 135;  // 13.5V during running.

    // Transmission simulation: gear up and down.
    const float gearPhase = std::fmod(seconds / 8.0f, 1.0f);
    if (gearPhase < 0.1f) {
      transmissionGear = 3;  // Drive.
    } else if (gearPhase < 0.5f && vehicleSpeed > 30) {
      transmissionGear = 4;  // Higher gear if moving.
    } else if (vehicleSpeed < 5) {
      transmissionGear = 3;
    }
  }
}

void broadcastEngineStatus() {
  const unsigned long now = millis();
  if (now - lastEngineStatusMs < 100) {
    return;
  }
  lastEngineStatusMs = now;

  uint8_t frame[8] = {
      static_cast<uint8_t>(engineRpm >> 8),
      static_cast<uint8_t>(engineRpm & 0xFF),
      vehicleSpeed,
      static_cast<uint8_t>(coolantTempC + 40),
      static_cast<uint8_t>(intakeTempC + 40),
      throttlePercent,
      static_cast<uint8_t>(ignitionState),
      0x00,
  };
  sendCanFrame(CAN_ENGINE_STATUS, frame, sizeof(frame));
}

void broadcastTransmission() {
  const unsigned long now = millis();
  if (now - lastTransmissionMs < 250) {
    return;
  }
  lastTransmissionMs = now;

  uint8_t frame[8] = {
      transmissionGear,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
  };
  sendCanFrame(CAN_TRANSMISSION, frame, sizeof(frame));
}

void broadcastFuelSystem() {
  const unsigned long now = millis();
  if (now - lastFuelSystemMs < 200) {
    return;
  }
  lastFuelSystemMs = now;

  uint8_t frame[8] = {
      static_cast<uint8_t>(fuelPressureKpa >> 8),
      static_cast<uint8_t>(fuelPressureKpa & 0xFF),
      mafGramsSec,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
  };
  sendCanFrame(CAN_FUEL_SYSTEM, frame, sizeof(frame));
}

void broadcastEmissions() {
  const unsigned long now = millis();
  if (now - lastEmissionsMs < 300) {
    return;
  }
  lastEmissionsMs = now;

  uint8_t frame[8] = {
      o2SensorVoltage,
      static_cast<uint8_t>(oilPressureKpa >> 8),
      static_cast<uint8_t>(oilPressureKpa & 0xFF),
      batteryVoltageDecimal,
      0x00,
      0x00,
      0x00,
      0x00,
  };
  sendCanFrame(CAN_EMISSIONS, frame, sizeof(frame));
}

void handleObdRequest(const struct can_frame &frame) {
  if (frame.can_dlc < 3) {
    return;
  }

  const uint8_t service = frame.data[1];
  const uint8_t pid = frame.data[2];

  if (service != 0x01) {
    return;
  }

  switch (pid) {
    case 0x00: {  // PIDs supported [01-20].
      const uint8_t payload[4] = {0xBE, 0x1F, 0xB8, 0x11};
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x05: {  // Coolant temperature.
      const uint8_t payload[1] = {static_cast<uint8_t>(coolantTempC + 40)};
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x0C: {  // Engine RPM.
      const uint16_t raw = static_cast<uint16_t>(engineRpm * 4U);
      const uint8_t payload[2] = {
          static_cast<uint8_t>(raw >> 8),
          static_cast<uint8_t>(raw & 0xFF),
      };
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x0D: {  // Vehicle speed.
      const uint8_t payload[1] = {vehicleSpeed};
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x0F: {  // Intake air temperature.
      const uint8_t payload[1] = {static_cast<uint8_t>(intakeTempC + 40)};
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x10: {  // MAF air flow rate.
      const uint16_t raw = static_cast<uint16_t>(mafGramsSec * 256U);
      const uint8_t payload[2] = {
          static_cast<uint8_t>(raw >> 8),
          static_cast<uint8_t>(raw & 0xFF),
      };
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x11: {  // Throttle position.
      const uint8_t payload[1] = {static_cast<uint8_t>(throttlePercent * 255U / 100U)};
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x13: {  // O2 sensor 1 voltage.
      const uint8_t payload[2] = {o2SensorVoltage, 0x00};
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x1C: {  // OBD standard this vehicle conforms to.
      const uint8_t payload[1] = {0x06};  // OBD-II (ISO 14230-4, KWP2000).
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x20: {  // PIDs supported [21-40].
      const uint8_t payload[4] = {0x80, 0x00, 0x00, 0x00};
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x2F: {  // Fuel level input.
      const uint8_t payload[1] = {static_cast<uint8_t>(750 + sinf(millis() / 10000.0f) * 250.0f)};
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x42: {  // Control module voltage (battery).
      const uint16_t raw = static_cast<uint16_t>(batteryVoltageDecimal * 100U);
      const uint8_t payload[2] = {
          static_cast<uint8_t>(raw >> 8),
          static_cast<uint8_t>(raw & 0xFF),
      };
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    case 0x43: {  // Engine load.
      const uint8_t payload[1] = {static_cast<uint8_t>(throttlePercent * 255U / 100U * 0.8f)};
      sendObdResponse(pid, payload, sizeof(payload));
      break;
    }
    default:
      break;
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== CAN ECU Simulator ===");
  Serial.println("ESP32 + MCP2515 Automotive Bench Test");
  Serial.println("Features:");
  Serial.println("  - Ignition state machine (OFF->ACCESSORY->ON->CRANK->RUNNING)");
  Serial.println("  - Multiple OBD2 PIDs (0x05, 0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x1C, 0x2F, 0x42, 0x43)");
  Serial.println("  - Typical CAN signals (engine, transmission, fuel, emissions)");
  Serial.println("  - Realistic temperature and pressure simulation");
  Serial.println("");

  pinMode(CAN_INT_PIN, INPUT_PULLUP);
  SPI.begin(CAN_SCK_PIN, CAN_MISO_PIN, CAN_MOSI_PIN, CAN_CS_PIN);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP2515_CLOCK);
  mcp2515.setNormalMode();

  Serial.println("MCP2515 initialized at 500 kbps");
  Serial.println("Cycle time: 30s (OFF 0-3s, ACC 3-6s, ON 6-9s, CRANK 9-12s, RUNNING 12-30s)");
}

void loop() {
  updateFakeVehicleState();
  broadcastEngineStatus();
  broadcastTransmission();
  broadcastFuelSystem();
  broadcastEmissions();

  struct can_frame frame {};
  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    const uint32_t canId = frame.can_id & 0x7FF;
    if (canId == OBD2_FUNCTIONAL_REQUEST || canId == OBD2_TESTER_REQUEST) {
      handleObdRequest(frame);
    }
  }
}