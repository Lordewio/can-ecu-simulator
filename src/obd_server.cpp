#include "obd_server.h"

#include <Arduino.h>
#include <string.h>

#include "obd_pids.h"
#include "vehicle_params.h"

namespace obd {

const ModuleInfo kModules[kModuleCount] = {
    {0x7E0, 0x7E8, "SIM-ECM-ESP32"},
    {0x7E1, 0x7E9, "SIM-TCM-ESP32"},
    {0x7E2, 0x7EA, "SIM-ABS-ESP32"},
    {0x7E3, 0x7EB, "SIM-BCM-ESP32"},
};

namespace {

constexpr char kVin[] = "JTDBU4EE9BJ104521";  // 17 characters, Corolla-format
constexpr char kCalId[] = "3441120A";
constexpr char kCvn[] = "\x11\x22\x33\x44";
constexpr uint32_t kFlowControlTimeoutMs = 1000;
constexpr uint32_t kRxTimeoutMs = 1000;

// Negative response codes.
constexpr uint8_t kNrcServiceNotSupported = 0x11;
constexpr uint8_t kNrcSubFunctionNotSupported = 0x12;
constexpr uint8_t kNrcIncorrectLength = 0x13;
constexpr uint8_t kNrcRequestOutOfRange = 0x31;

uint8_t hexDigit(uint8_t nibble) {
  return (nibble < 10) ? static_cast<uint8_t>('0' + nibble) : static_cast<uint8_t>('A' + nibble - 10);
}

int8_t hexValue(char c) {
  if (c >= '0' && c <= '9') return static_cast<int8_t>(c - '0');
  if (c >= 'A' && c <= 'F') return static_cast<int8_t>(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return static_cast<int8_t>(c - 'a' + 10);
  return -1;
}

uint8_t percentByte(float percent) {
  float scaled = percent * 255.0f / 100.0f;
  if (scaled < 0.0f) scaled = 0.0f;
  if (scaled > 255.0f) scaled = 255.0f;
  return static_cast<uint8_t>(scaled + 0.5f);
}

uint8_t tempByte(float celsius) {
  float v = celsius + 40.0f;
  if (v < 0.0f) v = 0.0f;
  if (v > 255.0f) v = 255.0f;
  return static_cast<uint8_t>(v + 0.5f);
}

uint8_t trimByte(float trimPercent) {
  float v = (trimPercent + 100.0f) * 128.0f / 100.0f;
  if (v < 0.0f) v = 0.0f;
  if (v > 255.0f) v = 255.0f;
  return static_cast<uint8_t>(v + 0.5f);
}

void put16(uint8_t *out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value >> 8);
  out[1] = static_cast<uint8_t>(value & 0xFF);
}

// STmin is milliseconds for 0x00-0x7F and 100-900 microseconds for 0xF1-0xF9.
uint32_t decodeStMin(uint8_t raw) {
  if (raw <= 0x7F) return raw;
  if (raw >= 0xF1 && raw <= 0xF9) return 1;  // sub-millisecond, round up
  return 10;                                 // reserved value: be conservative
}

}  // namespace

uint16_t encodeDtc(const char *text) {
  if (text == nullptr || strlen(text) < 5) return 0;
  uint16_t system = 0;
  switch (text[0]) {
    case 'P': case 'p': system = 0; break;
    case 'C': case 'c': system = 1; break;
    case 'B': case 'b': system = 2; break;
    case 'U': case 'u': system = 3; break;
    default: return 0;
  }
  uint16_t code = static_cast<uint16_t>(system << 14);
  for (int i = 1; i <= 4; ++i) {
    const int8_t v = hexValue(text[i]);
    if (v < 0) return 0;
    code = static_cast<uint16_t>(code | (static_cast<uint16_t>(v) << ((4 - i) * 4)));
  }
  // The first digit after the letter is only two bits wide.
  return static_cast<uint16_t>(code & 0x3FFF) | static_cast<uint16_t>(system << 14);
}

void decodeDtc(uint16_t code, char *out5) {
  static const char kSystems[] = "PCBU";
  out5[0] = kSystems[(code >> 14) & 0x03];
  out5[1] = static_cast<char>('0' + ((code >> 12) & 0x03));
  out5[2] = static_cast<char>(hexDigit((code >> 8) & 0x0F));
  out5[3] = static_cast<char>(hexDigit((code >> 4) & 0x0F));
  out5[4] = static_cast<char>(hexDigit(code & 0x0F));
  out5[5] = '\0';
}

void Server::begin(MCP2515 *can, vehicle::Model *model) {
  can_ = can;
  model_ = model;
  for (int i = 0; i < kMaxDtcs; ++i) dtcs_[i] = Dtc();
  for (int i = 0; i < kModuleCount; ++i) queue_[i] = QueuedResponse();
  tx_ = TxState();
  rx_ = RxState();
  freeze_ = FreezeFrame();
}

// ---------------------------------------------------------------- DTC store --

bool Server::setDtc(uint16_t code, Module module, bool confirmed) {
  if (code == 0) return false;
  for (int i = 0; i < kMaxDtcs; ++i) {
    if (dtcs_[i].used && dtcs_[i].code == code) {
      dtcs_[i].confirmed = dtcs_[i].confirmed || confirmed;
      dtcs_[i].pending = !confirmed;
      return true;
    }
  }
  for (int i = 0; i < kMaxDtcs; ++i) {
    if (dtcs_[i].used) continue;
    dtcs_[i].used = true;
    dtcs_[i].code = code;
    dtcs_[i].module = module;
    dtcs_[i].confirmed = confirmed;
    dtcs_[i].pending = !confirmed;
    // A confirmed emissions fault becomes permanent and captures a freeze
    // frame, exactly as it does on the real ECU.
    dtcs_[i].permanent = confirmed && ((code >> 14) & 0x03) == 0;

    if (confirmed && !freeze_.valid && model_ != nullptr) {
      const vehicle::State &s = model_->state();
      freeze_.valid = true;
      freeze_.dtcCode = code;
      freeze_.rpm = static_cast<uint16_t>(s.engineRpm);
      freeze_.speedKph = static_cast<uint8_t>(s.speedKph);
      freeze_.loadPercent = static_cast<uint8_t>(s.engineLoad * 100.0f);
      freeze_.coolantPlus40 = tempByte(s.coolantTempC);
      freeze_.intakePlus40 = tempByte(s.intakeTempC);
      freeze_.throttlePercent = static_cast<uint8_t>(s.throttlePercent);
      freeze_.mapKpa = static_cast<uint16_t>(s.mapKpa);
      freeze_.shortTrim = static_cast<int8_t>(s.shortTrimBank1);
      freeze_.longTrim = static_cast<int8_t>(s.longTrimBank1);
      freeze_.fuelSystemStatus = s.closedLoop ? 0x02 : 0x01;
    }
    return true;
  }
  return false;
}

bool Server::clearDtc(uint16_t code) {
  for (int i = 0; i < kMaxDtcs; ++i) {
    if (dtcs_[i].used && dtcs_[i].code == code) {
      dtcs_[i] = Dtc();
      return true;
    }
  }
  return false;
}

void Server::clearAll() {
  for (int i = 0; i < kMaxDtcs; ++i) {
    // Permanent codes survive a clear until the monitor runs and passes.
    if (dtcs_[i].used && dtcs_[i].permanent) {
      dtcs_[i].confirmed = false;
      dtcs_[i].pending = false;
      continue;
    }
    dtcs_[i] = Dtc();
  }
  freeze_ = FreezeFrame();
  if (model_ != nullptr) model_->clearAdaptations();
}

uint8_t Server::confirmedCount() const {
  uint8_t n = 0;
  for (int i = 0; i < kMaxDtcs; ++i) {
    if (dtcs_[i].used && dtcs_[i].confirmed) n++;
  }
  return n;
}

uint8_t Server::pendingCount() const {
  uint8_t n = 0;
  for (int i = 0; i < kMaxDtcs; ++i) {
    if (dtcs_[i].used && dtcs_[i].pending) n++;
  }
  return n;
}

bool Server::milOn() const { return confirmedCount() > 0; }

// ------------------------------------------------------------- ISO-TP layer --

// A fresh request supersedes anything still queued or in flight for that
// module. Without this, a response the tester abandoned (by never sending
// flow control) would go on to swallow the flow control meant for the next
// one, and every later answer would be off by a transfer.
void Server::cancelTransfers(int moduleIndex) {
  for (int i = 0; i < kModuleCount; ++i) {
    if (moduleIndex >= 0 && i != moduleIndex) continue;
    if (queue_[i].used && queue_[i].responseId == kModules[i].responseId) {
      queue_[i] = QueuedResponse();
    }
  }
  if (tx_.phase == TxPhase::IDLE) return;
  if (moduleIndex < 0) {
    tx_ = TxState();
    return;
  }
  if (tx_.responseId == kModules[moduleIndex].responseId) tx_ = TxState();
}

void Server::sendRaw(uint16_t id, const uint8_t *data, uint8_t dlc) {
  if (can_ == nullptr) return;
  struct can_frame frame {};
  frame.can_id = id;
  frame.can_dlc = dlc;
  for (uint8_t i = 0; i < dlc && i < 8; ++i) frame.data[i] = data[i];
  can_->sendMessage(&frame);
}

void Server::sendFlowControl(uint16_t responseId) {
  // Clear to send, no block limit, 0 ms separation.
  const uint8_t fc[8] = {0x30, 0x00, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55};
  sendRaw(responseId, fc, 8);
}

void Server::respond(Module module, const uint8_t *payload, uint16_t length) {
  if (length == 0 || length > kMaxPayload) return;
  const uint16_t responseId = kModules[static_cast<int>(module)].responseId;

  // A response that fits in a single frame goes out immediately; there is no
  // handshake for those and queueing them would only add latency.
  if (length <= 7) {
    uint8_t frame[8] = {static_cast<uint8_t>(length), 0, 0, 0, 0, 0, 0, 0};
    for (uint16_t i = 0; i < length; ++i) frame[1 + i] = payload[i];
    // Pad with 0x55 the way most real ECUs do, rather than zeros.
    for (uint16_t i = length + 1; i < 8; ++i) frame[i] = 0x55;
    sendRaw(responseId, frame, 8);
    return;
  }

  for (int i = 0; i < kModuleCount; ++i) {
    if (queue_[i].used) continue;
    queue_[i].used = true;
    queue_[i].responseId = responseId;
    queue_[i].length = length;
    memcpy(queue_[i].payload, payload, length);
    return;
  }
  // Queue full: the tester will time out, which is the honest failure mode.
}

void Server::negative(Module module, uint8_t service, uint8_t nrc) {
  const uint8_t payload[3] = {0x7F, service, nrc};
  respond(module, payload, sizeof(payload));
}

void Server::pump() {
  const uint32_t now = millis();

  // Abandon a stalled receive assembly.
  if (rx_.active && static_cast<int32_t>(now - rx_.timeoutMs) > 0) {
    rx_ = RxState();
  }

  switch (tx_.phase) {
    case TxPhase::IDLE: {
      for (int i = 0; i < kModuleCount; ++i) {
        if (!queue_[i].used) continue;
        tx_ = TxState();
        tx_.responseId = queue_[i].responseId;
        tx_.length = queue_[i].length;
        memcpy(tx_.payload, queue_[i].payload, queue_[i].length);
        queue_[i] = QueuedResponse();

        // First frame carries the total length and the first six bytes.
        uint8_t ff[8];
        ff[0] = static_cast<uint8_t>(0x10 | ((tx_.length >> 8) & 0x0F));
        ff[1] = static_cast<uint8_t>(tx_.length & 0xFF);
        for (uint8_t b = 0; b < 6; ++b) {
          ff[2 + b] = (b < tx_.length) ? tx_.payload[b] : 0x55;
        }
        sendRaw(tx_.responseId, ff, 8);
        tx_.index = (tx_.length < 6) ? tx_.length : 6;
        tx_.sequence = 1;
        tx_.phase = TxPhase::WAIT_FC;
        tx_.timeoutMs = now + kFlowControlTimeoutMs;
        return;
      }
      break;
    }

    case TxPhase::WAIT_FC: {
      // The tester owns the pace here. If it never answers, give up rather
      // than flooding the bus with frames nobody asked for.
      if (static_cast<int32_t>(now - tx_.timeoutMs) > 0) tx_ = TxState();
      break;
    }

    case TxPhase::SENDING: {
      if (static_cast<int32_t>(now - tx_.nextSendMs) < 0) break;

      uint8_t cf[8];
      cf[0] = static_cast<uint8_t>(0x20 | (tx_.sequence & 0x0F));
      for (uint8_t b = 0; b < 7; ++b) {
        cf[1 + b] = (tx_.index < tx_.length) ? tx_.payload[tx_.index++] : 0x55;
      }
      sendRaw(tx_.responseId, cf, 8);

      tx_.sequence = static_cast<uint8_t>((tx_.sequence + 1) & 0x0F);
      tx_.nextSendMs = now + tx_.stMinMs;

      if (tx_.index >= tx_.length) {
        tx_ = TxState();
        break;
      }
      // Honour the tester's block size: after BS frames, wait for another FC.
      if (tx_.blockSize != 0) {
        if (--tx_.blockRemaining == 0) {
          tx_.phase = TxPhase::WAIT_FC;
          tx_.timeoutMs = now + kFlowControlTimeoutMs;
        }
      }
      break;
    }
  }
}

void Server::handleFrame(const struct can_frame &frame) {
  const uint16_t id = static_cast<uint16_t>(frame.can_id & 0x7FF);
  if (frame.can_dlc == 0) return;

  bool functional = (id == kFunctionalRequestId);
  int targetModule = -1;
  if (!functional) {
    for (int i = 0; i < kModuleCount; ++i) {
      if (kModules[i].requestId == id) {
        targetModule = i;
        break;
      }
    }
    if (targetModule < 0) return;  // not addressed to us
  }

  const uint8_t pci = static_cast<uint8_t>(frame.data[0] & 0xF0);

  // Flow control from the tester, steering an in-flight segmented response.
  if (pci == 0x30) {
    if (tx_.phase != TxPhase::WAIT_FC) return;
    const uint8_t flowStatus = frame.data[0] & 0x0F;
    if (flowStatus == 0x01) {  // wait
      tx_.timeoutMs = millis() + kFlowControlTimeoutMs;
      return;
    }
    if (flowStatus == 0x02) {  // overflow: the tester cannot take the response
      tx_ = TxState();
      return;
    }
    tx_.blockSize = (frame.can_dlc > 1) ? frame.data[1] : 0;
    tx_.blockRemaining = tx_.blockSize;
    tx_.stMinMs = decodeStMin((frame.can_dlc > 2) ? frame.data[2] : 0);
    tx_.phase = TxPhase::SENDING;
    tx_.nextSendMs = millis();
    return;
  }

  if (pci == 0x00) {  // single frame
    const uint8_t length = frame.data[0] & 0x0F;
    if (length == 0 || length > 7 || length > frame.can_dlc - 1) return;
    cancelTransfers(functional ? -1 : targetModule);
    if (functional) {
      for (int i = 0; i < kModuleCount; ++i) {
        dispatch(static_cast<Module>(i), true, &frame.data[1], length);
      }
    } else {
      dispatch(static_cast<Module>(targetModule), false, &frame.data[1], length);
    }
    return;
  }

  if (pci == 0x10) {  // first frame of a segmented request
    if (functional) return;  // segmented functional requests are not a thing
    const uint16_t total = static_cast<uint16_t>(((frame.data[0] & 0x0F) << 8) | frame.data[1]);
    if (total < 8 || total > kMaxPayload) return;
    cancelTransfers(targetModule);
    rx_ = RxState();
    rx_.active = true;
    rx_.requestId = id;
    rx_.expected = total;
    rx_.sequence = 1;
    for (uint8_t b = 2; b < frame.can_dlc && rx_.index < total; ++b) {
      rx_.buffer[rx_.index++] = frame.data[b];
    }
    rx_.timeoutMs = millis() + kRxTimeoutMs;
    sendFlowControl(kModules[targetModule].responseId);
    return;
  }

  if (pci == 0x20) {  // consecutive frame of a segmented request
    if (!rx_.active || rx_.requestId != id) return;
    if ((frame.data[0] & 0x0F) != rx_.sequence) {
      rx_ = RxState();  // out of order: abort the assembly
      return;
    }
    rx_.sequence = static_cast<uint8_t>((rx_.sequence + 1) & 0x0F);
    for (uint8_t b = 1; b < frame.can_dlc && rx_.index < rx_.expected; ++b) {
      rx_.buffer[rx_.index++] = frame.data[b];
    }
    rx_.timeoutMs = millis() + kRxTimeoutMs;
    if (rx_.index >= rx_.expected) {
      const int mod = targetModule;
      const uint16_t len = rx_.expected;
      uint8_t assembled[kMaxPayload];
      memcpy(assembled, rx_.buffer, len);
      rx_ = RxState();
      if (mod >= 0) dispatch(static_cast<Module>(mod), false, assembled, len);
    }
    return;
  }
}

// --------------------------------------------------------------- dispatch ----

bool Server::serviceSupported(Module module, uint8_t service) const {
  const bool isEcm = (module == Module::ECM);
  switch (service) {
    // Emissions services live on the engine controller only, which is what a
    // real Corolla does -- the TCM does not answer mode 01.
    case 0x01: case 0x02: case 0x04: case 0x06: case 0x0A:
      return isEcm;
    // Every module reports its own stored faults and identifies itself.
    case 0x03: case 0x07: case 0x09:
      return true;
    // Manufacturer services are answered by whichever module is addressed.
    case 0x22: case 0x3E:
      return true;
    default:
      return false;
  }
}

void Server::dispatch(Module module, bool functional, const uint8_t *data, uint16_t length) {
  if (length < 1) return;
  const uint8_t service = data[0];

  if (!serviceSupported(module, service)) {
    // A module that does not implement a service stays silent on a functional
    // request and answers negatively only when addressed directly.
    if (!functional) negative(module, service, kNrcServiceNotSupported);
    return;
  }

  switch (service) {
    case 0x01: serviceMode01(module, data, length); break;
    case 0x02: serviceMode02(module, data, length); break;
    case 0x03: serviceMode03(module); break;
    case 0x04: serviceMode04(module); break;
    case 0x06: serviceMode06(module, data, length); break;
    case 0x07: serviceMode07(module); break;
    case 0x09: serviceMode09(module, data, length); break;
    case 0x0A: serviceMode0A(module); break;
    case 0x22: serviceMode22(module, data, length); break;
    case 0x3E: serviceMode3E(module, data, length); break;
    default:
      if (!functional) negative(module, service, kNrcServiceNotSupported);
      break;
  }
}

// ----------------------------------------------------------- mode 01 data ----

bool Server::buildPid(uint8_t pid, uint8_t *out, uint8_t &length) {
  if (model_ == nullptr) return false;
  const vehicle::State &s = model_->state();

  if (isRangePid(pid)) {
    const uint32_t mask = supportedBitmap(pid);
    length = 4;
    out[0] = static_cast<uint8_t>(mask >> 24);
    out[1] = static_cast<uint8_t>(mask >> 16);
    out[2] = static_cast<uint8_t>(mask >> 8);
    out[3] = static_cast<uint8_t>(mask);
    return true;
  }

  if (!isPidSupported(pid)) return false;
  length = pidLength(pid);

  switch (pid) {
    case 0x01: {  // monitor status since codes cleared
      const uint8_t count = confirmedCount();
      out[0] = static_cast<uint8_t>((milOn() ? 0x80 : 0x00) | (count & 0x7F));
      out[1] = 0x07;  // misfire, fuel system and components supported+complete
      out[2] = 0xE5;  // catalyst, evap, O2, O2 heater, EGR supported
      out[3] = 0x04;  // evap monitor still incomplete
      return true;
    }
    case 0x03: {  // fuel system status
      out[0] = s.closedLoop ? 0x02 : (s.coolantTempC < 60.0f ? 0x01 : 0x04);
      out[1] = 0x00;  // bank 2 not present
      return true;
    }
    case 0x04: out[0] = percentByte(s.engineLoad * 100.0f); return true;
    case 0x05: out[0] = tempByte(s.coolantTempC); return true;
    case 0x06: out[0] = trimByte(s.shortTrimBank1); return true;
    case 0x07: out[0] = trimByte(s.longTrimBank1); return true;
    case 0x0B: out[0] = static_cast<uint8_t>(s.mapKpa + 0.5f); return true;
    case 0x0C: put16(out, static_cast<uint16_t>(s.engineRpm * 4.0f)); return true;
    case 0x0D: out[0] = static_cast<uint8_t>(s.speedKph); return true;
    case 0x0E: {  // timing advance: retarded at idle, advanced on light load
      const float advance = s.engineRunning ? (8.0f + 22.0f * (1.0f - s.engineLoad)) : 0.0f;
      out[0] = static_cast<uint8_t>((advance + 64.0f) * 2.0f);
      return true;
    }
    case 0x0F: out[0] = tempByte(s.intakeTempC); return true;
    case 0x10: put16(out, static_cast<uint16_t>(s.mafGramsSec * 100.0f)); return true;
    case 0x11: out[0] = percentByte(s.throttlePercent); return true;
    case 0x13: out[0] = 0x03; return true;  // bank 1 sensors 1 and 2 present
    case 0x14: {  // upstream sensor voltage plus the trim it is driving
      const float volts = s.closedLoop ? (0.45f + 0.35f * (1.0f - s.lambda) * 10.0f) : 0.45f;
      float clamped = volts;
      if (clamped < 0.0f) clamped = 0.0f;
      if (clamped > 1.275f) clamped = 1.275f;
      out[0] = static_cast<uint8_t>(clamped / 0.005f);
      out[1] = trimByte(s.shortTrimBank1);
      return true;
    }
    case 0x15: {  // downstream sensor, lazy behind a healthy catalyst
      out[0] = static_cast<uint8_t>(s.o2DownstreamVolts / 0.005f);
      out[1] = 0xFF;  // trim not used by this sensor
      return true;
    }
    case 0x1C: out[0] = 0x01; return true;  // OBD-II as defined by CARB
    case 0x1F: put16(out, static_cast<uint16_t>(s.engineRunSeconds)); return true;
    case 0x21: {  // distance travelled with the MIL on
      const uint16_t km = milOn() ? static_cast<uint16_t>(s.distanceSinceClearKm) : 0;
      put16(out, km);
      return true;
    }
    case 0x2E: out[0] = percentByte(s.commandedEgrPercent); return true;
    case 0x2F: out[0] = percentByte(s.fuelLevelPercent); return true;
    case 0x30: out[0] = static_cast<uint8_t>(s.warmupsSinceClear & 0xFF); return true;
    case 0x31: put16(out, static_cast<uint16_t>(s.distanceSinceClearKm)); return true;
    case 0x32: {  // evap vapour pressure, signed, quarter-Pa units
      const int16_t pa = static_cast<int16_t>(-120.0f + 40.0f * s.engineLoad);
      put16(out, static_cast<uint16_t>(pa * 4 + 32767));
      return true;
    }
    case 0x33: out[0] = static_cast<uint8_t>(s.barometricKpa + 0.5f); return true;
    case 0x34: {  // wide-range sensor: equivalence ratio and sensor current
      const float ratio = (s.lambda > 0.01f) ? s.lambda : 1.0f;
      put16(out, static_cast<uint16_t>(ratio * 32768.0f));
      const int16_t currentMa = static_cast<int16_t>((ratio - 1.0f) * 4000.0f);
      put16(out + 2, static_cast<uint16_t>(currentMa + 32768));
      return true;
    }
    case 0x3C: {  // catalyst temperature, bank 1 sensor 1
      const float catTemp = 250.0f + s.engineLoad * 480.0f + s.coolantTempC * 1.4f;
      put16(out, static_cast<uint16_t>((catTemp + 40.0f) * 10.0f));
      return true;
    }
    case 0x41: {  // monitor status this drive cycle
      out[0] = 0x00;
      out[1] = 0x07;
      out[2] = 0xE5;
      out[3] = 0x04;
      return true;
    }
    case 0x42: put16(out, static_cast<uint16_t>(s.batteryVolts * 1000.0f)); return true;
    case 0x43: put16(out, static_cast<uint16_t>(s.engineLoad * 100.0f * 255.0f / 100.0f)); return true;
    case 0x44: put16(out, static_cast<uint16_t>(s.lambda * 32768.0f)); return true;
    case 0x45: out[0] = percentByte(s.throttlePercent * 0.9f); return true;
    case 0x46: out[0] = tempByte(s.ambientTempC); return true;
    case 0x47: out[0] = percentByte(s.throttlePercent); return true;
    case 0x49: out[0] = percentByte(s.pedalPercent); return true;
    case 0x4A: out[0] = percentByte(s.pedalPercent * 0.96f); return true;
    case 0x4C: out[0] = percentByte(s.throttlePercent); return true;
    case 0x51: out[0] = 0x01; return true;  // gasoline
    default:
      return false;
  }
}

void Server::serviceMode01(Module module, const uint8_t *data, uint16_t length) {
  if (length < 2) {
    negative(module, 0x01, kNrcIncorrectLength);
    return;
  }

  // A tester may ask for several PIDs in one request; answer each in turn.
  uint8_t response[kMaxPayload];
  response[0] = 0x41;
  uint16_t out = 1;
  bool any = false;

  for (uint16_t i = 1; i < length && i <= 6; ++i) {
    const uint8_t pid = data[i];
    uint8_t payload[8];
    uint8_t payloadLen = 0;
    if (!buildPid(pid, payload, payloadLen)) continue;
    if (out + 1 + payloadLen > kMaxPayload) break;
    response[out++] = pid;
    for (uint8_t b = 0; b < payloadLen; ++b) response[out++] = payload[b];
    any = true;
  }

  if (!any) {
    negative(module, 0x01, kNrcRequestOutOfRange);
    return;
  }
  respond(module, response, out);
}

void Server::serviceMode02(Module module, const uint8_t *data, uint16_t length) {
  if (length < 3) {
    negative(module, 0x02, kNrcIncorrectLength);
    return;
  }
  const uint8_t pid = data[1];
  const uint8_t frameNumber = data[2];

  if (!freeze_.valid || frameNumber != 0x00) {
    negative(module, 0x02, kNrcRequestOutOfRange);
    return;
  }

  uint8_t response[16];
  response[0] = 0x42;
  response[1] = pid;
  response[2] = frameNumber;
  uint8_t n = 3;

  switch (pid) {
    case 0x00:  // which freeze-frame PIDs exist
      response[n++] = 0x00; response[n++] = 0x0F;
      response[n++] = 0x80; response[n++] = 0x11;
      break;
    case 0x02: put16(&response[n], freeze_.dtcCode); n += 2; break;
    case 0x03: response[n++] = freeze_.fuelSystemStatus; response[n++] = 0x00; break;
    case 0x04: response[n++] = percentByte(freeze_.loadPercent); break;
    case 0x05: response[n++] = freeze_.coolantPlus40; break;
    case 0x06: response[n++] = trimByte(freeze_.shortTrim); break;
    case 0x07: response[n++] = trimByte(freeze_.longTrim); break;
    case 0x0B: response[n++] = static_cast<uint8_t>(freeze_.mapKpa); break;
    case 0x0C: put16(&response[n], static_cast<uint16_t>(freeze_.rpm * 4)); n += 2; break;
    case 0x0D: response[n++] = freeze_.speedKph; break;
    case 0x0F: response[n++] = freeze_.intakePlus40; break;
    case 0x11: response[n++] = percentByte(freeze_.throttlePercent); break;
    default:
      negative(module, 0x02, kNrcRequestOutOfRange);
      return;
  }
  respond(module, response, n);
}

void Server::serviceMode03(Module module) {
  uint8_t response[kMaxPayload];
  response[0] = 0x43;
  uint8_t count = 0;
  uint16_t out = 2;
  for (int i = 0; i < kMaxDtcs; ++i) {
    if (!dtcs_[i].used || !dtcs_[i].confirmed || dtcs_[i].module != module) continue;
    if (out + 2 > kMaxPayload) break;
    put16(&response[out], dtcs_[i].code);
    out += 2;
    count++;
  }
  response[1] = count;
  respond(module, response, out);
}

void Server::serviceMode04(Module module) {
  clearAll();
  const uint8_t response[1] = {0x44};
  respond(module, response, sizeof(response));
}

void Server::serviceMode06(Module module, const uint8_t *data, uint16_t length) {
  if (length < 2) {
    negative(module, 0x06, kNrcIncorrectLength);
    return;
  }
  const uint8_t mid = data[1];
  uint8_t response[kMaxPayload];
  response[0] = 0x46;
  uint8_t n = 1;

  // One record is MID, TID, unit/scaling, then value, min and max as words.
  auto record = [&](uint8_t m, uint8_t tid, uint8_t unit, uint16_t value, uint16_t lo, uint16_t hi) {
    response[n++] = m;
    response[n++] = tid;
    response[n++] = unit;
    put16(&response[n], value); n += 2;
    put16(&response[n], lo);    n += 2;
    put16(&response[n], hi);    n += 2;
  };

  switch (mid) {
    case 0x00:  // supported MIDs 0x01-0x20
      response[n++] = 0x00;
      response[n++] = 0x00; response[n++] = 0x00;
      response[n++] = 0x00; response[n++] = 0x01;
      break;
    case 0x21:  // oxygen sensor monitor, bank 1 sensor 1
      record(0x21, 0x01, 0x0B, 780, 200, 1000);
      break;
    case 0x3C:  // catalyst monitor, bank 1
      record(0x3C, 0x80, 0x0A, 420, 0, 620);
      break;
    default:
      negative(module, 0x06, kNrcRequestOutOfRange);
      return;
  }
  respond(module, response, n);
}

void Server::serviceMode07(Module module) {
  uint8_t response[kMaxPayload];
  response[0] = 0x47;
  uint8_t count = 0;
  uint16_t out = 2;
  for (int i = 0; i < kMaxDtcs; ++i) {
    if (!dtcs_[i].used || !dtcs_[i].pending || dtcs_[i].module != module) continue;
    if (out + 2 > kMaxPayload) break;
    put16(&response[out], dtcs_[i].code);
    out += 2;
    count++;
  }
  response[1] = count;
  respond(module, response, out);
}

void Server::serviceMode09(Module module, const uint8_t *data, uint16_t length) {
  if (length < 2) {
    negative(module, 0x09, kNrcIncorrectLength);
    return;
  }
  const uint8_t infoType = data[1];
  uint8_t response[kMaxPayload];
  response[0] = 0x49;
  response[1] = infoType;

  switch (infoType) {
    case 0x00: {  // supported info types: 02, 04, 06, 0A
      response[2] = 0x54;
      response[3] = 0x40;
      response[4] = 0x00;
      response[5] = 0x00;
      respond(module, response, 6);
      return;
    }
    case 0x02: {  // VIN, engine controller only
      if (module != Module::ECM) {
        negative(module, 0x09, kNrcSubFunctionNotSupported);
        return;
      }
      response[2] = 0x01;  // one data item follows
      memcpy(&response[3], kVin, 17);
      respond(module, response, 20);
      return;
    }
    case 0x04: {  // calibration identification
      if (module != Module::ECM) {
        negative(module, 0x09, kNrcSubFunctionNotSupported);
        return;
      }
      response[2] = 0x01;
      memset(&response[3], 0x00, 16);
      memcpy(&response[3], kCalId, strlen(kCalId));
      respond(module, response, 19);
      return;
    }
    case 0x06: {  // calibration verification number
      if (module != Module::ECM) {
        negative(module, 0x09, kNrcSubFunctionNotSupported);
        return;
      }
      response[2] = 0x01;
      memcpy(&response[3], kCvn, 4);
      respond(module, response, 7);
      return;
    }
    case 0x0A: {  // ECU name -- every module answers, which is how a scan
                  // tool discovers what is on the bus
      const char *name = kModules[static_cast<int>(module)].name;
      response[2] = 0x01;
      memset(&response[3], 0x00, 20);
      const size_t n = strlen(name);
      memcpy(&response[3], name, n > 20 ? 20 : n);
      respond(module, response, 23);
      return;
    }
    default:
      negative(module, 0x09, kNrcSubFunctionNotSupported);
      return;
  }
}

void Server::serviceMode0A(Module module) {
  uint8_t response[kMaxPayload];
  response[0] = 0x4A;
  uint8_t count = 0;
  uint16_t out = 2;
  for (int i = 0; i < kMaxDtcs; ++i) {
    if (!dtcs_[i].used || !dtcs_[i].permanent) continue;
    if (out + 2 > kMaxPayload) break;
    put16(&response[out], dtcs_[i].code);
    out += 2;
    count++;
  }
  response[1] = count;
  respond(module, response, out);
}

void Server::serviceMode22(Module module, const uint8_t *data, uint16_t length) {
  if (length < 3) {
    negative(module, 0x22, kNrcIncorrectLength);
    return;
  }
  if (model_ == nullptr) return;
  const vehicle::State &s = model_->state();
  const uint16_t did = static_cast<uint16_t>((data[1] << 8) | data[2]);

  uint8_t response[kMaxPayload];
  response[0] = 0x62;
  response[1] = data[1];
  response[2] = data[2];
  uint8_t n = 3;

  // Identification DIDs answered by every module.
  if (did == 0xF190) {  // VIN
    memcpy(&response[n], kVin, 17);
    respond(module, response, static_cast<uint16_t>(n + 17));
    return;
  }
  if (did == 0xF18C) {  // ECU serial number
    const char *name = kModules[static_cast<int>(module)].name;
    const size_t len = strlen(name);
    memcpy(&response[n], name, len);
    respond(module, response, static_cast<uint16_t>(n + len));
    return;
  }

  switch (module) {
    case Module::ECM:
      switch (did) {
        case 0x0100: put16(&response[n], static_cast<uint16_t>(s.engineRpm)); n += 2; break;
        case 0x0101: response[n++] = static_cast<uint8_t>(s.speedKph); break;
        case 0x0102: response[n++] = tempByte(s.coolantTempC); break;
        case 0x0103: put16(&response[n], static_cast<uint16_t>(s.oilPressureKpa)); n += 2; break;
        case 0x0104: put16(&response[n], static_cast<uint16_t>(s.mafGramsSec * 100.0f)); n += 2; break;
        case 0x0105: put16(&response[n], static_cast<uint16_t>(s.batteryVolts * 1000.0f)); n += 2; break;
        case 0x0106: response[n++] = tempByte(s.oilTempC); break;
        default: negative(module, 0x22, kNrcRequestOutOfRange); return;
      }
      break;
    case Module::TCM:
      switch (did) {
        case 0xF130: response[n++] = s.gear; break;
        case 0xF131: put16(&response[n], static_cast<uint16_t>(s.turbineRpm)); n += 2; break;
        case 0xF132: response[n++] = static_cast<uint8_t>(s.converterLocked ? 1 : 0); break;
        case 0xF133: response[n++] = tempByte(s.oilTempC); break;
        default: negative(module, 0x22, kNrcRequestOutOfRange); return;
      }
      break;
    case Module::ABS:
      switch (did) {
        case 0xC100:  // all four wheel speeds, quarter km/h per bit
          for (int w = 0; w < 4; ++w) {
            put16(&response[n], static_cast<uint16_t>(s.wheelSpeedKph[w] * 4.0f));
            n += 2;
          }
          break;
        case 0xC101: {
          const int16_t angle = static_cast<int16_t>(s.steeringAngleDeg * 10.0f);
          put16(&response[n], static_cast<uint16_t>(angle)); n += 2;
          break;
        }
        case 0xC102: response[n++] = percentByte(s.brakePercent); break;
        default: negative(module, 0x22, kNrcRequestOutOfRange); return;
      }
      break;
    case Module::BCM:
      switch (did) {
        case 0xB100: response[n++] = percentByte(s.fuelLevelPercent); break;
        case 0xB101: response[n++] = tempByte(s.ambientTempC); break;
        case 0xB102: put16(&response[n], static_cast<uint16_t>(s.batteryVolts * 1000.0f)); n += 2; break;
        case 0xB103:  // odometer in kilometres, three bytes
          response[n++] = static_cast<uint8_t>((static_cast<uint32_t>(s.odometerKm) >> 16) & 0xFF);
          response[n++] = static_cast<uint8_t>((static_cast<uint32_t>(s.odometerKm) >> 8) & 0xFF);
          response[n++] = static_cast<uint8_t>(static_cast<uint32_t>(s.odometerKm) & 0xFF);
          break;
        default: negative(module, 0x22, kNrcRequestOutOfRange); return;
      }
      break;
  }
  respond(module, response, n);
}

void Server::serviceMode3E(Module module, const uint8_t *data, uint16_t length) {
  if (length < 2) {
    negative(module, 0x3E, kNrcIncorrectLength);
    return;
  }
  // Suppress-positive-response bit: the tester wants silence.
  if ((data[1] & 0x80) != 0) return;
  const uint8_t response[2] = {0x7E, static_cast<uint8_t>(data[1] & 0x7F)};
  respond(module, response, sizeof(response));
}

}  // namespace obd
