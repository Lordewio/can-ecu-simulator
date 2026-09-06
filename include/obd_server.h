#pragma once

#include <mcp2515.h>
#include <stdint.h>

#include "vehicle_model.h"

namespace obd {

constexpr uint16_t kFunctionalRequestId = 0x7DF;
constexpr int kModuleCount = 4;
constexpr int kMaxPayload = 72;
constexpr int kMaxDtcs = 8;

enum class Module : uint8_t { ECM = 0, TCM = 1, ABS = 2, BCM = 3 };

struct ModuleInfo {
  uint16_t requestId;
  uint16_t responseId;
  const char *name;
};

extern const ModuleInfo kModules[kModuleCount];

struct Dtc {
  uint16_t code = 0;       // ISO 15031 encoded, e.g. 0x0171 = P0171
  Module module = Module::ECM;
  bool pending = false;    // seen this drive cycle          -> service 07
  bool confirmed = false;  // illuminates the MIL            -> service 03
  bool permanent = false;  // survives a service 04 clear    -> service 0A
  bool used = false;
};

// Snapshot captured at the moment a DTC confirms -> service 02.
struct FreezeFrame {
  bool valid = false;
  uint16_t dtcCode = 0;
  uint16_t rpm = 0;
  uint8_t speedKph = 0;
  uint8_t loadPercent = 0;
  uint8_t coolantPlus40 = 0;
  uint8_t intakePlus40 = 0;
  uint8_t throttlePercent = 0;
  uint16_t mapKpa = 0;
  int8_t shortTrim = 0;
  int8_t longTrim = 0;
  uint8_t fuelSystemStatus = 0;
};

// ISO 15765-2 transport plus the OBD-II / UDS services layered on top.
class Server {
 public:
  void begin(MCP2515 *can, vehicle::Model *model);

  // Feed every frame read off the bus; non-diagnostic IDs are ignored.
  void handleFrame(const struct can_frame &frame);

  // Advances any in-flight segmented transfer. Call every loop iteration.
  void pump();

  bool setDtc(uint16_t code, Module module, bool confirmed);
  bool clearDtc(uint16_t code);
  void clearAll();
  uint8_t confirmedCount() const;
  uint8_t pendingCount() const;
  bool milOn() const;
  const Dtc *dtcs() const { return dtcs_; }

  // True while a segmented response is still being transmitted.
  bool busy() const { return tx_.phase != TxPhase::IDLE; }

 private:
  enum class TxPhase : uint8_t { IDLE, WAIT_FC, SENDING };

  struct TxState {
    TxPhase phase = TxPhase::IDLE;
    uint16_t responseId = 0;
    uint8_t payload[kMaxPayload] = {0};
    uint16_t length = 0;
    uint16_t index = 0;
    uint8_t sequence = 1;
    uint8_t blockSize = 0;
    uint8_t blockRemaining = 0;
    uint32_t stMinMs = 0;
    uint32_t nextSendMs = 0;
    uint32_t timeoutMs = 0;
  };

  struct QueuedResponse {
    bool used = false;
    uint16_t responseId = 0;
    uint8_t payload[kMaxPayload] = {0};
    uint16_t length = 0;
  };

  struct RxState {
    bool active = false;
    uint16_t requestId = 0;
    uint8_t buffer[kMaxPayload] = {0};
    uint16_t expected = 0;
    uint16_t index = 0;
    uint8_t sequence = 1;
    uint32_t timeoutMs = 0;
  };

  void cancelTransfers(int moduleIndex);
  void sendRaw(uint16_t id, const uint8_t *data, uint8_t dlc);
  void sendFlowControl(uint16_t responseId);
  void respond(Module module, const uint8_t *payload, uint16_t length);
  void negative(Module module, uint8_t service, uint8_t nrc);

  void dispatch(Module module, bool functional, const uint8_t *data, uint16_t length);
  bool serviceSupported(Module module, uint8_t service) const;

  void serviceMode01(Module module, const uint8_t *data, uint16_t length);
  void serviceMode02(Module module, const uint8_t *data, uint16_t length);
  void serviceMode03(Module module);
  void serviceMode04(Module module);
  void serviceMode06(Module module, const uint8_t *data, uint16_t length);
  void serviceMode07(Module module);
  void serviceMode09(Module module, const uint8_t *data, uint16_t length);
  void serviceMode0A(Module module);
  void serviceMode22(Module module, const uint8_t *data, uint16_t length);
  void serviceMode3E(Module module, const uint8_t *data, uint16_t length);

  bool buildPid(uint8_t pid, uint8_t *out, uint8_t &length);

  MCP2515 *can_ = nullptr;
  vehicle::Model *model_ = nullptr;
  Dtc dtcs_[kMaxDtcs];
  FreezeFrame freeze_;
  TxState tx_;
  RxState rx_;
  QueuedResponse queue_[kModuleCount];
};

// Helpers shared with the console: "P0171" <-> 0x0171.
uint16_t encodeDtc(const char *text);
void decodeDtc(uint16_t code, char *out5);

}  // namespace obd
