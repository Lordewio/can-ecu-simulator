#pragma once

#include <stdint.h>

namespace obd {

// The set of mode 01 PIDs a 2012 Corolla 1.8 actually answers. A real ECU
// supports a sparse, characteristic subset -- scan tools fingerprint vehicles
// from this bitmap, so claiming 0xFFFFFFFF for every range (as the previous
// firmware did) is one of the fastest ways to be spotted as a simulator.
//
// Deliberately absent, because this engine has no such hardware:
//   0x08/0x09  bank 2 fuel trims      - single-bank inline four
//   0x0A       fuel pressure          - no fuel pressure sensor
//   0x22/0x23  fuel rail pressure     - port injection, not GDI
//   0x5C       engine oil temperature - no oil temperature sensor
constexpr uint8_t kSupportedPids[] = {
    // 0x01 - 0x20
    0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
    0x11, 0x13, 0x14, 0x15, 0x1C, 0x1F, 0x20,
    // 0x21 - 0x40
    0x21, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x3C, 0x40,
    // 0x41 - 0x60
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x49, 0x4A, 0x4C, 0x51,
};
constexpr int kSupportedPidCount = static_cast<int>(sizeof(kSupportedPids) / sizeof(kSupportedPids[0]));

inline bool isPidSupported(uint8_t pid) {
  for (int i = 0; i < kSupportedPidCount; ++i) {
    if (kSupportedPids[i] == pid) return true;
  }
  return false;
}

// Builds the 32-bit "PIDs supported" bitmap for a range request (0x00, 0x20,
// 0x40 ...). Bit 31 of the result is the first PID after `base`, bit 0 is
// `base + 0x20`. Derived from the table above so the two can never disagree.
inline uint32_t supportedBitmap(uint8_t base) {
  uint32_t mask = 0;
  for (uint8_t offset = 1; offset <= 32; ++offset) {
    const uint16_t pid = static_cast<uint16_t>(base) + offset;
    if (pid > 0xFF) break;
    if (isPidSupported(static_cast<uint8_t>(pid))) {
      mask |= (1UL << (32 - offset));
    }
  }
  return mask;
}

// A range PID is answerable only if it is itself listed as supported (0x00 is
// always answerable; 0x20/0x40/... only when the previous range advertised it).
inline bool isRangePid(uint8_t pid) { return (pid % 0x20) == 0; }

// Mode 01 response payload length for a supported PID, in bytes.
inline uint8_t pidLength(uint8_t pid) {
  if (isRangePid(pid)) return 4;
  switch (pid) {
    case 0x01: case 0x34: case 0x41:
      return 4;
    case 0x03: case 0x0C: case 0x10: case 0x14: case 0x15: case 0x1F:
    case 0x21: case 0x31: case 0x32: case 0x3C: case 0x42: case 0x43:
    case 0x44:
      return 2;
    default:
      return 1;
  }
}

}  // namespace obd
