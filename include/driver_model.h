#pragma once

#include <stdint.h>

#include "vehicle_model.h"

namespace driver {

enum class Scenario : uint8_t {
  TRIP = 0,      // full key-off -> crank -> city -> highway -> city loop
  CITY = 1,      // stop-and-go
  HIGHWAY = 2,   // sustained cruise
  IDLE = 3,      // running, stationary
  MANUAL = 4,    // follow an externally commanded target speed
  KEY_OFF = 5,   // vehicle shut down
};

// A closed-loop driver. It watches the vehicle's actual speed and works the
// pedals to track a target, which is what makes throttle and speed causally
// related instead of two independent waveforms.
class Driver {
 public:
  void begin(Scenario scenario);
  void setScenario(Scenario scenario);
  void setManualTarget(float kph);

  vehicle::Input update(const vehicle::State &st, float dt);

  Scenario scenario() const { return scenario_; }
  const char *phaseName() const { return phaseName_; }
  float targetKph() const { return targetKph_; }

 private:
  void advancePhase(const vehicle::State &st);

  Scenario scenario_ = Scenario::TRIP;
  const char *phaseName_ = "key off";
  int phaseIndex_ = 0;
  float phaseTimer_ = 0.0f;
  float targetKph_ = 0.0f;
  float manualTargetKph_ = 0.0f;
  float aggression_ = 0.5f;

  float integrator_ = 0.0f;
  float pedal_ = 0.0f;
  float brake_ = 0.0f;
  float steering_ = 0.0f;
  float noisePhase_ = 0.0f;

  bool keyOn_ = false;
  bool starter_ = false;
};

}  // namespace driver
