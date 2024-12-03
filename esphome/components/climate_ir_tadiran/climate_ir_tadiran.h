#pragma once

#include "esphome/components/climate_ir/climate_ir.h"

#include <cinttypes>

namespace esphome {
namespace climate_ir_tadiran {

// Temperature
const uint8_t TEMP_MIN = 16;  // Celsius
const uint8_t TEMP_MAX = 28;  // Celsius

class TadIrClimate : public climate_ir::ClimateIR {
 public:
  TadIrClimate()
      : climate_ir::ClimateIR(TEMP_MIN, TEMP_MAX,
                              /* temperature_step= */ 1.0f,
                              /* supports_dry= */ false,
                              /* supports_fan_only= */ false,
                              {climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM,
                               climate::CLIMATE_FAN_HIGH}){};

 protected:
  void transmit_state() override;
  bool on_receive(remote_base::RemoteReceiveData data) override;
  climate::ClimateTraits traits() override;

 private:
  void calcChecksum(uint64_t &);
  void transmit(uint64_t);
  void climateStateFromPacket(const uint32_t &);
  void packetFromClimateState(uint64_t &);
};

}  // namespace climate_ir_tadiran
}  // namespace esphome
