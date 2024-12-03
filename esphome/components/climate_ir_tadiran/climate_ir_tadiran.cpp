#include "climate_ir_tadiran.h"
#include "esphome/core/log.h"

namespace esphome {
namespace climate_ir_tadiran {

static const char *const TAG = "climate.climate_ir_tadiran";

namespace rx {
constexpr uint32_t MODE_MASK = 0xE000;
constexpr uint32_t MODE_COOL = 0x2000;
constexpr uint32_t MODE_HEAT = 0x4000;
constexpr uint32_t FAN_MASK = 0x30;
constexpr uint32_t FAN_AUTO = 0x00;
constexpr uint32_t FAN_LOW = 0x10;
constexpr uint32_t FAN_MED = 0x20;
constexpr uint32_t FAN_HIGH = 0x30;
constexpr uint32_t POWER_MASK = 0x80;
constexpr uint32_t POWER_ON = 0x80;
constexpr uint32_t TEMP_MASK = 0x1F00;
constexpr uint32_t TEMP_SHIFT = 8;
constexpr uint32_t TEMP_OFFSET = 12;
}  // namespace rx

namespace tx {
constexpr uint64_t COMMAND_SET = 0x200000000000000;
constexpr uint64_t COMMAND_GET = 0x400000000000000;
constexpr uint64_t FAN_AUTO = 0x00000000000000;
constexpr uint64_t FAN_HIGH = 0x30000000000000;
constexpr uint64_t FAN_MED = 0x20000000000000;
constexpr uint64_t FAN_LOW = 0x10000000000000;
constexpr uint64_t MODE_COOL = 0x1000000000000;
constexpr uint64_t MODE_HEAT = 0x2000000000000;
constexpr uint64_t POWER_ON = 0x300000;
constexpr uint64_t POWER_OFF = 0xC00000;
constexpr uint64_t TX_UNKNOWN = 0xC000;
constexpr uint32_t TEMP_SHIFT = 41;
}  // namespace tx

constexpr int HEADER_HIGH = 8'054;
constexpr int FOOTER_HIGH_TX = 180;
constexpr int HEADER_LOW = 3'892;
constexpr int ZERO_HIGH = 666;
constexpr int ZERO_HIGH_TX = 603;
constexpr int ZERO_LOW = 1484;
constexpr int ZERO_LOW_TX = 1436;
constexpr int ONE_HIGH = 1'661;
constexpr int ONE_HIGH_TX = 1601;
constexpr int ONE_LOW = 489;
constexpr int ONE_LOW_TX = 438;

const uint16_t RX_BITS = 24;
const uint16_t TX_BITS = 64;

climate::ClimateTraits TadIrClimate::traits() {
  auto traits = climate_ir::ClimateIR::traits();
  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
  });
  return traits;
}

void TadIrClimate::packetFromClimateState(uint64_t &packet) {
  packet = 0ULL;
  // ccc = 100 periodic, 010 command
  // fff = fan speed
  // mmm = 001 cold, 010 hot
  // tttttt = temperature
  // hhhhhhhhh = shabat mode
  // kkkkkkkk = timer settings
  // oooo = 1100 off, 0011 on, 0010 on timer
  // checksum - the sum of all previous nibbles

  // |???? ?ccc|?fff ?mmm|?ttt ttth|hhhh hhhh|kkkk kkkk|oooo ????|???? ????|chec ksum|
  // |0000'0010|0001'0001|0011'1010|0000'0000|0000'0000|0011'0000|1100'0000|0010'0000|
  //  0000 0010 0000 0010 0011 0010 0000 0000 0000 0000 0011 0000 1100 0000 0001 1000

  packet |= tx::COMMAND_SET;

  switch (this->fan_mode.value()) {
    case climate::CLIMATE_FAN_AUTO:
      packet |= tx::FAN_AUTO;
      break;
    case climate::CLIMATE_FAN_HIGH:
      packet |= tx::FAN_HIGH;
      break;
    case climate::CLIMATE_FAN_MEDIUM:
      packet |= tx::FAN_MED;
      break;
    case climate::CLIMATE_FAN_LOW:
    default:
      packet |= tx::FAN_LOW;
      break;
  }

  switch (this->mode) {
    case climate::CLIMATE_MODE_HEAT:
      packet |= tx::MODE_HEAT;
      break;
    case climate::CLIMATE_MODE_COOL:
    default:
      packet |= tx::MODE_COOL;
      break;
  }

  uint64_t temperature = static_cast<uint8_t>(std::round(this->target_temperature));
  packet |= (temperature << tx::TEMP_SHIFT);

  packet |= (this->mode == climate::CLIMATE_MODE_OFF) ? tx::POWER_OFF : tx::POWER_ON;
  packet |= tx::TX_UNKNOWN;
}

void TadIrClimate::transmit_state() {
  uint64_t packet = 0ULL;
  packetFromClimateState(packet);
  transmit(packet);
}

bool TadIrClimate::on_receive(remote_base::RemoteReceiveData data) {
  uint8_t bitCount = 0;
  uint32_t packet = 0ULL;

  if (!data.expect_item(HEADER_HIGH, HEADER_LOW))
    return false;

  for (bitCount = 0; bitCount < 32; bitCount++) {
    if (data.expect_item(ONE_HIGH, ONE_LOW)) {
      packet = (packet << 1) | 1;
    } else if (data.expect_item(ZERO_HIGH, ZERO_LOW)) {
      packet = (packet << 1) | 0;
    } else if (bitCount == RX_BITS) {
      break;
    } else {
      return false;
    }
  }

  this->climateStateFromPacket(packet);
  this->publish_state();
  return true;
}

void TadIrClimate::climateStateFromPacket(const uint32_t &data) {
  // ssss = 0 no error, 1-6 are errors, 7 condensor is on?,
  // mmm = 001 cold, 010 hot
  // tttt = temperature - 12
  // o = on or off
  // ff = fan speed
  // cksm = checksum - the not (~) of the sum of all previous nibbles

  // |ssss ????|mmmt'tttt|o?ff cksm|
  // |0000'0000|0011'0001|1001'0010|

  uint32_t packet = 0;

  for (uint8_t i = 0; i < 4; i++) {
    uint8_t byte = (data >> (i * 8)) & 0xFF;
    packet |= static_cast<uint32_t>(reverse_bits(byte)) << (i * 8);
  }
  ESP_LOGD(TAG, "Decoded 0x%06" PRIX32, packet);

  this->target_temperature = ((packet & rx::TEMP_MASK) >> rx::TEMP_SHIFT) + rx::TEMP_OFFSET;

  if (packet & rx::POWER_MASK) {
    switch (packet & rx::MODE_MASK) {
      case rx::MODE_HEAT:
        this->mode = climate::CLIMATE_MODE_HEAT;
        break;
      case rx::MODE_COOL:
      default:
        this->mode = climate::CLIMATE_MODE_COOL;
        break;
    }
  } else {
    this->mode = climate::CLIMATE_MODE_OFF;
  }

  switch (packet & rx::FAN_MASK) {
    case rx::FAN_AUTO:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case rx::FAN_HIGH:
      this->fan_mode = climate::CLIMATE_FAN_HIGH;
      break;
    case rx::FAN_MED:
      this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      break;
    case rx::FAN_LOW:
    default:
      this->fan_mode = climate::CLIMATE_FAN_LOW;
      break;
  }
}

void TadIrClimate::transmit(uint64_t value) {
  calcChecksum(value);
  ESP_LOGD(TAG, "Sending climate_tad_ir code: 0x%016" PRIX64, value);

  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();

  data->set_carrier_frequency(0);

  // Reserves 2 items for the header and two items for each bit's high and low.
  data->reserve(2 + TX_BITS * 2u);

  data->item(HEADER_HIGH, HEADER_LOW);

  for (int16_t i = TX_BITS - 8; i >= 0; i -= 8) {
    uint8_t byte = (value >> i) & 0xFF;
    for (uint16_t j = 0; j < 8; j++) {
      if (byte & (1 << j)) {
        data->item(ONE_HIGH_TX, ONE_LOW_TX);
      } else {
        data->item(ZERO_HIGH_TX, ZERO_LOW_TX);
      }
    }
  }

  data->mark(FOOTER_HIGH_TX);
  transmit.perform();
}

void TadIrClimate::calcChecksum(uint64_t &value) {
  uint64_t mask = 0xF;
  uint32_t sum = 0;
  for (uint8_t i = 2; i < 16; i++) {
    sum += (value & (mask << (i * 4))) >> (i * 4);
  }

  value |= (sum & 0xFF);
}

}  // namespace climate_ir_tadiran
}  // namespace esphome
