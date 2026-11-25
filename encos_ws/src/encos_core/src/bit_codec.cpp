#include "encos_core/bit_codec.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace encos::wire {
static_assert(sizeof(float) == sizeof(std::uint32_t) && std::numeric_limits<float>::is_iec559);

namespace {
void require_bits(const std::uint8_t* data, unsigned bit, unsigned width) {
  if (!data || width == 0 || width > 32 || bit > 64 - width) {
    throw CodecError(CodecIssue::value, "bit field outside ENCOS payload");
  }
}
}  // namespace

std::uint32_t read_bits(const std::uint8_t* data, unsigned bit, unsigned width) {
  require_bits(data, bit, width);
  std::uint32_t value = 0;
  for (unsigned i = bit; i < bit + width; ++i) {
    value = (value << 1) | ((data[i / 8] >> (7 - i % 8)) & 1u);
  }
  return value;
}

void write_bits(std::uint8_t* data, unsigned bit, unsigned width, std::uint32_t value) {
  require_bits(data, bit, width);
  if (width < 32 && value >= (1u << width)) {
    throw CodecError(CodecIssue::value, "value does not fit bit field");
  }
  for (unsigned i = 0; i < width; ++i) {
    const unsigned address = bit + i;
    const unsigned mask = 1u << (7 - address % 8);
    const unsigned payload = value >> (width - 1 - i);
    data[address / 8] = static_cast<std::uint8_t>((data[address / 8] & ~mask) |
                                                   ((payload & 1u) * mask));
  }
}

std::uint16_t read_u16(const std::uint8_t* data) {
  return static_cast<std::uint16_t>((static_cast<unsigned>(data[0]) << 8) | data[1]);
}

std::int16_t read_i16(const std::uint8_t* data) {
  const std::int32_t value = read_u16(data);
  return static_cast<std::int16_t>(value < 0x8000 ? value : value - 0x10000);
}

double float_value(std::uint32_t bits) {
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  if (!std::isfinite(value)) {
    throw CodecError(CodecIssue::nonfinite, "non-finite wire float32");
  }
  return value;
}

double read_float(const std::uint8_t* data) {
  return float_value(read_bits(data, 0, 32));
}

std::uint32_t float_bits(double value) {
  if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
    throw CodecError(CodecIssue::value, "float32 out of range");
  }
  const auto narrowed = static_cast<float>(value);
  std::uint32_t bits;
  std::memcpy(&bits, &narrowed, sizeof(bits));
  return bits;
}

std::uint16_t fixed(double value, double scale, std::int32_t min, std::int32_t max) {
  if (!std::isfinite(scale) || scale <= 0 || min < -32768 || max > 65535 || min > max ||
      !std::isfinite(value) || value < min / scale || value > max / scale) {
    throw CodecError(CodecIssue::value, "fixed-point field out of range");
  }
  return static_cast<std::uint16_t>(std::lround(value * scale));
}

}  // namespace encos::wire
