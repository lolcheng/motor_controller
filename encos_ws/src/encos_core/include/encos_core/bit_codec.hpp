#pragma once

#include <cstdint>
#include <stdexcept>

namespace encos {

enum class CodecIssue { invalid_frame, length, value, nonfinite, unsupported, ambiguous, profile, permission };

class CodecError : public std::runtime_error {
 public:
  CodecError(CodecIssue issue, const char* message) : std::runtime_error(message), issue(issue) {}
  const CodecIssue issue;
};

namespace wire {
constexpr double degrees_to_rad = 0.01745329251994329577;
constexpr double rpm_to_rad_s = 0.10471975511965977462;

std::uint32_t read_bits(const std::uint8_t* data, unsigned bit, unsigned width);
void write_bits(std::uint8_t* data, unsigned bit, unsigned width, std::uint32_t value);
std::uint16_t read_u16(const std::uint8_t* data);
std::int16_t read_i16(const std::uint8_t* data);
double read_float(const std::uint8_t* data);
double float_value(std::uint32_t bits);
std::uint32_t float_bits(double value);
std::uint16_t fixed(double value, double scale, std::int32_t min, std::int32_t max);
}  // namespace wire
}  // namespace encos
