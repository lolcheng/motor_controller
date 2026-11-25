#include "check.hpp"

using namespace test;

int main() {
  std::array<std::uint8_t, 8> data{};
  wire::write_bits(data.data(), 0, 3, 1);
  wire::write_bits(data.data(), 3, 32, 0xc2b40000);
  CHECK(data[0] == 0x38 && data[1] == 0x56 && data[2] == 0x80 && data[3] == 0);
  CHECK(wire::read_bits(data.data(), 3, 32) == 0xc2b40000);
  near(wire::float_value(0xc2b40000), -90);
  CHECK(wire::float_bits(-90) == 0xc2b40000);
  CHECK(wire::float_bits(-0.0) == 0x80000000);
  for (const auto raw : {0x7f800000u, 0xff800000u, 0x7fc00001u, 0x7f800001u}) {
    throws(CodecIssue::nonfinite, [raw] { wire::float_value(raw); });
  }
  throws(CodecIssue::value, [] { wire::float_bits(inf); });
  throws(CodecIssue::value, [] { wire::float_bits(quiet_nan); });
  throws(CodecIssue::value, [] { wire::float_bits(std::numeric_limits<double>::max()); });
  data = {0x80, 0x00, 0x7f, 0xff, 0xfe, 0x0c, 0, 0};
  CHECK(wire::read_i16(data.data()) == -32768);
  CHECK(wire::read_i16(data.data() + 2) == 32767);
  CHECK(wire::read_i16(data.data() + 4) == -500);
  CHECK(wire::fixed(-327.68, 100, -32768, 32767) == 0x8000);
  CHECK(wire::fixed(327.67, 100, -32768, 32767) == 0x7fff);
  throws(CodecIssue::value, [] { wire::fixed(327.67001, 100, -32768, 32767); });
  throws(CodecIssue::value, [] { wire::fixed(-327.68001, 100, -32768, 32767); });
  throws(CodecIssue::value, [] { wire::fixed(100, 0, 0, 1); });
  throws(CodecIssue::value, [&] { wire::read_bits(data.data(), 63, 2); });
  throws(CodecIssue::value, [&] { wire::read_bits(data.data(), 0, 33); });
  throws(CodecIssue::value, [&] { wire::write_bits(data.data(), 0, 2, 4); });
}
