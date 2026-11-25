#include "check.hpp"

using namespace test;
using Q = QueryCode;
using C = ConfigCode;

int main() {
  const Protocol p;
  const auto decode = [&](std::initializer_list<std::uint8_t> data) { return p.decode(rx(data)).feedback; };
  const auto range = profile();
  const auto vector = rx({0x20, 0xff, 0xff, 0x00, 0x0f, 0xff, 0, 0xff}, id, 9);
  const auto raw = std::get<Telemetry>(p.decode(vector).feedback);
  CHECK(raw.quantized && *raw.quantized == (std::array<std::uint16_t, 3>{65535, 0, 4095}));
  CHECK(!raw.position_rad && !raw.velocity_rad_s && !raw.phase_current_a && !raw.estimated_torque_nm);
  const auto t1 = std::get<Telemetry>(p.decode(vector, range).feedback);
  near(t1.position_rad->value, 12.5);
  near(t1.velocity_rad_s->value, -18);
  near(t1.phase_current_a->value, 90);
  near(t1.estimated_torque_nm->value, 225);
  near(t1.motor_c->value, -25);
  near(t1.mos_c->value, 102.5);
  CHECK(t1.position_rad->host_rx == at_ms(9));
  CHECK(t1.phase_current_a->source == MeasurementSource::quantized_feedback);
  CHECK(t1.estimated_torque_nm->source == MeasurementSource::verified_kt_estimate);
  auto compensated = range;
  compensated.query35_raw = 2;
  CHECK(!std::get<Telemetry>(p.decode(vector, compensated).feedback).estimated_torque_nm);
  auto invalid = range;
  invalid.feedback1->velocity_rad_s = {inf, inf};
  CHECK(!std::get<Telemetry>(p.decode(vector, invalid).feedback).position_rad);

  const auto t2 = std::get<Telemetry>(decode({0x40, 0xc2, 0xb4, 0, 0, 0xfe, 0x0c, 100}));
  near(t2.position_rad->value, -90 * deg);
  near(t2.phase_current_a->value, -5);
  near(t2.motor_c->value, 25);
  CHECK(!t2.velocity_rad_s && !t2.mos_c && !t2.estimated_torque_nm);
  const auto t3_vector = rx({0x60, 0xc2, 0x70, 0, 0, 0x80, 0, 150}, id, 12,
                            Origin::remote, WireFormat::fd_brs);
  const auto t3 = std::get<Telemetry>(Protocol{WireFormat::fd_brs}.decode(t3_vector, range).feedback);
  near(t3.velocity_rad_s->value, -60 * rpm);
  near(t3.phase_current_a->value, -327.68);
  near(t3.motor_c->value, 50);
  CHECK(!t3.position_rad && !t3.mos_c);
  for (std::uint8_t i = 0; i <= 7; ++i) {
    const auto code = static_cast<std::uint8_t>(0x20 | i);
    CHECK(std::get<Telemetry>(decode({code, 0, 0, 0, 0, 0, 0, 0})).error == static_cast<MotorError>(i));
  }

  const auto query = [&](Q expected, std::initializer_list<std::uint8_t> data) {
    const auto reply = std::get<QueryReply>(decode(data));
    CHECK(reply.code == expected && reply.error == MotorError::none);
    return reply.value;
  };
  near(std::get<Scalar>(query(Q::position, {0xa0, 1, 0x42, 0xb4, 0, 0})).value, 90 * deg);
  near(std::get<Scalar>(query(Q::velocity, {0xa0, 2, 0xc2, 0x70, 0, 0})).value, -60 * rpm);
  near(std::get<Scalar>(query(Q::phase_current, {0xa0, 3, 0xc0, 0x20, 0, 0})).value, -2.5);
  near(std::get<Scalar>(query(Q::power, {0xa0, 4, 0x42, 0xc8, 0, 0})).value, 100);
  CHECK(std::get<Raw16>(query(Q::acceleration, {0xa0, 5, 7, 0xd0})).value == 2000);
  near(std::get<Scalar>(query(Q::kt, {0xa0, 22, 0, 0xfa})).value, 2.5);
  const auto query_pair = [&](Q code, std::initializer_list<std::uint8_t> data, double first, double second) {
    const auto pair = std::get<Pair>(query(code, data));
    near(pair.first, first);
    near(pair.second, second);
  };
  query_pair(Q::kp_range, {0xa0, 23, 0, 1, 1, 0xf4}, 1, 500);
  query_pair(Q::kd_range, {0xa0, 24, 0, 0, 0, 50}, 0, 50);
  query_pair(Q::position_range, {0xa0, 25, 0xfb, 0x1e, 4, 0xe2}, -12.5, 12.5);
  query_pair(Q::velocity_range, {0xa0, 26, 0xf8, 0xf8, 7, 8}, -18, 18);
  query_pair(Q::torque_range, {0xa0, 27, 0xfc, 0x7c, 3, 0x84}, -90, 90);
  query_pair(Q::current_range, {0xa0, 28, 0x80, 0, 0x7f, 0xff}, -3276.8, 3276.7);
  const auto uuid = std::get<UuidPart>(query(Q::uuid, {0xa0, 29, 3, 8, 9, 10, 11}));
  CHECK(uuid.index == 3 && uuid.bytes == (std::array<std::uint8_t, 4>{8, 9, 10, 11}));
  const auto versions = std::get<Version>(query(Q::version, {0xa0, 30, 2, 4, 5, 6, 7, 8}));
  CHECK(versions.hardware == (std::array<std::uint8_t, 3>{2, 4, 5}));
  CHECK(versions.software == (std::array<std::uint8_t, 3>{6, 7, 8}));
  near(std::get<Scalar>(query(Q::timeout, {0xa0, 31, 1, 0xf4})).value, 500);
  query_pair(Q::current_pi, {0xa0, 32, 0, 0x64, 1, 0xf4}, 0.01, 50);
  query_pair(Q::velocity_pi, {0xa0, 33, 2, 0x58, 0x27, 0x10}, 0.006, 0.1);
  query_pair(Q::position_pd, {0xa0, 34, 2, 0x58, 0, 4}, 0.006, 0.00004);
  for (std::uint8_t i = 1; i <= 3; ++i) {
    CHECK(std::get<Enumeration>(query(Q::kt_calibration, {0xa0, 35, i})).raw == i);
  }
  CHECK(std::get<Enumeration>(query(Q::brake, {0xa0, 37, 0})).raw == 0);
  CHECK(std::get<Enumeration>(query(Q::brake, {0xa0, 37, 1})).raw == 1);
  near(std::get<Scalar>(query(Q::zero_offset, {0xa0, 38, 0xfc, 0x18})).value, -10 * deg);
  const auto angle = std::get<EncoderAngles>(query(Q::encoder, {0xa0, 39, 1, 0xdc, 0xd8, 0x23, 0x28}));
  CHECK(angle.calibrated);
  CHECK(angle.raw_count == -9000 && angle.calibrated_count == 9000);
  CHECK(!angle.raw_rad && !angle.calibrated_rad);
  auto confirmed_encoder = range;
  confirmed_encoder.query39_degrees_confirmed = true;
  const auto angle_sample = rx({0xa0, 39, 1, 0xdc, 0xd8, 0x23, 0x28});
  const auto with_unit = std::get<EncoderAngles>(
      std::get<QueryReply>(p.decode(angle_sample, confirmed_encoder).feedback).value);
  near(*with_unit.raw_rad, -90 * deg);
  near(*with_unit.calibrated_rad, 90 * deg);
  CHECK(std::get<QueryReply>(decode({0xa5, 1, 0, 0, 0, 0})).error == MotorError::encoder);

  for (std::uint8_t group = 1; group <= 10; ++group) {
    const auto kt = std::get<KtReply>(decode({0xee, group, 0x0f, 0xa1, 4, 0x10, 0xe1, 0x18}));
    CHECK(kt.raw_header == group && kt.group == group);
    near(kt.values[0], 2.5);
    near(kt.values[1], 2.6);
    near(kt.values[2], 2.7);
    near(kt.values[3], 2.8);
  }
  const auto common = std::get<KtReply>(decode({0xee, 0x21, 0, 0, 0, 0, 0, 0}));
  CHECK(common.raw_header == 0x21 && common.group == 1);
  const auto ambiguous = std::get<KtReply>(decode({0xee, 0x12, 0, 0, 0, 0, 0, 0}));
  CHECK(ambiguous.raw_header == 0x12 && !ambiguous.group);

  const auto echo_scalar = [&](C code, std::initializer_list<std::uint8_t> data, double expected) {
    const auto echo = std::get<ConfigEcho>(decode(data));
    CHECK(echo.code == code);
    near(std::get<Scalar>(echo.value).value, expected);
  };
  const auto echo_pair = [&](C code, std::initializer_list<std::uint8_t> data, double first, double second) {
    const auto echo = std::get<ConfigEcho>(decode(data));
    CHECK(echo.code == code);
    const auto pair = std::get<Pair>(echo.value);
    near(pair.first, first);
    near(pair.second, second);
  };
  echo_scalar(C::acceleration, {0xff, 0xfe, 1, 7, 0xd0}, 20);
  echo_scalar(C::kt, {0xff, 0xfe, 4, 0, 0xfa}, 2.5);
  echo_pair(C::kp_range, {0xff, 0xfe, 5, 0, 0, 1, 0xf4}, 0, 500);
  echo_pair(C::kd_range, {0xff, 0xfe, 6, 0, 0, 0, 50}, 0, 50);
  echo_pair(C::position_range, {0xff, 0xfe, 7, 0xfb, 0x1e, 4, 0xe2}, -12.5, 12.5);
  echo_pair(C::velocity_range, {0xff, 0xfe, 8, 0xf8, 0xf8, 7, 8}, -18, 18);
  echo_pair(C::torque_range, {0xff, 0xfe, 9, 0xfc, 0x7c, 3, 0x84}, -90, 90);
  echo_pair(C::current_range, {0xff, 0xfe, 10, 0xfc, 0x7c, 3, 0x84}, -90, 90);
  echo_scalar(C::timeout, {0xff, 0xfe, 11, 1, 0xf4}, 500);
  echo_pair(C::current_pi, {0xff, 0xfe, 12, 0, 0x64, 1, 0xf4}, 0.01, 50);
  echo_pair(C::velocity_pi, {0xff, 0xfe, 13, 2, 0x58, 0x27, 0x10}, 0.006, 0.1);
  echo_pair(C::position_pd, {0xff, 0xfe, 14, 2, 0x58, 0, 4}, 0.006, 0.00004);
  const auto mode = std::get<ConfigEcho>(decode({0xff, 0xfe, 15, 2}));
  CHECK(mode.code == C::kt_calibration && std::get<Enumeration>(mode.value).raw == 2);
  echo_scalar(C::zero_offset, {0xff, 0xfe, 17, 0xfc, 0x18}, -10 * deg);
  for (std::uint8_t i = 1; i <= 17; ++i) {
    if (i == 3) {
      continue;
    }
    const auto ok = std::get<ConfigStatus>(decode({0x80, i, 1}));
    const auto failed = std::get<ConfigStatus>(decode({0x83, i, 0}));
    CHECK(ok.success && static_cast<unsigned>(ok.code) == i);
    CHECK(!failed.success && failed.error == MotorError::overvoltage);
  }
  CHECK(std::get<CodecIssue>(decode({0xff, 0xfe, 2, 1})) == CodecIssue::ambiguous);
  CHECK(std::get<CodecIssue>(decode({0xff, 0xfe, 2, 1, 0})) == CodecIssue::ambiguous);

  const auto b1 = std::get<BrakeReply>(decode({0xb2, 1}));
  CHECK(b1.state == BrakeState::released && !b1.error);
  const auto b2 = std::get<BrakeReply>(decode({0xc0, 0}));
  CHECK(b2.state == BrakeState::engaged && b2.error == MotorError::none);
  CHECK(std::get<BrakeReply>(decode({0xc6, 0})).error == MotorError::brake_overvoltage);

  const auto admin = [&](std::initializer_list<std::uint8_t> data) {
    return std::get<ManagementReply>(p.decode(rx(data, 0x7ff)).feedback);
  };
  const auto discovered = admin({0xff, 0xff, 1, 7, 0xfe});
  CHECK(discovered.kind == ManagementKind::discover && discovered.success && discovered.id == 0x7fe);
  CHECK(!admin({0x80, 0x80, 1, 0x80}).success);
  CHECK(admin({0, 1, 1, 0x03}).kind == ManagementKind::zero);
  CHECK(admin({0, 2, 1, 0x04}).kind == ManagementKind::set_id);
  CHECK(admin({0, 1, 1, 0}).kind == ManagementKind::write_failed);
  CHECK(admin({0x7f, 0x7f, 1, 5, 0x7f, 0x7f}).kind == ManagementKind::reset_id);
}
