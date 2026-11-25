#include "check.hpp"
#include "encos_core/mock_transport.hpp"
#include "encos_core/transaction_manager.hpp"

using namespace test;

int main() {
  for (const auto format : {WireFormat::classic, WireFormat::fd_brs}) {
    MockTransport mock(1);
    Transport& bus = mock;
    const Protocol codec(format);
    TransactionManager transactions;
    const auto query = codec.query(id, QueryCode::position);
    CHECK(bus.connected() && !bus.receive());
    CHECK(transactions.begin(query, at_ms(1), at_ms(10)) == Begin::accepted);
    CHECK(bus.send(query.frame) == SendStatus::accepted);
    CHECK(bus.send(query.frame) == SendStatus::full);
    CHECK(!bus.receive());
    const auto sent = mock.take_sent();
    CHECK(sent && matches_wire_format(*sent, format) && sent->data[0] == 0xe1);
    CHECK(mock.inject({*sent, at_ms(2), Origin::local_echo}));
    const auto reply = rx({0xa0, 1, 0x42, 0xb4, 0, 0}, id, 3, Origin::remote, format);
    CHECK(!mock.inject(reply));
    CHECK(transactions.receive(codec.decode(*bus.receive()), at_ms(2)).state == TransactionState::ignored);
    CHECK(mock.inject(reply));
    const auto event = transactions.receive(codec.decode(*bus.receive()), at_ms(3));
    CHECK(event.state == TransactionState::completed && event.reply);
    if (event.reply) {
      const auto response = std::get<QueryReply>(event.reply->feedback);
      near(std::get<Scalar>(response.value).value, 90 * deg);
    }

    CHECK(bus.send(query.frame) == SendStatus::accepted);
    CHECK(mock.inject(reply));
    mock.set_connected(false);
    CHECK(!bus.connected() && !bus.receive() && !mock.take_sent());
    CHECK(bus.send(query.frame) == SendStatus::disconnected && !mock.inject(reply));
    mock.set_connected(true);
    CHECK(!bus.receive() && !mock.take_sent());
  }
}
