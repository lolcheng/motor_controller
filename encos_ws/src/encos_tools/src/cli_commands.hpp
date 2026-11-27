#pragma once

#include "encos_core/protocol.hpp"

#include <string>
#include <string_view>

namespace encos_tools {

unsigned parse_unsigned(std::string_view text, unsigned maximum);
encos::QueryCode parse_query_code(std::string_view text);
std::string format_can(const encos::CanFrame& frame);
encos::Encoded parse_named_command(std::string_view verb, int count, char** args);
void print_tx(const encos::Encoded& command);
void print_command_usage();

}  // namespace encos_tools
