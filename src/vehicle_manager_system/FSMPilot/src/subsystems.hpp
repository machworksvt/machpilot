#ifndef SUBSYSTEMS_HPP
#define SUBSYSTEMS_HPP

#include <string_view>
#include <type_traits>

enum class SubSystems : std::uint8_t {
  SUBSYSTEM_0,
  SUBSYSTEM_1,
};

static const std::underlying_type_t<SubSystems> SUBSYSTEM_COUNT = 2;

std::string_view sub_system_to_string(SubSystems id);

#endif  // SUBSYSTEMS_HPP