#include "subsystems.hpp"

std::string_view sub_system_to_string(SubSystems id) {
  switch (id) {
    case SubSystems::SUBSYSTEM_0:
      return "Subsystem 0";
    case SubSystems::SUBSYSTEM_1:
      return "Subsystem 1";
    default:
      return "Unknown Subsystem";
  }
};
