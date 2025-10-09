#include <string>
#include "sub_systems.hpp"

std::string sub_system_to_String(SubSystems id) {
  switch (id) {
      case SUBSYSTEM_0: return "Subsystem 0";
      case SUBSYSTEM_1: return "Subsystem 1";
      default: return "Unknown Subsystem";
  }
};