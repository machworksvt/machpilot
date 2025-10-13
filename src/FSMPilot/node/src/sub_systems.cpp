#include "sub_systems.hpp"

std::string_view SubSystemToString(SubSystems id) {
  switch (id) {
    case SUBSYSTEM_0: return "Subsystem 0";
    case SUBSYSTEM_1: return "Subsystem 1";
    default: return "Unknown Subsystem";
  }
}