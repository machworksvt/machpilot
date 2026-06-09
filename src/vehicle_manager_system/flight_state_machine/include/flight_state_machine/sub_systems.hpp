#ifndef SUBSYSTEMS_HPP
#define SUBSYSTEMS_HPP

#include <string_view>
#include <type_traits>

/**
 * A list of subsystems that need to start up durring initialization
 */
enum class SubSystems : std::uint8_t {
  SUBSYSTEM_0,
  SUBSYSTEM_1,
};

/**
 * how many SubStytems there are
 */
static const std::underlying_type_t<SubSystems> SUBSYSTEM_COUNT = 2;

/**
 * @brief converts a SubSystems to string veiw with a static lifetime
*/
std::string_view sub_system_to_string(SubSystems id);

#endif  // SUBSYSTEMS_HPP