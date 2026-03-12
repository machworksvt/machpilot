#ifndef SEVERITY_HPP
#define SEVERITY_HPP

#include <string_view>

/**
 * @brief Represents how severe a log is.
 */
enum class Severity : uint8_t { Log, Warning, Error };

/// The number of eliments in Severity
const std::underlying_type_t<Severity> SEVERITY_SIZE=3;

/**
 * @brief converts a Severity to string view with a static lifetime
*/
std::string_view severity_to_string(Severity s);


#endif
