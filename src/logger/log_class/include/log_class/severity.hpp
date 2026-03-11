#ifndef SEVERITY_HPP
#define SEVERITY_HPP

#include <string_view>

enum class Severity : uint8_t { Log, Warning, Error };


const std::underlying_type_t<Severity> SEVERITY_SIZE=3;

/**
 * @brief converts a Severity to string view with a static lifetime
*/
std::string_view severity_to_string(Severity s);


#endif
