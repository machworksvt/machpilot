#ifndef SOURCE_HPP
#define SOURCE_HPP

#include <string_view>

/**
 * @brief Represents the source of a log
 */
enum class Source: uint16_t{
    Logger,
    ExampleNode,
    LoggerTester0,
    LoggerTester1,
    LoggerTester2,
};

/// The number of eliments in Source
const std::underlying_type_t<Source> SOURCE_SIZE=5;

/**
 * @brief converts a Source to string view with a static lifetime
*/
std::string_view source_to_string(Source s);


#endif


