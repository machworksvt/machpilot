#include "log_class/source.hpp"

std::string_view source_to_string(Source s){
    switch (s)
    {
        case Source::ExampleNode:
            return "ExampleNode";
        case Source::LoggerTester0:
            return "LoggerTester0";
        case Source::LoggerTester1:
            return "LoggerTester1";
        case Source::LoggerTester2:
            return "LoggerTester2";
        case Source::Logger:
            return "Logger";
    }
    return "Unknown";
};