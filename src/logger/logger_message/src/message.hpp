
#include <variant>
#include <chrono>

// Forward declarations
class ExampleLog1{

};

class ExampleLog2{
};

enum class Source{None};
enum class Severity{Log, Warning, Error};

class Log{
	std::chrono::milliseconds time;
	Severity severity;
	std::variant<ExampleLog1, ExampleLog2> sub_log;
};

