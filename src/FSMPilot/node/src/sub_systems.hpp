#include <string_view>

////imu, gps, peto, engine, landing gear, pressure and temp,
// --- Subsystem Names ---
enum SubSystems: unsigned char {
    SUBSYSTEM_0,
    SUBSYSTEM_1,
};
static const unsigned char SUBSYSTEM_COUNT = 2;

std::string_view SubSystemToString(SubSystems id);