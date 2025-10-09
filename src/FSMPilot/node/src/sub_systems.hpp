
#include <memory>

////imu, gps, peto, engine, landing gear, pressure and temp,
// --- Subsystem Names ---
enum SubSystems: size_t {
    SUBSYSTEM_0,
    SUBSYSTEM_1,
};
static const size_t SUBSYSTEM_COUNT = 2;

std::string sub_system_to_String(SubSystems id);