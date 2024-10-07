#include <sensor.h>

class BNO055 : Sensor {
public:
    BNO055();
private:

};

BNO055::BNO055() : Sensor("BNO055_Node") {

}

Sensor::Sensor(std::string name) : Node(name) {

};