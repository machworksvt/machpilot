#include "ublox_gps_driver/ublox_messages.hpp"

namespace ublox {

    std::pair<uint8_t, uint8_t> calculateChecksum(
        const uint8_t* data,
        size_t length
    ){
        uint8_t ck_a = 0;
        uint8_t ck_b = 0;
        for (size_t i = 0; i < length ) {
            ck_a = (ck_a + data[i]) & 0xFF;
            ck_b = (ck_b + ck_a) & 0xFF;
        }

        return {ck_a, ck_b};
    }


}