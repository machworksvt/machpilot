#pragma once

#include "ublox_messages.hpp"
#include <vector>
#include <functional>

namespace ublox {

    class UbloxParser {
    public:
        using NavPVTCallback = std::function<void(const NavPVT&)>;

        UbloxParser();

        void processByte(uint8_t byte);

        void setNavPVTCallback(NavPVTCallback callback) {
            nav_pvt_callback_ = callback;
        }

    private:
        enum class ParserState {
            SYNC1,
            SYNC2,
            CLASS,
            ID,
            LENGTH1,
            LENGTH2,
            PAYLOAD,
            CHECKSUM1,
            CHECKSUM2
        };

        void processCompleteMessage();
        void reset();

        // Parser state
        ParserState parser_state_;
        uint8_t msg_class_;
        uint8_t msg_id_;
        uint16_t payload_length_;
        uint16_t payload_index_;
        std::vector<uint8_t> payload_;
        uint8_t checksum_a_;
        uint8_t checksum_b_;
        
        // Callbacks
        NavPVTCallback nav_pvt_callback_;
    }; 

}
