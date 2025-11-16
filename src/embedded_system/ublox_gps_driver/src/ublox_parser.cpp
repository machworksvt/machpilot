#include "ublox_gps_driver/ublox_parser.hpp"
#include <cstring>
#include <iostream>

namespace ublox {

    UbloxParser::UbloxParser()
        : parser_state_(ParserState::SYNC1),
          msg_class_(0),
          msg_id_(0),
          payload_length_(0),
          payload_index_(0),
          checksum_a_(0),
          checksum_b_(0){
            payload_.reserve(256);
          }

    void UbloxParser::reset() {
        parser_state_ = ParserState::SYNC1;
        payload_.clear();
        payload_index_ = 0;
    }

    void UbloxParser::processByte(uint8_t byte) {
        #ifdef GPS_DEBUG_MODE
            static int byte_count = 0;
            byte_count++;
            
            if (byte_count % 1000 == 0) {
                std::cout << "Processed " << byte_count << " bytes, state=" << (int)parser_state_ << std::endl;
            }
        #endif


        switch (parser_state_) {
            case ParserState::SYNC1:
                if (byte == SYNC_CHAR_1) {
                    parser_state_ = ParserState::SYNC2;
                    #ifdef GPS_DEBUG_MODE
                        std::cout << "Found SYNC2 (0xB5)!" << std::endl;
                    #endif 
                }
                break;
            
            case ParserState::SYNC2:
                if (byte == SYNC_CHAR_2) {
                    parser_state_ = ParserState::CLASS;
                    #ifdef GPS_DEBUG_MODE
                        std::cout << "Found SYNC2 (0x62)!" << std::endl;
                    #endif 
                } 
                else {
                    reset();
                }
                break;

            case ParserState::CLASS:
                msg_class_ = byte;
                parser_state_ = ParserState::ID;
                break;

            case ParserState::ID:
                msg_id_ = byte;
                parser_state_ = ParserState::LENGTH1;
                break;

            case ParserState::LENGTH1:
                payload_length_ = byte;
                parser_state_ = ParserState::LENGTH2;
                break;

            case ParserState::LENGTH2:
                payload_length_ |= (static_cast<uint16_t>(byte) << 8);
                payload_.clear();
                payload_index_ = 0;

                std::cout << "MSG: Class=0x" << std::hex << (int)msg_class_ 
                      << " ID=0x" << (int)msg_id_ 
                      << " Len=" << std::dec << payload_length_ << std::endl;

                if (payload_length_ == 0) {
                    parser_state_ = ParserState::CHECKSUM1;
                }
                else {
                    parser_state_ = ParserState::PAYLOAD;
                }

                break;

            case ParserState::PAYLOAD:
                payload_.push_back(byte);
                payload_index_++;

                if (payload_index_ >= payload_length_) {
                    parser_state_ = ParserState::CHECKSUM1;
                }
                break;

            case ParserState::CHECKSUM1:
                checksum_a_ = byte;
                std::cout << "  CK_A: 0x" << std::hex << (int)byte << std::dec << std::endl;
                parser_state_ = ParserState::CHECKSUM2;
                break;
                
            case ParserState::CHECKSUM2:
                checksum_b_ = byte;
                std::cout << "  CK_B: 0x" << std::hex << (int)byte << std::dec << std::endl;
                
                // Build checksum data: class, id, length, payload
                std::vector<uint8_t> cksum_data;
                cksum_data.push_back(msg_class_);
                cksum_data.push_back(msg_id_);
                cksum_data.push_back(payload_length_ & 0xFF);
                cksum_data.push_back((payload_length_ >> 8) & 0xFF);
                cksum_data.insert(cksum_data.end(), payload_.begin(), payload_.end());
                
                // Calculate expected checksum
                auto [calc_a, calc_b] = calculateChecksum(cksum_data.data(), cksum_data.size());
                
                std::cout << "  Checksum verify: RX=(" << std::hex 
                        << (int)checksum_a_ << "," << (int)checksum_b_
                        << ") CALC=(" << (int)calc_a << "," << (int)calc_b 
                        << ")" << std::dec;
                
                if (calc_a == checksum_a_ && calc_b == checksum_b_) {
                    std::cout << " Passsed!" << std::endl;
                    processCompleteMessage();
                } else {
                    std::cout << " Failed." << std::endl;
                }
                
                reset();
                break;  
        }
    }
    
    void UbloxParser::processCompleteMessage() {
        if (msg_class_ == CLASS_NAV && msg_id_ == NAV_PVT) {
            if (payload_.size() >= sizeof(NavPVT)) {
                NavPVT pvt; 
                std::memcpy(&pvt, payload_.data(), sizeof(NavPVT));
            
                if (nav_pvt_callback_) {
                    nav_pvt_callback_(pvt);
                }
            }
        }

    }

}