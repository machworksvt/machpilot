#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>

namespace ublox {
    
    // Frame constants
    constexpr uint8_t SYNC_CHAR_1 = 0xB5;
    constexpr uint8_t SYNC_CHAR_2 = 0x62;

    // Message classes
    constexpr uint8_t CLASS_NAV = 0x01;
    constexpr uint8_t CLASS_CFG = 0x06;
    constexpr uint8_t CLASS_ACK = 0x05;

    // NAV Message IDs
    constexpr uint8_t NAV_PVT = 0x07;
    constexpr uint8_t NAV_STATUS = 0x03;
    constexpr uint8_t NAV_SAT = 0x35;

    // CFG Message IDs
    constexpr uint8_t CFG_PRT = 0x00;
    constexpr uint8_t CFG_MSG = 0x01;
    constexpr uint8_t CFG_RATE = 0x08;

    // ACK Message IDs
    constexpr uint8_t ACK_ACK = 0x01;
    constexpr uint8_t ACK_NAK = 0x00;

    struct UBXHeader {
        uint8_t sync1;
        uint8_t synx2;
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t length;
    };

    struct NavPVT {
    uint32_t iTOW;        // GPS time of week (ms)
    uint16_t year;        // Year (UTC)
    uint8_t month;        // Month (1-12)
    uint8_t day;          // Day (1-31)
    uint8_t hour;         // Hour (0-23)
    uint8_t min;          // Minute (0-59)
    uint8_t sec;          // Second (0-60)
    uint8_t valid;        // Validity flags
    uint32_t tAcc;        // Time accuracy estimate (ns)
    int32_t nano;         // Fraction of second (-1e9 to 1e9)
    uint8_t fixType;      // GNSS fix type
    uint8_t flags;        // Fix status flags
    uint8_t flags2;       // Additional flags
    uint8_t numSV;        // Number of satellites used
    int32_t lon;          // Longitude (deg * 1e-7)
    int32_t lat;          // Latitude (deg * 1e-7)
    int32_t height;       // Height above ellipsoid (mm)
    int32_t hMSL;         // Height above MSL (mm)
    uint32_t hAcc;        // Horizontal accuracy estimate (mm)
    uint32_t vAcc;        // Vertical accuracy estimate (mm)
    int32_t velN;         // North velocity (mm/s)
    int32_t velE;         // East velocity (mm/s)
    int32_t velD;         // Down velocity (mm/s)
    int32_t gSpeed;       // Ground speed (mm/s)
    int32_t headMot;      // Heading of motion (deg * 1e-5)
    uint32_t sAcc;        // Speed accuracy estimate (mm/s)
    uint32_t headAcc;     // Heading accuracy estimate (deg * 1e-5)
    uint16_t pDOP;        // Position DOP (0.01)
    uint8_t reserved1[6]; // Reserved
    int32_t headVeh;      // Heading of vehicle (deg * 1e-5)
    int16_t magDec;       // Magnetic declination (deg * 1e-2)
    uint16_t magAcc;      // Magnetic declination accuracy (deg * 1e-2)
} __attribute__((packed));

enum class FixType : uint8_t {
    NO_FIX = 0,
    DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    GNSS_DEAD_RECKONING = 4,
    TIME_ONLY = 5
};

std::pair<uint8_t, uint8_t> calculateChecksum(
    const uint8_t* data, size_t length);
    
bool verifyChecksum(const uint8_t* data, size_t length,
    uint8_t ck_a, uint8_t ck_b);
}