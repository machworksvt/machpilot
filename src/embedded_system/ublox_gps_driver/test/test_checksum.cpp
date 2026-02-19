#include <gtest/gtest.h>
#include "ublox_gps_driver/ublox_messages.hpp"

TEST(UbloxMessages, CalculateChecksum) {
    uint8_t data[] = {0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01};

    auto [ck_a, ck_b] = ublox::calculateChecksum(data, 7);

    EXPECT_EQ(ck_a, 0x13);
    EXPECT_EQ(ck_b, 0x51);
}

TEST(UbloxMessages, VerifyChecksumValid) {
    uint8_t data[] = {0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01};
    
    bool valid = ublox::verifyChecksum(data, 7, 0x13, 0x51);
    
    EXPECT_TRUE(valid);
}

TEST(UbloxMessages, VerifyChecksumInvalid) {
    uint8_t data[] = {0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01};
    
    bool valid = ublox::verifyChecksum(data, 7, 0x00, 0x00);
    
    EXPECT_FALSE(valid);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}