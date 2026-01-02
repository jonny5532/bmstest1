#pragma GCC optimize ("Os")

#include <stdint.h>

const uint16_t CRC14_POLYNOMIAL = 0x025b;

// Weird Batman CRC14

uint16_t crc14(uint8_t *data, int len, uint16_t initial_crc) {
    // CRC'd received data has an initial value of 0x1000 (ie, it effectively
    // has a leading zero byte). For tx use an initial value of 0x0010.

    uint16_t crc = initial_crc;

    void shift() {
        if(crc & 0x2000) {
            crc = (uint16_t)((crc << 1) ^ CRC14_POLYNOMIAL);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }

    for(int i=0; i<len; i++) {
        uint8_t c = data[i];
        // XOR byte into the top bits
        crc ^= (uint16_t)(c << 6);
        // Shift 8 times
        for(int j=0; j<8; j++) {
            shift();
        }
    }

    // Two extra zero bits
    shift();
    shift();

    return crc & 0x3fff;
}

/*
def crc14(data, initial):
    crc = initial
    poly = 0x025B

    for byte in data:
        crc ^= (byte << 6)
        for _ in range(8):
            if crc & 0x2000:
                crc = (crc << 1) ^ poly
            else:
                crc = crc << 1
            crc &= 0x3FFF

    # Two extra zero bits
    for _ in range(2):
        if crc & 0x2000:
            crc = (crc << 1) ^ poly
        else:
            crc = crc << 1
        crc &= 0x3FFF

    return crc & 0x3FFF
*/