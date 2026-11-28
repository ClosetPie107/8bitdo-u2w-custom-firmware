/* cc -DHOST_TEST patches/elite.c patches/test_elite.c -o /tmp/test_elite */
#include <assert.h>
#include <stdint.h>
#include <string.h>

void elite_pack_report(uint8_t out[38], const uint8_t stock[33]);
int elite_decode_rumble(const uint8_t *p, uint8_t motors[2]);

int main(void) {
    uint8_t stock[33] = {
        7, 0x00, 0x80, 0xff, 0x12, 0x40, 0xff, 0xff, 0xff, 3, 0, 0, 0, 0,
        0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x21, 0x43, 0x65, 0x87,
    };
    const uint8_t expected[38] = {
        0, 0, 0x80, 0x80, 0xff, 0xff, 0x12, 0x12, 0xff, 3, 1, 1,
        8, 0xdb, 0x7c, 0, 0x0f, 0, 0, 0, 0, 0,
        0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x21, 0x43, 0x65, 0x87,
        0, 0, 0, 0,
    };
    uint8_t report[38];
    memset(report, 0xff, sizeof(report));
    elite_pack_report(report, stock);
    assert(memcmp(report, expected, sizeof(report)) == 0);

    stock[0] = 15;
    stock[7] = 0;
    stock[9] = 2;
    elite_pack_report(report, stock);
    assert(report[12] == 0 && report[16] == 1); /* R4 */
    stock[9] = 1;
    elite_pack_report(report, stock);
    assert(report[16] == 4); /* L4 */
    stock[9] = 0;
    stock[7] = 4;
    elite_pack_report(report, stock);
    assert(report[16] == 2); /* PR */
    stock[7] = 32;
    elite_pack_report(report, stock);
    assert(report[16] == 8); /* PL */

    uint8_t packet[17] = { [2] = 11, [9] = 15, 25, 9, 0, 100, 10, 0, 0 };
    uint8_t motors[2];
    assert(elite_decode_rumble(packet, motors));
    assert(motors[0] == 128 && motors[1] == 255);
    packet[16] = 0xeb;
    packet[13] = 255;
    assert(elite_decode_rumble(packet, motors));
    assert(motors[0] == 64 && motors[1] == 255);
    packet[2] = 7;
    assert(!elite_decode_rumble(packet, motors));
    return 0;
}
