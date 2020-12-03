#include <unity.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <SPortDecoder.h>

#include "test_main.h"


static const uint8_t stream[] = {
0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,
0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,
0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,
0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,
0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,
0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,
0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,
0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,
0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,
0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,
0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,
0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,
0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,
0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,
0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,
0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,
0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0xBA,0x10,0x00,0xF0,0x00,0x00,0x00,
0x00,0x5A,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x29,0x00,0x00,0x00,0x51,0x7E,0x7E,0x98,
0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x1B,0x32,0x10,0x03,0x00,0x01,
0x2B,0x28,0x38,0x7E,0x7E,0x1B,0x10,0x00,0x04,0x12,0x27,0x00,0x00,0x3A,0x7E,0x7E,
0x1B,0x10,0x01,0x04,0x18,0x00,0x00,0x00,0x16,0x7E,0x7E,0x1B,0x10,0x10,0x02,0x4E,
0x02,0x00,0x00,0x55,0x7E,0x7E,0x1B,0x10,0x10,0x09,0x00,0x00,0x00,0x00,0x12,0x7E,
0x7E,0x1B,0x10,0x00,0x02,0x00,0x00,0x00,0x00,0x09,0x7E,0x7E,0x1B,0x10,0x00,0x06,
0x00,0x00,0x00,0x00,0x0D,0x7E,0x7E,0x1B,0x10,0x40,0x08,0x0A,0x00,0x00,0x00,0x49,
0x7E,0x7E,0x1B,0x10,0x30,0x52,0x66,0x00,0x00,0x00,0x0F,0x7E,0x7E,0x1B,0x10,0x40,
0x52,0x02,0x00,0x00,0x00,0x1B,0x7E,0x7E,0x1B,0x10,0x00,0x07,0xDA,0xFF,0xFF,0xFF,
0x29,0x7E,0x7E,0x1B,0x10,0x10,0x07,0xFD,0xFF,0xFF,0xFF,0x1E,0x7E,0x7E,0x1B,0x10,
0x20,0x07,0x56,0x00,0x00,0x00,0x7A,0x7E,0x7E,0x1B,0x10,0x00,0x04,0x22,0x4E,0x00,
0x00,0x63,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x5C,0x00,0x00,0x00,0x24,0x7E,0x7E,0x98,
0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x1B,0x10,0x01,0x04,0x18,0x00,
0x00,0x00,0x16,0x7E,0x7E,0x1B,0x10,0x10,0x02,0x71,0x03,0x00,0x00,0x6B,0x7E,0x7E,
0x1B,0x10,0x10,0x09,0x00,0x00,0x00,0x00,0x12,0x7E,0x7E,0x1B,0x10,0x00,0x02,0x00,
0x00,0x00,0x00,0x09,0x7E,0x7E,0x1B,0x10,0x00,0x06,0x00,0x00,0x00,0x00,0x0D,0x7E,
0x7E,0x1B,0x10,0x40,0x08,0x0A,0x00,0x00,0x00,0x49,0x7E,0x7E,0x1B,0x32,0x11,0x00,
0xF6,0x00,0x00,0xEC,0x22,0x7E,0x7E,0xBA,0x10,0x00,0xF0,0x00,0x00,0x00,0x00,0x5A,
0x7E,0x7E,0x1B,0x10,0x30,0x52,0x9F,0x00,0x00,0x00,0xF6,0x7E,0x7E,0x1B,0x10,0x40,
0x52,0x0A,0x00,0x00,0x00,0x13,0x7E,0x7E,0x1B,0x10,0x00,0x07,0xE2,0xFF,0xFF,0xFF,
0x11,0x7E,0x7E,0x1B,0x10,0x10,0x07,0x0F,0x00,0x00,0x00,0x13,0x7E,0x7E,0x1B,0x10,
0x20,0x07,0x60,0x00,0x00,0x00,0x4C,0x7E,0x7E,0x1B,0x32,0x12,0x02,0x04,0x01,0xBC,
0xEC,0x6C,0x7E,0x7E,0x1B,0x10,0x00,0x04,0x32,0x75,0x00,0x00,0x48,0x7E,0x7E,0x1B,
0x10,0x01,0x04,0x18,0x00,0x00,0x00,0x16,0x7E,0x7E,0x1B,0x10,0x10,0x02,0x24,0x04,
0x00,0x00,0x39,0x7E,0x7E,0x1B,0x10,0x10,0x09,0x00,0x00,0x00,0x00,0x12,0x7E,0x7E,
0x1B,0x10,0x00,0x02,0x00,0x00,0x00,0x00,0x09,0x7E,0x7E,0x1B,0x10,0x00,0x06,0x00,
0x00,0x00,0x00,0x0D,0x7E,0x7E,0x1B,0x10,0x40,0x08,0x14,0x8C,0x00,0x00,0xDB,0x7E,
0x7E,0x98,0x10,0x01,0xF1,0x64,0x00,0x00,0x00,0x1C,0x7E,0x7E,0x98,0x10,0x04,0xF1,
0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x1B,0x10,0x30,0x52,0xBD,0x00,0x00,0x00,0xD4,
0x7E,0x7E,0x1B,0x10,0x40,0x52,0x3F,0x00,0x00,0x00,0x26,0x7E,0x7E,0x1B,0x10,0x00,
0x07,0xD7,0xFF,0xFF,0xFF,0x24,0x7E,0x7E,0x1B,0x10,0x10,0x07,0x13,0x00,0x00,0x00,
0x0F,0x7E,0x7E,0x1B,0x10,0x20,0x07,0x5B,0x00,0x00,0x00,0x77,0x7E,0x7E,0x1B,0x10,
0x00,0x04,0x12,0x27,0x00,0x00,0x3A,0x7E,0x7E,0x1B,0x10,0x01,0x04,0x18,0x00,0x00,
0x00,0x16,0x7E,0x7E,0x1B,0x10,0x10,0x02,0x70,0x04,0x00,0x00,0x6D,0x7E,0x7E,0x1B,
0x10,0x10,0x09,0x00,0x00,0x00,0x00,0x12,0x7E,0x7E,0x1B,0x10,0x00,0x02,0x00,0x00,
0x00,0x00,0x09,0x7E,0x7E,0x1B,0x10,0x00,0x06,0x00,0x00,0x00,0x00,0x0D,0x7E,0x7E,
0x1B,0x10,0x40,0x08,0x9C,0x8B,0x00,0x00,0x54,0x7E,0x7E,0x1B,0x10,0x30,0x52,0xD5,
0x00,0x00,0x00,0xBC,0x7E,0x7E,0x1B,0x10,0x40,0x52,0x61,0x00,0x00,0x00,0x78,0x7E,
0x7E,0x1B,0x32,0x13,0x00,0xBA,0x00,0x00,0x03,0x83,0x7E,0x7E,0x1B,0x10,0x00,0x07,
0xD9,0xFF,0xFF,0xFF,0x2A,0x7E,0x7E,0x1B,0x10,0x10,0x07,0x15,0x00,0x00,0x00,0x09,
0x7E,0x7E,0x1B,0x10,0x20,0x07,0x5D,0x00,0x00,0x00,0x71,0x7E,0x7E,0x1B,0x10,0x00,
0x04,0x22,0x4E,0x00,0x00,0x63,0x7E,0x7E,0x1B,0x10,0x01,0x04,0x18,0x00,0x00,0x00,
0x16,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x64,0x00,0x00,0x00,0x1C,0x7E,0x7E,0x98,0x10,
0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x1B,0x10,0x10,0x02,0x9A,0x04,0x00,
0x00,0x87,0x7E,0x7E,0x1B,0x10,0x10,0x09,0x00,0x00,0x00,0x00,0x12,0x7E,0x7E,0x1B,
0x10,0x00,0x02,0x00,0x00,0x00,0x00,0x09,0x7E,0x7E,0xBA,0x10,0x00,0xF0,0x00,0x00,
0x00,0x00,0x5A,0x7E,0x7E,0x1B,0x10,0x00,0x06,0x00,0x00,0x00,0x00,0x0D,0x7E,0x7E,
0x1B,0x10,0x40,0x08,0x42,0x8B,0x00,0x00,0x8A,0x7E,0x7E,0x1B,0x10,0x30,0x52,0xE1,
0x00,0x00,0x00,0x88,0x7E,0x7E,0x1B,0x10,0x40,0x52,0x73,0x00,0x00,0x00,0x6A,0x7E,
0x7E,0x1B,0x10,0x00,0x07,0xD6,0xFF,0xFF,0xFF,0x25,0x7E,0x7E,0x1B,0x10,0x10,0x07,
0x12,0x00,0x00,0x00,0x0E,0x7E,0x7E,0x1B,0x10,0x20,0x07,0x57,0x00,0x00,0x00,0x7B,
0x7E,0x7E,0x1B,0x10,0x00,0x04,0x32,0x75,0x00,0x00,0x48,0x7E,0x7E,0x1B,0x10,0x01,
0x04,0x18,0x00,0x00,0x00,0x16,0x7E,0x7E,0x1B,0x10,0x10,0x02,0xB0,0x04,0x00,0x00,
0xAD,0x7E,0x7E,0x1B,0x10,0x10,0x09,0x00,0x00,0x00,0x00,0x12,0x7E,0x7E,0x1B,0x10,
0x00,0x02,0x00,0x00,0x00,0x00,0x09,0x7E,0x7E,0x1B,0x10,0x00,0x06,0x00,0x00,0x00,
0x00,0x0D,0x7E,0x7E,0x1B,0x32,0x14,0x00,0xBA,0x00,0x00,0xEC,0x6B,0x7E,0x7E,0x1B,
0x10,0x40,0x08,0x42,0x8B,0x00,0x00,0x8A,0x7E,0x7E,0x1B,0x10,0x30,0x52,0xF2,0x00,
0x00,0x00,0x9B,0x7E,0x7E,0x1B,0x10,0x40,0x52,0x74,0x00,0x00,0x00,0x6D,0x7E,0x7E,
0x1B,0x10,0x00,0x07,0xD4,0xFF,0xFF,0xFF,0x27,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x63,
0x00,0x00,0x00,0x1B,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,
0x7E,0x1B,0x10,0x10,0x07,0x13,0x00,0x00,0x00,0x0F,0x7E,0x7E,0x1B,0x10,0x20,0x07,
0x59,0x00,0x00,0x00,0x75,0x7E,0x7E,0x1B,0x10,0x00,0x04,0x12,0x27,0x00,0x00,0x3A,
0x7E,0x7E,0x1B,0x10,0x01,0x04,0x18,0x00,0x00,0x00,0x16,0x7E,0x7E,0x1B,0x10,0x10,
0x02,0xBD,0x04,0x00,0x00,0xA0,0x7E,0x7E,0x1B,0x10,0x10,0x09,0x94,0x01,0x00,0x00,
0x87,0x7E,0x7E,0x1B,0x10,0x00,0x02,0x03,0x00,0x00,0x00,0x0A,0x7E,0x7E,0x1B,0x10,
0x00,0x06,0x00,0x00,0x00,0x00,0x0D,0x7E,0x7E,0x1B,0x10,0x40,0x08,0x38,0x8B,0x00,
0x00,0xF0,0x7E,0x7E,0x1B,0x10,0x30,0x52,0xFD,0x00,0x00,0x00,0x94,0x7E,0x7E,0x1B,
0x10,0x40,0x52,0x75,0x00,0x00,0x00,0x6C,0x7E,0x7E,0x1B,0x10,0x00,0x07,0xD4,0xFF,
0xFF,0xFF,0x27,0x7E,0x7E,0x1B,0x10,0x10,0x07,0x13,0x00,0x00,0x00,0x0F,0x7E,0x7E,
0x1B,0x10,0x20,0x07,0x59,0x00,0x00,0x00,0x75,0x7E,0x7E,0x1B,0x10,0x00,0x04,0x22,
0x4E,0x00,0x00,0x63,0x7E,0x7E,0x1B,0x10,0x01,0x04,0x18,0x00,0x00,0x00,0x16,0x7E,
0x7E,0x1B,0x10,0x10,0x02,0xC1,0x04,0x00,0x00,0xDC,0x7E,0x7E,0x1B,0x32,0x15,0x00,
0xBA,0x00,0x00,0xEC,0x6A,0x7E,0x7E,0x1B,0x10,0x10,0x09,0x95,0x01,0x00,0x00,0x86,
0x7E,0x7E,0xBA,0x10,0x00,0xF0,0x00,0x00,0x00,0x00,0x5A,0x7E,0x7E,0x1B,0x10,0x00,
0x02,0x03,0x00,0x00,0x00,0x0A,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x63,0x00,0x00,0x00,
0x1B,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x1B,0x10,
0x00,0x06,0x00,0x00,0x00,0x00,0x0D,0x7E,0x7E,0x1B,0x10,0x40,0x08,0x2E,0x8B,0x00,
0x00,0xE6,0x7E,0x7E,0x1B,0x10,0x30,0x52,0xFE,0x00,0x00,0x00,0x97,0x7E,0x7E,0x1B,
0x10,0x40,0x52,0x79,0x00,0x00,0x00,0x60,0x7E,0x7E,0x1B,0x10,0x00,0x07,0xD6,0xFF,
0xFF,0xFF,0x25,0x7E,0x7E,0x1B,0x10,0x10,0x07,0x14,0x00,0x00,0x00,0x08,0x7E,0x7E,
0x1B,0x10,0x20,0x07,0x59,0x00,0x00,0x00,0x75,0x7E,0x7E,0x1B,0x10,0x00,0x04,0x32,
0x75,0x00,0x00,0x48,0x7E,0x7E,0x1B,0x10,0x01,0x04,0x18,0x00,0x00,0x00,0x16,0x7E,
0x7E,0x1B,0x10,0x10,0x02,0xC4,0x04,0x00,0x00,0xD9,0x7E,0x7E,0x1B,0x10,0x10,0x09,
0x96,0x01,0x00,0x00,0x85,0x7E,0x7E,0x1B,0x10,0x00,0x02,0x01,0x00,0x00,0x00,0x08,
0x7E,0x7E,0x1B,0x10,0x00,0x06,0x00,0x00,0x00,0x00,0x0D,0x7E,0x7E,0x1B,0x10,0x40,
0x08,0x1A,0x8B,0x00,0x00,0xD2,0x7E,0x7E,0x1B,0x10,0x30,0x52,0xFC,0x00,0x00,0x00,
0x95,0x7E,0x7E,0x1B,0x10,0x40,0x52,0x7D,0x5D,0x00,0x00,0x00,0x64,0x7E,0x7E,0x1B,
0x10,0x00,0x07,0xD6,0xFF,0xFF,0xFF,0x25,0x7E,0x7E,0x1B,0x10,0x10,0x07,0x14,0x00,
0x00,0x00,0x08,0x7E,0x7E,0x1B,0x10,0x20,0x07,0x59,0x00,0x00,0x00,0x75,0x7E,0x7E,
0x1B,0x32,0x16,0x00,0xBA,0x00,0x00,0xEC,0x69,0x7E,0x7E,0x1B,0x10,0x00,0x04,0x12,
0x27,0x00,0x00,0x3A,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x65,0x00,0x00,0x00,0x1D,0x7E,
0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x1B,0x10,0x01,0x04,
0x19,0x00,0x00,0x00,0x17,0x7E,0x7E,0x1B,0x10,0x10,0x02,0xC7,0x04,0x00,0x00,0xDA,
0x7E,0x7E,0x1B,0x10,0x10,0x09,0x98,0x01,0x00,0x00,0x8B,0x7E,0x7E,0x1B,0x10,0x00,
0x02,0x03,0x00,0x00,0x00,0x0A,0x7E,0x7E,0x1B,0x10,0x00,0x06,0x00,0x00,0x00,0x00,
0x0D,0x7E,0x7E,0x1B,0x10,0x40,0x08,0x06,0x8B,0x00,0x00,0xCE,0x7E,0x7E,0x1B,0x10,
0x30,0x52,0xFB,0x00,0x00,0x00,0x92,0x7E,0x7E,0x1B,0x10,0x40,0x52,0x81,0x00,0x00,
0x00,0x98,0x7E,0x7E,0x1B,0x10,0x00,0x07,0xD6,0xFF,0xFF,0xFF,0x25,0x7E,0x7E,0x1B,
0x10,0x10,0x07,0x15,0x00,0x00,0x00,0x09,0x7E,0x7E,0x1B,0x10,0x20,0x07,0x58,0x00,
0x00,0x00,0x74,0x7E,0x7E,0x1B,0x10,0x00,0x04,0x22,0x4E,0x00,0x00,0x63,0x7E,0x7E,
0x1B,0x10,0x01,0x04,0x19,0x00,0x00,0x00,0x17,0x7E,0x7E,0x1B,0x10,0x10,0x02,0xC8,
0x04,0x00,0x00,0xD5,0x7E,0x7E,0xBA,0x10,0x00,0xF0,0x00,0x00,0x00,0x00,0x5A,0x7E,
0x7E,0x1B,0x10,0x10,0x09,0x98,0x01,0x00,0x00,0x8B,0x7E,0x7E,0x1B,0x10,0x00,0x02,
0x04,0x00,0x00,0x00,0x0D,0x7E,0x7E,0x1B,0x10,0x00,0x06,0x00,0x00,0x00,0x00,0x0D,
0x7E,0x7E,0x1B,0x10,0x40,0x08,0xFC,0x8A,0x00,0x00,0x35,0x7E,0x7E,0x1B,0x10,0x30,
0x52,0xFD,0x00,0x00,0x00,0x94,0x7E,0x7E,0x1B,0x10,0x40,0x52,0x86,0x00,0x00,0x00,
0x9F,0x7E,0x7E,0x1B,0x10,0x00,0x07,0xD6,0xFF,0xFF,0xFF,0x25,0x7E,0x7E,0x98,0x10,
0x01,0xF1,0x65,0x00,0x00,0x00,0x1D,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,
0x00,0x19,0x7E,0x7E,0x1B,0x32,0x17,0x00,0xBA,0x00,0x00,0xEC,0x68,0x7E,0x7E,0x1B,
0x10,0x10,0x07,0x17,0x00,0x00,0x00,0x0B,0x7E,0x7E,0x1B,0x10,0x20,0x07,0x59,0x00,
0x00,0x00,0x75,0x7E,0x7E,0x1B,0x10,0x00,0x04,0x32,0x75,0x00,0x00,0x48,0x7E,0x7E,
0x1B,0x10,0x01,0x04,0x19,0x00,0x00,0x00,0x17,0x7E,0x7E,0x1B,0x10,0x10,0x02,0xC9,
0x04,0x00,0x00,0xD4,0x7E,0x7E,0x1B,0x10,0x10,0x09,0x98,0x01,0x00,0x00,0x8B,0x7E,
0x7E,0x1B,0x10,0x00,0x02,0x05,0x00,0x00,0x00,0x0C,0x7E,0x7E,0x1B,0x10,0x00,0x06,
0x00,0x00,0x00,0x00,0x0D,0x7E,0x7E,0x1B,0x10,0x40,0x08,0xC0,0x8A,0x00,0x00,0x09,
0x7E,0x7E,0x1B,0x10,0x30,0x52,0xF8,0x00,0x00,0x00,0x91,0x7E,0x7E,0x1B,0x10,0x40,
0x52,0x94,0x00,0x00,0x00,0x8D,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x37,0x00,0x00,0x00,
0x4F,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,
0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,
0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,
0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,
0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,
0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,
0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,
0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,
0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,
0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,
0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,0x01,0xF1,0x00,0x00,0x00,0x00,
0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,0x00,0x19,0x7E,0x7E,0x98,0x10,
0x01,0xF1,0x00,0x00,0x00,0x00,0x78,0x7E,0x7E,0x98,0x10,0x04,0xF1,0x64,0x00,0x00,
0x00,0x19,0x7E    
};


class SPortTelemetryListener : public TelemetryListener {
public: 
    virtual void onFrameDecoded(TelemetryDecoder* decoder, uint32_t id)                         { /*printf("Frame %04X\n", id); */}
    virtual void onFrameError(TelemetryDecoder* decoder, TelemetryError error, uint32_t param)  { printf("Frame error: %d - 0x%X\n", error, param); }
    virtual void onFuelData(TelemetryDecoder* decoder, int fuel)                                { printf("Fuel       : %d\n", fuel); }
    virtual void onGPSData(TelemetryDecoder* decoder, double latitude, double longitude)        { printf("GPS        : %lf, %lf\n", latitude, longitude); }
    virtual void onVBATData(TelemetryDecoder* decoder, float voltage)                           { printf("VBat       : %.2fV\n", voltage); }
    virtual void onCellVoltageData(TelemetryDecoder* decoder, float voltage)                    { printf("Cell       : %.2fV\n", voltage); }
    virtual void onCurrentData(TelemetryDecoder* decoder, float current)                        { printf("Current    : %.2fA\n", current); }
    virtual void onHeadingData(TelemetryDecoder* decoder, float heading)                        { printf("Heading    : %.2f°\n", heading); }
    virtual void onRSSIData(TelemetryDecoder* decoder, int rssi)                                { printf("Rssi       : %ddB\n", rssi); }
    virtual void onRxBtData(TelemetryDecoder* decoder, float voltage)                           { printf("RxBt       : %.2fV\n", voltage); }
    virtual void onGPSStateData(TelemetryDecoder* decoder, int satellites, bool gpsFix)         { printf("GPSState   : %d, %d\n", satellites, gpsFix); }
    virtual void onVSpeedData(TelemetryDecoder* decoder, float vspeed)                          { printf("VSpeed     : %.2fm/s\n", vspeed); }
    virtual void onAltitudeData(TelemetryDecoder* decoder, float altitude)                      { printf("Altitude   : %.2fm\n", altitude); }
    virtual void onGPSAltitudeData(TelemetryDecoder* decoder, float altitude)                   { printf("GAlt       : %.2fm\n", altitude); }
    virtual void onDistanceData(TelemetryDecoder* decoder, int distance)                        { printf("Distance   : %.2dm\n", distance); }
    virtual void onRollData(TelemetryDecoder* decoder, float rollAngle)                         { printf("Roll       : %.2f°\n", rollAngle); }
    virtual void onPitchData(TelemetryDecoder* decoder, float pitchAngle)                       { printf("Pitch      : %.2f°\n", pitchAngle); }
    virtual void onGSpeedData(TelemetryDecoder* decoder, float speed)                           { printf("GSpeed     : %.2fm/s\n", speed); }
    virtual void onAirSpeedData(TelemetryDecoder* decoder, float speed)                         { printf("AirSpeed   : %.2fm/s\n", speed); }
} ;


static SPortDecoder    decoder;
static SPortTelemetryListener telemetry;

void test_SPortDecoder(void) {
    decoder.setTelemetryListener(&telemetry);
    for(int t= 0; t<sizeof(stream); ++t) {
        decoder.process(stream[t]);
    }
}

