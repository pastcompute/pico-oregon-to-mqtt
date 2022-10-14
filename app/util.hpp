#ifndef POTM_UTIL_H_
#define POTM_UTIL_H_

#include <pico/stdlib.h>

inline uint8_t fliplr(uint8_t b) {
  b = (b & 0b11110000) >> 4 | (b & 0b00001111) << 4;
  b = (b & 0b11001100) >> 2 | (b & 0b00110011) << 2;
  b = (b & 0b10101010) >> 1 | (b & 0b01010101) << 1;
  return b;
}

inline void printBinary(uint8_t number, bool flip=false)
{
  for (int b=0; b < 8; b++) {
    auto shift = flip ? b : 7-b; // this is relave to printing the same thing with %02x
    printf("%c", ((number>>shift) & 1) ? '1' : '0');
  }
}

inline void dumpMessageBinary(const uint8_t* data, uint8_t len, bool flip=false) {
  for (uint8_t i = 0; i < len; i++) {
    printBinary(data[i], flip); printf(" ");
  }
  printf("\n");
}

inline void dumpMessageHex(const uint8_t* data, uint8_t len, bool flip=false) {
  for (uint8_t i = 0; i < len; i++) {
    uint8_t v = data[i];
    printf("%02x", flip ? fliplr(v) : v);
  }
  printf("\n");
}

#endif
