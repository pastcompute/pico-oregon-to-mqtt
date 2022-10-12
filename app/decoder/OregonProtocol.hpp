#include <pico/stdlib.h>

class OregonProtocol {
public:

  static int checksum(uint8_t count, const uint8_t* buffer) {
    int s = 0;
    for(uint8_t i = 0; i < count; i++) {
        s += (buffer[i]&0xF0) >> 4;
        s += (buffer[i]&0xF);
    }
    if(int(count) != count) {
      s += (buffer[count]&0xF0) >> 4;
    }
    return s;
  }
  static bool checksumPasses(uint16_t sensorType, const uint8_t* data, int len) {
    uint8_t s1 = 0;
    uint8_t s2 = 0;
    switch (sensorType) {
    case 0x1A2D:                                // THGR2228N
      s1 = (checksum(8, data) - 0xa) & 0xFF;
      return (data[8] == s1);
    case 0xEA4C:                                // TNHN132N
      s1 = (checksum(6, data) + (data[6]&0xF) - 0xa) & 0xff;
      s2 = (s1 & 0xF0) >> 4;
      s1 = (s1 & 0x0F) << 4;
      return ((s1 == data[6]) && (s2 == data[7]));
    default:
      break;
    }
    return false;
  }

  // This will not alter any outputs unless the length and checksum pass
  static bool decodeTempHumidity(const uint8_t* data, int len, uint16_t& actualType, uint8_t& channel, uint8_t& rollingCode, int16_t& temp, uint8_t& hum, bool& battOK) {
    if (len < 8) { return false; }
    const uint16_t sensorType = (data[0] << 8) | data[1];
    if (!checksumPasses(sensorType, data, len)) { return false; }
    int16_t t = data[5] >> 4;                   // 1st decimal digit
    t *= 10;
    t += data[5] & 0x0F;                        // 2nd decimal digit
    t *= 10;
    t += data[4] >> 4;                          // 3rd decimal digit
    if (data[6] & 0x08) { t *= -1; }
    temp = t;
    hum = 0;
    battOK = !(data[4] & 0x0C);
    if (sensorType == 0x1A2D) {                 // THGR228N, THGN123N, THGR122NX, THGN123N
      hum  = data[7] & 0xF;
      hum *= 10;
      hum += data[6] >> 4;
    }
    channel = data[2] >> 4;
    rollingCode = 
      ((data[3] & 0xf) << 4) |     // 2
      ((data[3] >> 4));            // 7

    // sensorType 1a2D shows as 1d20 in Pulseview... for the exact same data...
    // we can see where that comes from here...
    actualType = (uint16_t(data[0] >> 4) << 12) |     // 1
      (uint16_t(data[1] & 0xf) << 8) |     // d
      (uint16_t(data[1] >> 4) << 4)  |     // 2
      (uint16_t(data[2]) & 0xf);           // 0

    return true;
  }
};
