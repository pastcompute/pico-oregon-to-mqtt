#ifndef POTM_MANCHESTER_H_
#define POTM_MANCHESTER_H_

#include "decoder/DecodeOOK.hpp"
#include "decoder/OregonDecoderV2.hpp"
#include "decoder/OregonProtocol.hpp"

/// This class can try and detect and decode messages from one or more OOK Manchester encodings

class ManchesterHandler {
private:
  OregonDecoderV2 oregonV2Decoder_;
  OregonProtocol protocol_;

public:
  bool debugMessageHex;

  ManchesterHandler() { }

  /// This should be called from the main loop with the most recent pulse length in microseconds.
  /// This class maintains state, so this method when called consecutively for every edge detected,
  /// it will eventually detect and decode an entire frame if received.
  /// When a frame was succesfully decoded, it will be stored into the passed msg reference.
  /// @return  false if no completed frame was detected; true if a message completed decoding
  bool nextPulse(uint32_t pulseLength_us, uint64_t now_us, DecodedMessageUnion_t& msg) {
    // Try each decoder in turn, see if we have captured a complete message
    if (oregonV2Decoder_.nextPulse(pulseLength_us)) {
      // OK, see if we can decode it to something we know
      // Whether we can or not, this goes on the queue, because we want to output hex of unknown messages
      if (!decodeV2(now_us, msg)) {
        // it was either scrambled or something we dont understand yet
        // we would expect value to be BaseType_t::UNKNOWN
      }
      // make ready for next messge
      oregonV2Decoder_.resetDecoder();
      return true;
    }
    // The pulse did not complete a frame
    return false;
  }

private:

  void dumpMessageHex(const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
      printf("%02x", data[i]);
    }
    printf("\n");
  }

  bool decodeV2(uint64_t now_us, DecodedMessageUnion_t& msg) {
    uint8_t len;
    auto data = oregonV2Decoder_.getData(len);
    if (!data) { return false; }
    if (debugMessageHex) { dumpMessageHex(data, len); }

    // Prepare the common data and initialise to unknown
    DecodedMessage_t::init(msg.base, DecodedMessage_t::BaseType_t::UNDECODED, data, len, now_us);

    // Try for a temperature / humidity sensor
    if (protocol_.decodeTempHumidity(data, len,
          msg.oregon.actualType, msg.oregon.channel, msg.oregon.rollingCode,
          msg.oregon.temp, msg.oregon.hum, msg.oregon.battOK)) {
      // yes, it is
      msg.base.baseType = DecodedMessage_t::BaseType_t::OREGON;
      return true;
    }
    return false;
  }
};

#endif
