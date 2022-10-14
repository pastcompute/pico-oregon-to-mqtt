/// This file declares a number of data types;
/// - information shared between the DIO2 IRQ handler and the Core1 main loop
/// - message data decoded from Manchester queued from Core1 to Core0
///
/// For performance and avoiding malloc/new, we send just the pointers through the queue, after allocate the messages
/// using a pre-allocated array of fixed sized objects. As we dont generally to expect to get overwhelmed with messages
/// (the THD122 sends 2 successive messages every 39 seconds, and we dont expect to have too many devices at once)
/// the queue and the array dont need to be huge.
/// We also adhere to C++ standard-layout semantics

#ifndef POTM_TYPES_H_
#define POTM_TYPES_H_

#include <pico/stdlib.h>
#include <algorithm>

/// Common fields for our queued messages.
/// Specific decoded types derive from this.

struct DecodedMessage_t {
  enum : uint8_t { MAX_RAW_MESSAGE_BYTES = 100 };

  // Because we are using standard-layout, we need to independently keep track of what kind of object this is
  enum BaseType_t : uint8_t { UNDEFINED = 0, UNDECODED, OREGON, BASETYPE_MAX };

  BaseType_t baseType; 

  uint64_t rawTime_us;                    ///< value of time_us_64() at start of the IRQ handler

  uint8_t len;                            ///< Number of raw decoded bytes that was decoded
  uint8_t bytes[MAX_RAW_MESSAGE_BYTES];   ///< Raw decoded bytes (may be truncated)

  bool isTruncated;                       ///< If this was an unknown message, and len > MAX_RAW_MESSAGE_BYTES

  static void init(DecodedMessage_t& obj, BaseType_t baseType, const uint8_t* bytes, uint8_t len, uint64_t t_us) {
    assert(baseType > UNDEFINED && baseType < BASETYPE_MAX);
    obj.baseType = baseType;
    obj.rawTime_us = t_us;
    obj.len = len;
    obj.isTruncated = len > MAX_RAW_MESSAGE_BYTES;
    std::copy(bytes, bytes + std::min(len, (uint8_t)MAX_RAW_MESSAGE_BYTES), obj.bytes);
  }
};

/// This stucture holds decoded Oregon protocol message data
/// The value of DecodedMessage_t::baseType should be set to BaseType_t::OREGON or this data will be ignored
struct OregonSensorData_t : public DecodedMessage_t {
  uint16_t actualType;
  uint8_t channel;
  // Note that the "rolling code" is BCD...
  uint8_t rollingCode;
  int16_t temp;
  uint8_t hum;
  bool battOK;
};

/// This next structure provides the fixed size array element used for allocating messages in the ring buffer
union DecodedMessageUnion_t {
  DecodedMessage_t base;
  OregonSensorData_t oregon;
};

/// This structure holds data shared between the dio2 interrupt handler and the second core main loop.
struct Dio2SharedData_t {
  volatile uint32_t edgesCount; ///< Statistical diagnostics - count of number of edges detected 
  volatile uint32_t nextPulseLength_us; ///< Duration between the previous and latest edge; this is cleared when read by the main loop to avoid duplication
  volatile uint64_t now; ///< raw timestamp from then the latest edge was detected
  volatile bool fallingEdge; ///< If true,  nextPulseLength_us is time signal was high, otherwise, time signal was low
};

#endif
