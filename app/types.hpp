/// This file declares a number of data types;
/// - information shared between the DIO2 IRQ handler and the Core1 main loop
/// - message data decoded from Manchester queued from Core1 to Core0
///
/// For performance and avoiding malloc/new, we send just the pointers through the queue, after allocate the messages
/// using a pre-allocated array of fixed sized objects. As we dont generally to expect to get overwhelmed with messages
/// (the THD122 sends 2 successive messages every 39 seconds, and we dont expect to have too many devices at once)
/// the queue and the array dont need to be huge.
///
/// For message types (in the ring buffer) we adhere to C++ standard-layout semantics (no virtual, etc), to ensure when selecting
/// a message out of the union used for fixed memory sizing the base type data is in the correct memory location

#ifndef POTM_TYPES_H_
#define POTM_TYPES_H_

#include <pico/stdlib.h>
#include <algorithm>

/// Oregon-specific protocol message data
/// The value of DecodedMessage_t::baseType should be set to BaseType_t::OREGON
struct OregonSensorData_t {
  uint16_t actualType;
  uint8_t channel;
  // Note that the "rolling code" is BCD...
  uint8_t rollingCode;
  int16_t temp;
  uint8_t hum;
  bool battOK;
};

/// Lacrosse-specific protocol message data
/// The value of DecodedMessage_t::baseType should be set to BaseType_t::LACROSSE
struct LacrosseSensorData_t {
  uint8_t id;
  uint8_t channel;
  int16_t temp;
  bool battOK;
};

/// Common fields for our queued messages.
/// Specific decoded types derive from this.

struct DecodedMessage_t {
  enum : uint8_t { MAX_RAW_MESSAGE_BYTES = 100 };

  enum BaseType_t : uint8_t { UNDEFINED = 0, UNDECODED, OREGON, LACROSSE, BASETYPE_MAX };

  BaseType_t baseType; 

  uint64_t rawTime_us;                    ///< value of time_us_64() at start of the IRQ handler for the last pulse when
                                          ///< the message was complete
  uint8_t len;                            ///< Number of raw decoded bytes that was decoded
  uint8_t bytes[MAX_RAW_MESSAGE_BYTES];   ///< Raw decoded bytes (may be truncated)

  bool isTruncated;                       ///< If this was an unknown message, and len > MAX_RAW_MESSAGE_BYTES
  float rssi;                             ///< RSSI associated with the message, as peak in last ~12ms after the preamble
  uint32_t uniqueCount;                   ///< Ordered number of this message for the given type, added after deduplication

  // This union ensures that each slot in the ring buffer will be the same size and large enough to hold any type
  // Whether these hold data depends on the value of baseType
  union {
    OregonSensorData_t oregon;
    LacrosseSensorData_t lacrosse;
  };

  static DecodedMessage_t makeUndefined() { DecodedMessage_t o = { .baseType = UNDEFINED, .rawTime_us = 0 }; return o; }

  static void init(DecodedMessage_t& obj, BaseType_t baseType, const uint8_t* bytes, uint8_t len, uint64_t t_us) {
    assert(baseType > UNDEFINED && baseType < BASETYPE_MAX);
    obj.baseType = baseType;
    obj.rawTime_us = t_us;
    obj.len = len;
    obj.isTruncated = len > MAX_RAW_MESSAGE_BYTES;
    obj.rssi = -128.F;
    obj.uniqueCount = 0;
    std::copy(bytes, bytes + std::min(len, (uint8_t)MAX_RAW_MESSAGE_BYTES), obj.bytes);
  }

// Check if a message is one of a series of duplicate messages sent in a burst
// Note, this will get confused in the unlikely event we get to contemporaneous bursts of
// messages from different sensors. We could in theory fix this using a hashmap for recent
// with a key of the basetype... but for the common case this gets the job done
static bool isRecentDupe(const DecodedMessage_t& next, uint32_t interval_us) {
  static auto recent = DecodedMessage_t::makeUndefined();
  auto dt = next.rawTime_us - recent.rawTime_us;
  bool dupe = false;
  if (dt < interval_us && recent.baseType != DecodedMessage_t::BaseType_t::UNDEFINED) {
    // prior message is within 0.5 second, probably duplicate.
    // But a better check is also to compare the data, so do that to be sure
    dupe = next.len == recent.len && std::equal(next.bytes, next.bytes + next.len, recent.bytes);
  }
  if (!dupe) {
    recent = next;
  }
  return dupe;
}

};

static_assert(std::is_standard_layout<DecodedMessage_t>::value);

/// This structure holds data shared between the dio2 interrupt handler and the second core main loop.
struct Dio2SharedData_t {
  volatile uint32_t edgesCount; ///< Statistical diagnostics - count of number of edges detected 
  volatile uint32_t nextPulseLength_us; ///< Duration between the previous and latest edge; this is cleared when read by the main loop to avoid duplication
  volatile uint64_t now; ///< raw timestamp from then the latest edge was detected
  volatile bool fallingEdge; ///< If true,  nextPulseLength_us is time signal was high, otherwise, time signal was low
};

#endif
