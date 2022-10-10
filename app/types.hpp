#ifndef POTM_TYPES_H_
#define POTM_TYPES_H_

#include <pico/stdlib.h>
#include <variant>

/// This structure holds unknown message bytes decoded from Manchester
struct UnknownSensorData_t {
  enum { MAX_MESSAGE_BYTES = 25 };
  uint64_t t_us;
  uint8_t len;
  uint8_t bytes[MAX_MESSAGE_BYTES];
  bool is_truncated;

  // We need to avoid methods so we can make our variant below
  static UnknownSensorData_t Create(const uint8_t* data, uint8_t len, uint64_t t_us) {
    UnknownSensorData_t o; 
    o.t_us = t_us;
    o.len = len;
    o.is_truncated = false; 
    if (len > UnknownSensorData_t::MAX_MESSAGE_BYTES) {
      o.len = UnknownSensorData_t::MAX_MESSAGE_BYTES;
      o.is_truncated = true;
    }    
    std::copy(data, data + len, o.bytes);
    return o;
  } 
};

/// This stucture holds decoded Oregon protocol message data
struct OregonSensorData_t {
  uint64_t t_us;
  uint8_t channel;
  uint8_t rollingCode;
  int16_t temp;
  uint8_t hum;
  uint16_t actualType;
  bool battOK;
};

/// Custom variant type for queueing messages
/// We can't use C++17 Variant because we need to post the raw data
/// Something just smells but I cant quite make this neat and elegant
/// - std::variant is not a POD
/// - if we use inheritence, again, not a POD
struct ReceivedMessage_t {
  enum ReceivedMessageType { INVALID=0, UNKNOWN, OREGON };
  ReceivedMessageType type;

  ReceivedMessage_t() : type(INVALID) {}
  ReceivedMessage_t(const UnknownSensorData_t& src) : type(UNKNOWN) { value.unknown = src; }
  ReceivedMessage_t(const OregonSensorData_t& src) : type(OREGON) { value.oregonSensor = src; }
  union X {
    UnknownSensorData_t unknown;
    OregonSensorData_t oregonSensor;
  } value;
  bool isValid() const { return type != INVALID; }
};

/// This structure holds data shared between the dio2 interrupt handler and the second core main loop.
struct Dio2SharedData_t {
  volatile uint32_t edgesCount; ///< Statistical diagnostics - count of number of edges detected 
  volatile uint32_t nextPulseLength_us; ///< Duration between the previous and latest edge; this is cleared when read by the main loop to avoid duplication
  volatile uint64_t now; ///< raw timestamp from then the latest edge was detected
};


#endif
