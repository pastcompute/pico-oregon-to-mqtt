/// This pico program will detect and decode Oregon weather sensor messages
/// and output the result on the serial port / USB serial port
/// in a form easily ingested to MQTT or other home automation systems
///
/// The main core configures the RFM69 module SX1231 radio chip,
/// monitors the RSSI, decodes the oregon message protocol and outputs
/// this to the serial port.
///
/// The second core processes rising and falling edge interrupts on the demodulated
/// OOK signal output by the SX1231 on dio2, and uses the time between edges
/// to detect and decode Manchester-encoded binary and the Oregon protocol
/// sending the messages to the other core for output.
///
/// My unproven hypothesis is that dedicating the second core to timing and
/// mnchester detection may reduce the likelihood of a dropped
/// message when the device is communicating.

#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <pico/util/queue.h>
#include <algorithm>

#include "decoder/DecodeOOK.hpp"
#include "decoder/OregonDecoderV2.hpp"
#include "decoder/OregonProtocol.hpp"
#include "constants.h"
#include "config.h"
#include "types.hpp"
#include "ringbuf.hpp"
#include "radio.hpp"

/// Set this to true to have the second core dump decoded manchester in hex to the serial port.
static bool debugMessageHex = false;

/// Message queue used to transfer decoded Manchester OOK from the second core to the first
static queue_t decodedMessagesQueue;

typedef RingBuffer<DecodedMessageUnion_t, 5> RingBuffer_t;

/// Data store for the message queue
static RingBuffer_t decodedMessagesRingBuffer;

/// This critical section protects data shared between the dio2 interrupt handler and the second core main loop.
static critical_section_t dio2_crit;

/// This data is shared between the dio2 interrupt handler and the second core main loop
static Dio2SharedData_t sharedData;

/// RFM69 SX1231 dio2 Interrupt handler
extern "C" void dio2InterruptHandler() {
  // this may not handle wrap, but I think we can ignore that for the moment
  // it means we may get one edge with a massive size, at 2^64/10^6 seconds after power on
  // and we can live with corrupting potentially one message on such a timeframe
  auto now = time_us_64();

  if (gpio_get_irq_event_mask(RFM69_DIO2) & GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL) {
      gpio_acknowledge_irq(RFM69_DIO2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
  }

  // From the second and successive interrupt, sharedData.nextPulseLength_us will hold the time between successive interrupts
  // and local prevTime_us will track what micros() was
  // sharedData.nextPulseLength_us is cleared after being processed in the loop
  static uint64_t prevTime_us = 0;
  critical_section_enter_blocking(&dio2_crit);
  sharedData.now = now;
  sharedData.nextPulseLength_us = now - prevTime_us;
  sharedData.edgesCount++;
  critical_section_exit(&dio2_crit);
  prevTime_us = now;
}

/// This class can try and detect and decode messages from one or more OOK Manchester encodings

class ManchesterHandler {
private:
  OregonDecoderV2 oregonV2Decoder_;
  OregonProtocol protocol_;

public:
  ManchesterHandler()
  {}

  /// This should be called from the main loop ith the most recent pulse length in microseconds.
  /// This class maintains state, so this method when called consecutively for every edge detected
  /// will eventually detect and decode an entire message if received.
  /// When a message was succesfully decoded it may be accessed as the current head of the ring buffer
  /// @return  false if no entire message was detected or the ring buffer was full; true if a message completed decoding
  bool nextPulse(uint32_t pulseLength_us, uint64_t now_us) {
    // Try each decoder in turn, see if we have captured a complete message
    bool ok = false;
    if (oregonV2Decoder_.nextPulse(pulseLength_us)) {
      // OK, see if we can decode it to something we know.
      // Whether we can or not, this goes on the queue, because we want to output hex of unknown messages

      auto element = decodedMessagesRingBuffer.reserve();
      if (element) {
        if (!decodeV2(now_us, *element)) {
          // it was either scrambled or something we dont understand yet
          // we would expect value to be BaseType_t::UNKNOWN
        }
        decodedMessagesRingBuffer.advance();
        ok = true;
      } else {
        // queue full! all we can do is discard
      }
      // make ready for next messge
      oregonV2Decoder_.resetDecoder();
      return ok;
    }
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

/// Entry point for second core.
static void core1_main() {
  printf("Core1: OOK Demodulator\n");

  sharedData = { 0, 0, 0 };
  critical_section_init(&dio2_crit);

  // Interrupt on the pin connected to DIO2, with pulldown
  gpio_init(RFM69_DIO2);
  gpio_pull_down(RFM69_DIO2);
  gpio_set_dir(RFM69_DIO2, false);
  gpio_set_input_enabled(RFM69_DIO2, true);
  gpio_add_raw_irq_handler(RFM69_DIO2, &dio2InterruptHandler);
  gpio_set_irq_enabled(RFM69_DIO2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  irq_set_enabled(IO_IRQ_BANK0, true);

  // Initialise the manchester detector/decoders
  ManchesterHandler manchesterHandler;
  while (true) {
    // Idle at low power when not procesing
    __wfi();

    // Pickup the next data if there is any    
    critical_section_enter_blocking(&dio2_crit);
    auto pulseLength_us = sharedData.nextPulseLength_us;
    sharedData.nextPulseLength_us = 0;
    auto now_us = sharedData.now;
    critical_section_exit(&dio2_crit);

    if (pulseLength_us < 1) {
      // this is a failsafe; as a general rule, because we use wfi, we should not
      // get here until there has been an edge. However, if we decide to process
      // additional interrupts, then we will get here.
      continue;
    }

    // Every edge, try and decode a message
    // We have around 400 uS to get this done, worst case, when processing valid Oregon messages
    // The decoder fails as soon as it can

    bool decoded = manchesterHandler.nextPulse(pulseLength_us, now_us);
    if (decoded) {
      // push the decoded msg back to the other core...
      // printf("%d,%04x,%d,%x,%.1f,%d,Batt=%s,FIXMEdB\n", result.t, 
      //   result.actualType, result.channel, result.rollingCode, result.temp / 10.F, result.hum, result.battOK?"ok":"flat");
      //auto message = manchesterHandler.getLastDecodedMessage();

      // OK this is hacky; using our lock free queue we dont even need the Pico Queue, we just need some way to wake the other core
      int flag = 1;
      queue_try_add(&decodedMessagesQueue, &flag); // this is lossy, but thats ok too
    }
  }
}

// On Core0, loop and measure RSSI and run the decoder when there is a new pulse
void core0_main(RFM69Radio& radio) {
  const int rssiPoll_us = 100;
  auto nextOutput_us = ONE_SECOND_US /4;

  absolute_time_t tNow = get_absolute_time();
  absolute_time_t tNextOutput = delayed_by_us(tNow, nextOutput_us);
  absolute_time_t tNextPoll = delayed_by_us(tNow, rssiPoll_us);
  uint8_t rssi = 0;

  // We expect around 10000 samples per output, with a value E 60, 127 s a float should be sufficient
  int32_t runningSum = 0;
  int runningCount = 0;
  int periods = 0;
  uint8_t floorRssi = 0;
  float longTermMean = +1;
  auto t0 = to_ms_since_boot(get_absolute_time());
  while (true) {

    int flag;
    if (queue_try_remove(&decodedMessagesQueue, &flag)) {
      assert(flag == 1);
      auto tail = decodedMessagesRingBuffer.peek();
      if (tail.has_value()) {
        auto element = tail.value();
        decodedMessagesRingBuffer.pop();
        absolute_time_t t;
        switch (element.base.baseType) {
          case DecodedMessage_t::BaseType_t::OREGON: {
            OregonSensorData_t& od = element.oregon;  
            update_us_since_boot(&t, od.rawTime_us);
            printf("%d,%04x,%d,%x,%.1f,%d,Batt=%s,FIXMEdB\n", to_ms_since_boot(t), 
              od.actualType, od.channel, od.rollingCode,
              od.temp / 10.F, od.hum, od.battOK?"ok":"flat");
            break;
          }

          case DecodedMessage_t::BaseType_t::UNDECODED: {
            DecodedMessage_t& ud = element.base;
            update_us_since_boot(&t, ud.rawTime_us);
            size_t mx = ud.len * 2 + 1;
            char hexdump[mx];
            char*p = hexdump;
            for (int i=0; i < ud.len; i++) {
              p = p + snprintf(p, hexdump + mx - p, "%02x", ud.bytes[i]);
            }
            printf("%d,UNK,%s\n", to_ms_since_boot(t), hexdump); 
            break;
          }

        }
      }
    }

    tNow = get_absolute_time();
    if (time_reached(tNextPoll)) {
      tNextPoll = delayed_by_us(tNow, rssiPoll_us);
      rssi = radio.readRSSIByte();
      runningSum += rssi;
      runningCount++;
      if (rssi > floorRssi) { floorRssi = rssi; }
    }
    if (time_reached(tNextOutput)) {
      // Here we are "integrating" the received "energy" above the floor
      // Of course RSSI is dB and relative to "something" but this is a useful proxy still
      // A period with no transmissions will have a lower value...
      // The value has units of dB still
      //float energyProxy = (runningSum - floorRssi * runningCount) / -2.F / runningCount;
      float energyProxy = runningSum / -2.F / runningCount;
      if (longTermMean > 0) {
          longTermMean = energyProxy;
      } else {
          longTermMean = (energyProxy + longTermMean);
      }
      auto t1 = to_ms_since_boot(tNow);
      float background = longTermMean / (periods + 1);
      bool detection = energyProxy - background > 2;
      if (periods > 1 && detection) {
          printf("%8.2f %6.1f %6.1f    ", (t1 - t0) / 1000.F, background, energyProxy);
          // Bin this into 5dB slots from -127
          int nx = (energyProxy + 127.5) / 5;
          for (int i=0; i < nx; i++) { printf("*"); } printf("\n");
      } else if (periods < 1) { 
          printf("%8.2f %6.1f (initial integration)\n", (t1 - t0) / 1000.F, background);
      }
      tNextOutput = delayed_by_us(tNow, nextOutput_us);
      runningSum = 0;
      runningCount = 0;
      periods ++;
    }
  }
}

int main() {
    stdio_init_all();
    printf("\n\nCore0: Pico Oregon Serial Relay\n");

    RFM69Radio radio;
    radio.setPins(RFM69_MISO, RFM69_MOSI, RFM69_SCK, RFM69_CS, RFM69_IRQ, RFM69_RST);
    radio.setChipRate(RFM_OOK_CHIPRATE);
    radio.setFrequency(433920000);
    radio.begin(spi0);

    if (radio.bad()) {
      panic("Unable init radio spi\n");
    }
    printf("Version=0x%02x\n", radio.getVersion());
    radio.enableReceiver();
    //radio.dumpRegisters();

    queue_init(&decodedMessagesQueue, sizeof(int), decodedMessagesRingBuffer.getMaxElements());

    multicore_launch_core1(core1_main);
    core0_main(radio);
}
