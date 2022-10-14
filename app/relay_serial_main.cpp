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

#include "constants.h"
#include "config.h"
#include "types.hpp"
#include "ringbuf.hpp"
#include "radio.hpp"
#include "manchester.hpp"

/// Set this to true to have the second core dump all decoded manchester in hex to the serial port.
/// Note that unknown data is also output by core0
static bool debugMessageHex = false;

/// Allocate a fixed ring buffer that can hold up to 10 elements
typedef RingBuffer<DecodedMessageUnion_t, 10> RingBuffer_t;

/// Ring buffer store for the message queue. This is a lock free buffer where core1 writes and core0 reads
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

  auto mask = gpio_get_irq_event_mask(RFM69_DIO2);
  if (mask & GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL) {
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
  sharedData.fallingEdge = mask & GPIO_IRQ_EDGE_FALL;
  critical_section_exit(&dio2_crit);
  prevTime_us = now;
}

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
  manchesterHandler.debugMessageHex = debugMessageHex;

  DecodedMessageUnion_t* msg = decodedMessagesRingBuffer.reserve(); // pre-start the ring buffer
  // fallback memory if the ring buffer is full: let message decoding still complete, so pulses are processed in order
  DecodedMessageUnion_t dummy;
  while (true) {
    // Idle at low power when not procesing
    __wfi();

    // Pickup the next data if there is any    
    critical_section_enter_blocking(&dio2_crit);
    auto pulseLength_us = sharedData.nextPulseLength_us;
    sharedData.nextPulseLength_us = 0;
    auto now_us = sharedData.now;
    auto fallingEdge = sharedData.fallingEdge;
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
    if (!msg) {
      msg = decodedMessagesRingBuffer.reserve();
      // If the buffer is full, we will lose this message (it will be NULL)
    }
    bool decoded = manchesterHandler.nextPulse(pulseLength_us, now_us, msg ? *msg : dummy);
    if (decoded) {
      // Make available for other core
      decodedMessagesRingBuffer.advance();
      msg = NULL;
    }
  }
}

// On Core0, loop and measure RSSI and run the decoder when there is a new pulse
void core0_main(RFM69Radio& radio) {
  const int rssiPoll_us = 25;
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

    // This core is not so power efficient, we have to continually poll the timer and the ring buffer
    // A slightly more efficient design would have core1 signal core0 to wake up and check the ring buffer
    // but we would need to also integrate that with alarms

    auto tail = decodedMessagesRingBuffer.peek();
    if (tail) {
      decodedMessagesRingBuffer.pop();
      absolute_time_t t;
      switch (tail->base.baseType) {
        case DecodedMessage_t::BaseType_t::OREGON: {
          const OregonSensorData_t& od = tail->oregon;  
          update_us_since_boot(&t, od.rawTime_us);
          printf("%d,%04x,%d,%x,%.1f,%d,Batt=%s,FIXMEdB\n", to_ms_since_boot(t), 
            od.actualType, od.channel, od.rollingCode,
            od.temp / 10.F, od.hum, od.battOK?"ok":"flat");
          break;
        }

        case DecodedMessage_t::BaseType_t::UNDECODED: {
          const DecodedMessage_t& ud = tail->base;
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

        default:
          panic("Unknown type");
          break;
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
          // Bin this into 3dB slots from -127
          int nx = (energyProxy + 127.5F) / 3.F;
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

    multicore_launch_core1(core1_main);
    core0_main(radio);
}

// Notable observations:
// THGN123N temp + humid every 39 seconds
// Something not a V2 or V3, every 67 seconds (on the RTL-SDR) and 73 (here) - my freezer?