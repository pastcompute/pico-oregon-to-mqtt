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
#include "spectrograph.hpp"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

/// Set this to true to have the second core dump all decoded manchester in hex to the serial port.
/// Note that unknown data is also output by core0
static bool debugMessageHex = false;

/// Set this to true to output a spectograph of RSSI detections on Core0
static bool debugSpectrograph = true;

/// Set this to true to suppress decoder output and instead stream continuous spectrograph
static bool continuousSpectrograph = false;

/// Global statistical information
static struct Stats_t {
  uint32_t rawCount;
  uint32_t perTypeUniqueCount[DecodedMessage_t::BASETYPE_MAX];

  Stats_t() {
    rawCount = 0;
    std::fill(perTypeUniqueCount, perTypeUniqueCount + DecodedMessage_t::BASETYPE_MAX, 0);
  }
} stats;

/// Allocate a fixed ring buffer that can hold up to 15 elements
/// Until we implement message merging in the decoder in Core1,
/// this lets us cope with a Lacross that repeats a message 12 times happening at the same time as an oregon (etc)
typedef RingBuffer<DecodedMessage_t, 15> RingBuffer_t;

/// Ring buffer store for the message queue. This is a lock free buffer where core1 writes and core0 reads
static RingBuffer_t decodedMessagesRingBuffer;

/// Queue (FIFO) for pushing recent RSSI from Core0 to Core1 so that it can be associated with detection of a preamble
queue_t rssiFifo;

// The larger this can be and still work, the less time is used in processing
// although there could be a miss in the peak read from the sx1231

/// RSSI polling interval
const int rssiPoll_us = 100;

// The smaller these can be and still work, the shorter the time to flush

/// Recent RSSI queue size
const int rssiFifoQueueLength = 16000 / rssiPoll_us;

/// Recent RSSI queue elements to keep at least this many of
const int rssiFifoQueueWatermark = 10000 / rssiPoll_us;

/// RSSI spectrograph integration interval
auto spectrographIntegation_us = ONE_SECOND_US / 4;
struct TimestampedRssi_t {
  uint32_t t;
  uint8_t rssiByte;
};

// Flash LED for approx 0.5 seconds each new message. If it sticks on we had a hang in somewhere after posting
const auto messageLedMinTime_us = ONE_SECOND_US / 2;

/// This critical section protects data shared between the dio2 interrupt handler and the second core main loop.
static critical_section_t dio2_crit;

/// This data is shared between the dio2 interrupt handler and the second core main loop
static Dio2SharedData_t sharedData;

/// Common time on entry to core0 main loop
static absolute_time_t core0now;

/// RFM69 SX1231 dio2 Interrupt handler
extern "C" void dio2InterruptHandler() {
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
  // nb. this wont handle wrap, but I dont think we have to worry about what happens in 584924 years time...
  // OTOH, nextPulseLength_us will overflow after 4295 seconds, but that can also be disregarded
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

  DecodedMessage_t* msg = decodedMessagesRingBuffer.reserve(); // pre-start the ring buffer
  // fallback memory if the ring buffer is full: let message decoding still complete, so pulses are processed in order
  DecodedMessage_t dummy;

  bool preambleLatch = false;
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
    // We have around ~400 uS to get this done, worst case, when processing valid Oregon messages
    // and ~200uS for Lacrosse messages
    // The decoder fails as soon as it can
    if (!msg) {
      msg = decodedMessagesRingBuffer.reserve();
      // If the buffer is full, we will lose this message (it will be NULL)
    }
    TimestampedRssi_t rssi = { 0, 0 };
    bool decoded = manchesterHandler.nextPulse(pulseLength_us, now_us, msg ? *msg : dummy);
    if (decoded) {
      // Scan for a peak for likely the most recent 400-odd samples ; we will assign it to the current message
      // We are looking at rssi values from eldest to newest
      // Note, there is an edge case if we had two messages very near to each other, the wrong or no rssi might be assigned
      uint8_t rssiBytePeak = 255;
      int scanned = 0;
      while (true) {
        if (!queue_try_remove(&rssiFifo, &rssi)) { break; }
        if (rssiBytePeak > rssi.rssiByte) {
          rssiBytePeak = rssi.rssiByte;
        }
        scanned++;
        // Should we sanity check the timestamps, if they are too far out, then ignore?
      }
      if (msg) { msg->rssi = rssiBytePeak / -2.F; }
      // Make the completed message available for other core
      decodedMessagesRingBuffer.advance();
      // Turn LED on, other core will turn it off 250ms after it processes it
      gpio_put(LED_PIN, 1);
      msg = NULL;
    } else {
      // this will stay true until the message is reset... which wont be long if this was not actually a valid preamble
      bool preamble = manchesterHandler.maybePreamble();
      // if (preamble) { if (!preambleLatch) { preambleLatch = true; printf("+\n"); } } else { preambleLatch = false; }
      if (!preamble) {
        // We get here if we have 12 lacrosse message bits or 16 oregon bits
        // So, either 833*8 + 675*12 us or 16*1024us+preamble
        // So if we sample 14msec of RSSI we should have a good idea
        // flush stale RSSI values, provided we have at least 12msec in the queue
        auto n = queue_get_level_unsafe(&rssiFifo);
        // TODO - analyse if this potentially takes too long...
        for (int i=rssiFifoQueueWatermark; i < n; i++) {
          queue_try_remove(&rssiFifo, &rssi);
        }
      }
    }
  }
}

/// Output a message in psuedo-CSV format on serial port (to host)
void displayMessage(const EnrichedMessage_t& msg) {
  switch(msg.baseType) {
    case EnrichedMessage_t::BaseType_t::OREGON:
      printf("Oregon,%d,%04x,%d,%x,%.1f,%d,%s,%.1f,%d\n", msg.timestamp_ms,
          msg.oregon.actualType, msg.oregon.channel, msg.oregon.rollingCode, msg.oregon.temp / 10.F, msg.oregon.hum,
          msg.oregon.battOK?"ok":"flat", msg.rssi, msg.count);
      break;

    case EnrichedMessage_t::BaseType_t::LACROSSE:
      printf("Lacrosse,%d,%d,%.1f,%s,%.1f,%d\n", msg.timestamp_ms,
        msg.lacrosse.id, msg.lacrosse.channel, (msg.lacrosse.temp - 500.F) / 10.F,
        msg.lacrosse.battOK?"ok":"flat", msg.rssi, msg.count);
      break;

    case EnrichedMessage_t::BaseType_t::UNDECODED:
      printf("%d,UNK,%s,%.1f\n", msg.timestamp_ms, bytesToHex(msg.bytes, msg.len).c_str(), msg.rssi);
      break;

    default:
      panic("Unknown type");
  }
}

/// Handle a message popped from the ring buffer.
/// Enrich with additional metadata (type count, converted time, etc.)
void handleMessage(const DecodedMessage_t& decodedMessage) {
  // Total count
  stats.rawCount++;

  // Discard if this is a duplicate in a burst
  if (decodedMessage.isRecentDupe(decodedMessage, 750000)) { return; }

  // Convert the raw interrupt time of the message
  absolute_time_t t; update_us_since_boot(&t, decodedMessage.rawTime_us);
  auto tb = to_ms_since_boot(t);

  // Update the per-type count of the message
  auto count = ++stats.perTypeUniqueCount[decodedMessage.baseType];

  EnrichedMessage_t message(decodedMessage, count, tb);
  displayMessage(message);
}

void displaySpectrographBar(float energy, float background, uint32_t t0) {
  printf("Spectro,%8.2f %6.1f %6.1f    ", (to_ms_since_boot(core0now) - t0) / 1000.F, background, energy);
    // Bin this into 3dB slots from -127
    int nx = (energy + 127.5F) / 3.F;
    for (int i=0; i < nx; i++) { printf("*"); } printf("\n");
}

// On Core0, loop and measure RSSI and run the decoder when there is a new pulse
void core0_main(RFM69Radio& radio) {
  core0now = get_absolute_time();
  absolute_time_t tNextSpectrograph = delayed_by_us(core0now, spectrographIntegation_us);
  absolute_time_t tNextPoll = delayed_by_us(core0now, rssiPoll_us);
  absolute_time_t tNextLEDCheck = delayed_by_us(core0now, messageLedMinTime_us);

  Spectrograph spectrograph;
  TimestampedRssi_t rssi = { 0, 0 };
  bool latchRssiQueueFull = false;
  bool needToTurnOffLed = true;
  while (true) {

    // This core is not so power efficient, we have to continually poll the timer and the ring buffer
    // A slightly more efficient design would have core1 signal core0 to wake up and check the ring buffer
    // but we would need to also integrate that with alarms

    core0now = get_absolute_time();
    auto tSinceBoot = to_ms_since_boot(core0now);

    auto tail = decodedMessagesRingBuffer.peek();
    if (tail) {
      decodedMessagesRingBuffer.pop(); // release a slot ASAP
      handleMessage(*tail);
      // if (!continuousSpectrograph) {
      //   outuptDecoder(*tail);
      // }
      // if LED had been hopefully turned on, schedule to turn it off
      tNextLEDCheck = delayed_by_us(core0now, messageLedMinTime_us);
      // this is hacky we should use an alarm instead
      needToTurnOffLed = true;
    }
    if (needToTurnOffLed && time_reached(tNextLEDCheck)) {
      gpio_put(LED_PIN, 0);
      needToTurnOffLed = false;
    }

    if (time_reached(tNextPoll)) {
      tNextPoll = delayed_by_us(core0now, rssiPoll_us);
      rssi.rssiByte = radio.readRSSIByte();
      rssi.t = tSinceBoot;
      if (!queue_try_add(&rssiFifo, &rssi)) {
        // if (!latchRssiQueueFull) { latchRssiQueueFull = true; printf("!\n"); }
      } else {
        latchRssiQueueFull = false;
      }
      spectrograph.updateRssi(rssi.rssiByte);
    }

    if (time_reached(tNextSpectrograph)) {
      // Capture the time the first time integrate() was called
      static auto t0 = tSinceBoot;
      spectrograph.integrate();
      if (debugSpectrograph && !continuousSpectrograph && spectrograph.getDetection()) {
        displaySpectrographBar(spectrograph.getEnergy(), spectrograph.getBackground(), t0);
      } else if (continuousSpectrograph) {
        displaySpectrographBar(spectrograph.getEnergy(), spectrograph.getBackground(), t0);
      }
      tNextSpectrograph = delayed_by_us(core0now, spectrographIntegation_us);
    }
  }
}

int main() {
    // Turn LED on
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
    sleep_ms(250);
    gpio_put(LED_PIN, 0);

    stdio_init_all();
    printf("\n\nCore0: Pico Oregon Serial Relay\n");

  #if !NDEBUG
    printf("\n\nCore0: DEBUG BUILD\n");
  #endif

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

    // Turn LED on a second time, if we got to here
    sleep_ms(250);
    gpio_put(LED_PIN, 1);
    sleep_ms(250);
    gpio_put(LED_PIN, 0);

    // Push RSSI samples through to the other core
    // We want to cover at least 12-14 milliseconds of message data to find a peak in RSSI
    // So buffer that muchl Core1 will let it fill up, and then keep removing old values
    // Once it has started detecting a message, then this FIFO should fill and data will be lost;
    // and then core1 will flush it back again and sample for the peak
    queue_init(&rssiFifo, sizeof(TimestampedRssi_t), rssiFifoQueueLength);

    multicore_launch_core1(core1_main);
    core0_main(radio);
}

// Notable observations:
// THGN123N temp + humid every 39 seconds
// Something not a V2 or V3, every 67 seconds (on the RTL-SDR) and 73 (here) - my freezer?