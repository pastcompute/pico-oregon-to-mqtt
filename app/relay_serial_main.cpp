#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>

#include "decoder/DecodeOOK.hpp"
#include "decoder/OregonDecoderV2.hpp"
#include "decoder/OregonProtocol.hpp"
#include "constants.h"
#include "config.h"
#include "radio.hpp"

critical_section_t sensor_crit;

struct sensorData_t {
  uint8_t channel;
  uint8_t rollingCode;
  int16_t temp;
  uint8_t hum;
  uint16_t actualType;
  bool battOK;
};

static sensorData_t sensorData;

critical_section_t dio2_crit;

struct dio2_shared_data_t {
  volatile uint32_t edgesCount;
  volatile uint32_t nextPulseLength_us;
  volatile uint32_t now;
};

static dio2_shared_data_t sharedData = {
  .edgesCount = 0,
  .nextPulseLength_us = 0,
  .now = 0,
};

void dio2InterruptHandler() {
  static uint32_t prevTime_us = 0;
  if (gpio_get_irq_event_mask(RFM69_DIO2) & GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL) {
      gpio_acknowledge_irq(RFM69_DIO2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
  }
  // From the second and successive interrupt, sharedData.nextPulseLength_us will hold the time between successive interrupts
  // and local prevTime_us will track what micros() was
  // sharedData.nextPulseLength_us is cleared after being processed in the loop
  auto now = time_us_64(); // this may not handle wrap, but I think we can ignore that for the moment
  critical_section_enter_blocking(&dio2_crit);
  sharedData.now = now;
  sharedData.nextPulseLength_us = now - prevTime_us;
  sharedData.edgesCount++;
  critical_section_exit(&dio2_crit);
  prevTime_us = now;
}

void core1_main() {
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

  OregonDecoderV2 oregonV2Decoder;
  OregonProtocol protocol;
  while (true) {
    __wfi();
    
    // Every edge, try and decode a message
    // We have around 400 uS to get this done, worst case
 
    critical_section_enter_blocking(&dio2_crit);
    auto pulseLength_us = sharedData.nextPulseLength_us;
    sharedData.nextPulseLength_us = 0;
    auto now_us = sharedData.now;
    critical_section_exit(&dio2_crit);
    sensorData_t result;
    if (pulseLength_us != 0) {
      bool decoded = false;
      if (oregonV2Decoder.nextPulse(pulseLength_us)) {
        uint8_t len;
        const uint8_t* data = oregonV2Decoder.getData(len);
        if (data) {
          for (uint8_t i = 0; i < len; i++) {
            printf("%02x", data[i]);
          }
          printf("\n");
          // Rolling code is BCD...
          if (protocol.decodeTempHumidity(data, len,
                result.actualType, result.channel, result.rollingCode,
                result.temp, result.hum, result.battOK)) {
            decoded = true;
          }
          // if decoded is false it was either scrambled or something we dont understand yet
        }
        oregonV2Decoder.resetDecoder();
      }
      if (decoded) {
        // push the decoded msg back to the other core...
        auto t = to_ms_since_boot(get_absolute_time());
        printf("%d,%04x,%d,%x,%.1f,%d,Batt=%s,FIXMEdB\n", t, 
          result.actualType, result.channel, result.rollingCode, result.temp / 10.F, result.hum, result.battOK?"ok":"flat");
      }
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

    multicore_launch_core1(core1_main);
    core0_main(radio);
}
