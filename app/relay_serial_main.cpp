#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>

#include "constants.h"
#include "config.h"
#include "radio.h"

void runTaskOokScope(RFM69Radio&);

critical_section_t crit1;

struct shared_data_t {
    volatile uint32_t edgesCount;
    volatile uint32_t nextPulseLength_us;
    volatile uint32_t now;
};

static shared_data_t sharedData = {
    .edgesCount = 0,
    .nextPulseLength_us = 0,
    .now = 0,
};

void dio2InterruptHandler() {
    if (gpio_get_irq_event_mask(RFM69_DIO2) & GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL) {
        gpio_acknowledge_irq(RFM69_DIO2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    }

    static uint32_t prevTime_us = 0;

    // From the second and successive interrupt, sharedData.nextPulseLength_us will hold the time between successive interrupts
    // and local prevTime_us will track what micros() was
    // sharedData.nextPulseLength_us is cleared after being processed in the loop
    auto now = time_us_64(); // this may not handle wrap, but I think we can ignore that for the moment
    critical_section_enter_blocking(&crit1);
    sharedData.now = now;
    sharedData.nextPulseLength_us = now - prevTime_us;
    sharedData.edgesCount++;
    critical_section_exit(&crit1);
    prevTime_us = now;
}

void core1_main() {
    printf("Core1: OOK Receiver\n");

    sharedData.edgesCount = 0;
    sharedData.nextPulseLength_us = 0;

    critical_section_init(&crit1);

    // Interrupt on the pin connected to DIO2, with pulldown
    gpio_init(RFM69_DIO2);
    gpio_pull_down(RFM69_DIO2);
    gpio_set_dir(RFM69_DIO2, false);
    gpio_set_input_enabled(RFM69_DIO2, true);
    gpio_add_raw_irq_handler(RFM69_DIO2, &dio2InterruptHandler);
    gpio_set_irq_enabled(RFM69_DIO2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    while (true) {
        __wfi();
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

    radio.dumpRegisters();

    multicore_launch_core1(core1_main);

    runTaskOokScope(radio);
}

void runTaskOokScope(RFM69Radio& radio) {
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
        critical_section_enter_blocking(&crit1);
        auto pulseLength_us = sharedData.nextPulseLength_us;
        sharedData.nextPulseLength_us = 0;
        auto now_us = sharedData.now;
        critical_section_exit(&crit1);


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

        // And this is not using a real time O/S, so perhaps this will delay the next sample slightly
        auto t1 = to_ms_since_boot(get_absolute_time());

        float background = longTermMean / (periods + 1);

        bool detection = energyProxy - background > 1;

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

void handleDio2Edge(uint, uint32_t) {

}
