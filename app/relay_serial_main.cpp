#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/spi.h>

#include "config.h"
#include "radio.h"

void core1_main() {
    printf("Core1: OOK Receiver\n");

    // Interrupt on the pin connected to DIO2

    while (true) {
      __wfi();
    }
}

int main() {
    stdio_init_all();
    printf("\n\nCore0: Pico Oregon Serial Relay\n");

    RFM69Radio radio;
    radio.setPins(RFM69_MISO, RFM69_MOSI, RFM69_SCK, RFM69_CS, RFM69_IRQ, RFM69_RST);
    radio.begin(spi0);

    if (radio.bad()) {
      panic("Unable init radio spi\n");
    }
    printf("Version=0x%02x\n", radio.getVersion());

    multicore_launch_core1(core1_main);

    int n=0;
    while (true) {
        sleep_ms(993);
        printf(n%2?"\r- ":"\r| ");
        n++;
    }
}
