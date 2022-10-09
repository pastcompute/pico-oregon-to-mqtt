#ifndef POTM_RADIO_H_
#define POTM_RADIO_H_

#ifndef _PICO_STDLIB_H
#include <pico/stdlib.h>
#endif

#ifndef _HARDWARE_SPI_H
#include <hardware/spi.h>
#endif

#include <assert.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#include "pico/binary_info.h"


//#include "config.h"

// Instead of using an all-encompassing library such as RadioHead,
// Just provide convenience wrappers and leverage Pico hardware SPI
// Given this is a Pico-specific system, I haven't bothered abstracting the SPI

#include "rfm69.h"

class RFM69Radio {
private:
    uint8_t pin_miso;
    uint8_t pin_mosi;
    uint8_t pin_sck;
    uint8_t pin_cs;
    uint8_t pin_irq;
    uint8_t pin_rst;

    spi_inst_t* spi;

    volatile bool spiReadError;
    bool initError;

    uint8_t version;

public:
    RFM69Radio()
    : 
      pin_miso(13),
      pin_mosi(14),
      pin_sck(15),
      pin_cs(16),
      pin_irq(17),
      pin_rst(18),
      spiReadError(false),
      initError(false),
      version(-1)
    {}

    void setPins(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irq, uint8_t rst) {
      pin_miso = miso;
      pin_mosi = mosi;
      pin_sck = sck;
      pin_cs = cs;
      pin_irq = irq;
      pin_rst = rst;
    }

    bool bad() {
      return initError;
    }
    uint8_t getVersion() const {
      return version;
    }

    void begin(spi_inst_t* spiInst) {
      assert(spiInst);
      spi = spiInst;
      spi_init(spi, SPI_BAUD_RATE);
      gpio_set_function(pin_miso, GPIO_FUNC_SPI);
      gpio_set_function(pin_mosi, GPIO_FUNC_SPI);
      gpio_set_function(pin_sck, GPIO_FUNC_SPI);

      gpio_init(pin_cs);
      gpio_set_dir(pin_cs, GPIO_OUT);
      gpio_put(pin_cs, 1);

      gpio_init(pin_rst);
      gpio_set_dir(pin_rst, GPIO_OUT);
      gpio_put(pin_rst, 0);

      reset();
      init();
    }

    void reset() {
      gpio_put(pin_rst, 1); sleep_ms(10);
      gpio_put(pin_rst, 0); sleep_ms(10);
    }

    void spiSelect() {
      asm volatile("nop \n nop \n nop");
      gpio_put(pin_cs, 0);
      asm volatile("nop \n nop \n nop");      
    }
    void spiUnselect() {
      asm volatile("nop \n nop \n nop");
      gpio_put(pin_cs, 1);
      asm volatile("nop \n nop \n nop");      
    }

    void spiWriteReg(uint8_t addr, uint8_t value) {
      printf("[W] *%02x := %02x\n", addr, value);
      addr = addr | SX1231_SPI_WRITE_MASK;
      const uint8_t buf[2] = { addr, value };

      spiSelect();

      sleep_ms(10);
      spi_write_blocking(spi, buf, 2);
      sleep_ms(10);

      spiUnselect();
    }

    uint8_t spiReadReg(uint8_t addr) {

      // FIXME: it should be possible to call this in a batch outside this
      // But then I had noisy reads for some reason
      spiSelect();

      uint8_t dst = 0xaa;
      addr = addr & ~SX1231_SPI_WRITE_MASK;
      sleep_ms(10);
      spiReadError = spi_write_blocking(spi, &addr, 1) != 1;
      spiReadError = spiReadError || spi_read_blocking(spi, 0, &dst, 1) != 1;
      sleep_ms(10);

      spiUnselect();

      printf("[R] *%02x -> %02x (%s)\n", addr, dst, spiReadError ? "!" : "✓");
      return dst;
    }

    void init() {
      initError = false;
      version = spiReadReg(SX1231_REG_10_VERSION);
      if (spiReadError) {
        initError = true;
        return;
      }

      // Start in standby mode
      uint8_t opmode = spiReadReg(SX1231_REG_01_OPMODE);
      opmode = (opmode & ~SX1231_OPMODE_MODE_MASK) | SX1231_OPMODE_MODE_STBY;
      spiWriteReg(SX1231_REG_01_OPMODE, opmode);
      spiReadReg(SX1231_REG_01_OPMODE);
      while (!spiReadError && !(spiReadReg(SX1231_REG_27_IRQFLAGS1) & SX1231_IRQFLAGS1_MODEREADY));

      // We dont have a lot to set up because we will be running on continuous receive and not transmitting
      // So leave lots of stuff in default; if for some reason the boost was on turn it off
      // If high power boost set previously, disable it
      spiWriteReg(SX1231_REG_5A_TESTPA1, 0x55);
      spiWriteReg(SX1231_REG_5C_TESTPA2, 0x70);

      // Set an initial frequency
      writeFrequency(433920000);

      enableConOOK();

      // Read back the current operating mode & confirm we set the data mode succesfully
      uint8_t opMode = spiReadReg(SX1231_REG_01_OPMODE);
      uint8_t datMode = spiReadReg(SX1231_REG_02_DATAMODUL);
      uint8_t map1 = spiReadReg(SX1231_REG_25_DIOMAPPING1);
      uint8_t map2 = spiReadReg(SX1231_REG_26_DIOMAPPING2);

      // Go to rx mode
      opmode = spiReadReg(SX1231_REG_01_OPMODE);
      opmode = (opmode & ~SX1231_OPMODE_MODE_MASK) | SX1231_OPMODE_MODE_RX;
      spiWriteReg(SX1231_REG_01_OPMODE, opmode);
      spiReadReg(SX1231_REG_01_OPMODE);
      while (!spiReadError && !(spiReadReg(SX1231_REG_27_IRQFLAGS1) & SX1231_IRQFLAGS1_MODEREADY));

      spiReadReg(SX1231_REG_10_VERSION);
      spiReadReg(SX1231_REG_01_OPMODE);
      spiReadReg(SX1231_REG_01_OPMODE);

      uint32_t frq = readFrequency();

      // Note, DIO2 is always OOK out in Continuous mode
      printf("Actual OPMODE=%02x DATAMOD=%02x DIOMAP=%02x %02x FRQ=%u\n", opMode, datMode, map1, map2, frq);
    }

    void writeFrequency(uint32_t frequency_hz, bool cs=true) {
      // The frequency is 24 bits and set in three registers RegFrfMsb, Mid, Lsb
      // Fcarrier = Fstep x (24 bit value - FrF)
      // (Table 24 in the SX1231 manual)
      // Fstep = FXOSC / 2^19, where FXOSC = 32MHz
      // --> Fc = FXOSC / 2^19 x FrF
      // --> Frf = Fc * 2^19 / FXOSC
      uint64_t frf = (uint64_t(frequency_hz) << 19) / FXOSC;
      // Write to registers MSB, MID, LSB
      spiWriteReg(SX1231_REG_07_FRFMSB, (frf >> 16) & 0xff);
      spiWriteReg(SX1231_REG_08_FRFMID, (frf >> 8) & 0xff);
      spiWriteReg(SX1231_REG_09_FRFLSB, frf & 0xff);
    }

    uint32_t readFrequency(bool cs=true) {
      uint8_t f1 = spiReadReg(SX1231_REG_07_FRFMSB);
      uint8_t f2 = spiReadReg(SX1231_REG_08_FRFMID);
      uint8_t f3 = spiReadReg(SX1231_REG_09_FRFLSB);
      uint64_t frf = uint64_t((f1 << 16) | (f2 << 8) | f3) * FXOSC;
      return uint32_t(frf >> 19);
    }

    void enableConOOK(bool cs=true) {
      // Enable continuous OOK mode without bit synchronisation,
      // because we are trying to receive OOK transmissions from anywhere
      const uint8_t MODEM_CONFIG_OOK_CONT_NO_SYNC =
          0b01100000 |  // Continuous, no sync
          0b00001000 |  // OOK
          0b00000000 ;  // no OOK modulation shaping

      // Set the bandwidth to 100kHz with 4% DC cancellation
      // See SX1231 manual - Channel Filter - pages ~27,28
      const uint8_t MODEM_CONFIG_BW_100k_DCC_1 = 0b01001001;

      // Set the bit rate to correspond to an Oregon V2/V3 protocol transmitter
      // Although it was not obvious from the manual, this cleans up noise that remains otherwise
      // For OOK this is the chip rate so 2x the bitrate
      // Without setting the bitrate, the OOK decoder in Pulseview fails to work
      // and you can see significant noise on each bit
      const uint32_t OOK_BITRATE = RFM_OOK_CHIPRATE;
      const uint8_t brLSB = (FXOSC / OOK_BITRATE) & 0xff;
      const uint8_t brMSB = ((FXOSC / OOK_BITRATE) >> 8) & 0xff;    

      spiWriteReg(SX1231_REG_02_DATAMODUL, MODEM_CONFIG_OOK_CONT_NO_SYNC);
      spiWriteReg(SX1231_REG_03_BITRATEMSB, brMSB);
      spiWriteReg(SX1231_REG_04_BITRATELSB, brLSB);
      spiWriteReg(SX1231_REG_19_RXBW, MODEM_CONFIG_BW_100k_DCC_1);
    }
};

#pragma GCC diagnostic pop

#endif
