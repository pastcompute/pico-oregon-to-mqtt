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

#define RADIO_SPI_DEBUG printf
//#define RADIO_SPI_DEBUG()

// Instead of using an all-encompassing library such as RadioHead,
// Just provide convenience wrappers and leverage Pico hardware SPI
// Given this is a Pico-specific system, I haven't bothered abstracting the SPI
// Also I havent  bothered making a generic API like what I did for SX1276 once before
// This code is specifically optimised for performing OOK reception

#include "sx1231.h"

class RFM69Radio {
private:
    uint8_t pin_miso_;
    uint8_t pin_mosi_;
    uint8_t pin_sck_;
    uint8_t pin_cs_;
    uint8_t pin_irq_;
    uint8_t pin_rst_;

    spi_inst_t* spi_;

    volatile bool spiReadError_;
    bool initError_;

    uint8_t version_;
    uint16_t chipRate_;
    uint32_t frequency_;

public:

    // Notional clock rate of RFM69 module
    enum { FXOSC = 32000000u };

    RFM69Radio()
    : 
      pin_miso_(13),
      pin_mosi_(14),
      pin_sck_(15),
      pin_cs_(16),
      pin_irq_(17),
      pin_rst_(18),
      spiReadError_(false),
      initError_(false),
      version_(-1),
      chipRate_(10),
      frequency_(434000000)
    {}

    // Ignored after begin()
    void setPins(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irq, uint8_t rst) {
      pin_miso_ = miso;
      pin_mosi_ = mosi;
      pin_sck_ = sck;
      pin_cs_ = cs;
      pin_irq_ = irq;
      pin_rst_ = rst;
    }

    // Ignored after begin()
    void setChipRate(uint16_t chipRate) {
      chipRate_ = chipRate;
    }
    void setFrequency(uint32_t frequency) {
      frequency_ = frequency;
    }

    bool bad() const {
      return initError_;
    }
    uint8_t getVersion() const {
      return version_;
    }

    void begin(spi_inst_t* spiInst) {
      assert(spiInst);
      spi_ = spiInst;
      spi_init(spi_, SPI_BAUD_RATE);
      gpio_set_function(pin_miso_, GPIO_FUNC_SPI);
      gpio_set_function(pin_mosi_, GPIO_FUNC_SPI);
      gpio_set_function(pin_sck_, GPIO_FUNC_SPI);

      gpio_init(pin_cs_);
      gpio_set_dir(pin_cs_, GPIO_OUT);
      gpio_put(pin_cs_, 1);

      gpio_init(pin_rst_);
      gpio_set_dir(pin_rst_, GPIO_OUT);
      gpio_put(pin_rst_, 0);

      reset();
      init();
    }

    void reset() {
      gpio_put(pin_rst_, 1); sleep_ms(10);
      gpio_put(pin_rst_, 0); sleep_ms(10);
    }

    void spiSelect() {
      asm volatile("nop \n nop \n nop");
      gpio_put(pin_cs_, 0);
      asm volatile("nop \n nop \n nop");      
    }
    void spiUnselect() {
      asm volatile("nop \n nop \n nop");
      gpio_put(pin_cs_, 1);
      asm volatile("nop \n nop \n nop");      
    }

    void spiWriteReg(uint8_t addr, uint8_t value) {
      RADIO_SPI_DEBUG("[W] *%02x := %02x\n", addr, value);
      addr = addr | SX1231_SPI_WRITE_MASK;
      const uint8_t buf[2] = { addr, value };
      spiSelect();
      spi_write_blocking(spi_, buf, 2);
      spiUnselect();
    }

    void spiWriteRegN(uint8_t addr, const uint8_t *value, uint8_t numValues) {
      uint8_t buf[numValues+1] = { addr | SX1231_SPI_WRITE_MASK };
      for (uint8_t n=0; n < numValues; n++) { 
        RADIO_SPI_DEBUG("[W] *%02x := %02x\n", addr+n, value[n]);
        buf[n+1] = value[n];
      }
      spiSelect();
      spi_write_blocking(spi_, buf, 1 + numValues);
      spiUnselect();
    }

    uint8_t spiReadReg(uint8_t addr, bool alwaysSuppressDebug=false) {
      spiSelect();
      uint8_t dst = 0xaa;
      addr = addr & ~SX1231_SPI_WRITE_MASK;
      spiReadError_ = spi_write_blocking(spi_, &addr, 1) != 1;
      spiReadError_ = spiReadError_ || spi_read_blocking(spi_, 0, &dst, 1) != 1;
      spiUnselect();
      if (!alwaysSuppressDebug) { RADIO_SPI_DEBUG("[R] *%02x -> %02x (%s)\n", addr, dst, spiReadError_ ? "!" : "✓"); }
      return dst;
    }

    void init() {
      initError_ = false;
      version_ = spiReadReg(SX1231_REG_10_VERSION);
      if (spiReadError_) {
        initError_ = true;
        return;
      }

      // Start in standby mode
      uint8_t opmode = spiReadReg(SX1231_REG_01_OPMODE);
      opmode = (opmode & ~SX1231_OPMODE_MODE_MASK) | SX1231_OPMODE_MODE_STBY;
      spiWriteReg(SX1231_REG_01_OPMODE, opmode);
      spiReadReg(SX1231_REG_01_OPMODE);
      while (!spiReadError_ && !(spiReadReg(SX1231_REG_27_IRQFLAGS1) & SX1231_IRQFLAGS1_MODEREADY));

      // Note, our configuration is contingent on having just been reset

      // Set an initial frequency
      writeFrequency(frequency_);

      // Prepare for OOK reception
      enableConOOK();

      // Read back the current operating mode & confirm we set the data mode succesfully
      uint8_t opMode = spiReadReg(SX1231_REG_01_OPMODE);
      uint8_t datMode = spiReadReg(SX1231_REG_02_DATAMODUL);
      uint8_t map1 = spiReadReg(SX1231_REG_25_DIOMAPPING1);
      uint8_t map2 = spiReadReg(SX1231_REG_26_DIOMAPPING2);

      uint32_t frq = readFrequency();

      // Note, DIO2 is always OOK out in Continuous mode
      printf("Actual OPMODE=%02x DATAMOD=%02x DIOMAP=%02x %02x FRQ=%u\n", opMode, datMode, map1, map2, frq);
    }
  
    void enableReceiver() {
      // Go to rx mode
      uint8_t opmode = spiReadReg(SX1231_REG_01_OPMODE);
      opmode = (opmode & ~SX1231_OPMODE_MODE_MASK) | SX1231_OPMODE_MODE_RX;
      spiWriteReg(SX1231_REG_01_OPMODE, opmode);
      while (!spiReadError_ && !(spiReadReg(SX1231_REG_27_IRQFLAGS1) & SX1231_IRQFLAGS1_MODEREADY));
      opmode = spiReadReg(SX1231_REG_01_OPMODE);
      printf("Actual OPMODE=%02x\n", opmode);
    }

    void writeFrequency(uint32_t frequency_hz, bool cs=true) {
      // The frequency is 24 bits and set in three registers RegFrfMsb, Mid, Lsb
      // Fcarrier = Fstep x (24 bit value - FrF)
      // (Table 24 in the SX1231 manual)
      // Fstep = FXOSC / 2^19, where FXOSC = 32MHz
      // --> Fc = FXOSC / 2^19 x FrF
      // --> Frf = Fc * 2^19 / FXOSC
      uint64_t frf = (uint64_t(frequency_hz) << 19) / FXOSC;
      // Write to successive registers MSB, MID, LSB
      uint8_t buf[3] = { (frf >> 16) & 0xff, (frf >> 8) & 0xff, frf & 0xff };
      spiWriteRegN(SX1231_REG_07_FRFMSB, buf, 3);
    }

    uint32_t readFrequency(bool cs=true) {
      uint8_t f1 = spiReadReg(SX1231_REG_07_FRFMSB);
      uint8_t f2 = spiReadReg(SX1231_REG_08_FRFMID);
      uint8_t f3 = spiReadReg(SX1231_REG_09_FRFLSB);
      uint64_t frf = uint64_t((f1 << 16) | (f2 << 8) | f3) * FXOSC;
      return uint32_t(frf >> 19);
    }

    static inline float byteToRSSI(uint8_t rssi) {
      return rssi / -2.0F;
    }
    
    uint8_t readRSSIByte() {
        return spiReadReg(SX1231_REG_24_RSSIVALUE, true);
    }

    void enableConOOK(bool cs=true) {

      // Set a value for FDEV that works for us; 512 x ~61hZ ~= 30.7kHz
      // The 20dB BW = 2x(FDEV + BRate/2), so this gives us approx 64kHz 20dB bandwidth, the default is too narrow
      spiWriteReg(SX1231_REG_05_FDEVMSB, 0x10);
      spiWriteReg(SX1231_REG_06_FDEVLSB, 0x00);
      // And AFC; DCC_FREQ=0b111 (DC Offset cancellation) used during AFC as opposed to OOK (so this could be redundant)
      spiWriteReg(SX1231_REG_1A_AFCBW, 0xe0);
      spiWriteReg(SX1231_REG_6F_TESTDAGC, 0x30);

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
      const uint8_t brLSB = (FXOSC / chipRate_) & 0xff;
      const uint8_t brMSB = ((FXOSC / chipRate_) >> 8) & 0xff;    

      spiWriteReg(SX1231_REG_02_DATAMODUL, MODEM_CONFIG_OOK_CONT_NO_SYNC);
      spiWriteReg(SX1231_REG_03_BITRATEMSB, brMSB);
      spiWriteReg(SX1231_REG_04_BITRATELSB, brLSB);
      spiWriteReg(SX1231_REG_19_RXBW, MODEM_CONFIG_BW_100k_DCC_1);
    }

    void dumpRegisters() {
      const uint8_t numRegisters = 0x72;
      uint8_t reg[numRegisters] = { 0 };
      for (uint8_t i=0; i < numRegisters; i++) {
        spiSelect();
        spi_write_blocking(spi_, &i, 1);
        spi_read_blocking(spi_, 0, reg+i, 1);
        spiUnselect();
      }
      for (int i=0; i < numRegisters;) {
        printf("%02x", reg[i]);
        i++;
        if (i % 16) {
          printf(" ");
        } else {
          printf("\n");
        }
      }
      printf("\n");
    }
};

#pragma GCC diagnostic pop

#endif
