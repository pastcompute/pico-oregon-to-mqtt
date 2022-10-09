#define SX1231_SPI_WRITE_MASK 0x80

#define SX1231_REG_00_FIFO          0x00
#define SX1231_REG_01_OPMODE        0x01
#define SX1231_REG_02_DATAMODUL     0x02
#define SX1231_REG_03_BITRATEMSB    0x03
#define SX1231_REG_04_BITRATELSB    0x04
#define SX1231_REG_05_FDEVMSB       0x05
#define SX1231_REG_06_FDEVLSB       0x06
#define SX1231_REG_07_FRFMSB        0x07
#define SX1231_REG_08_FRFMID        0x08
#define SX1231_REG_09_FRFLSB        0x09
#define SX1231_REG_0A_OSC1          0x0a
#define SX1231_REG_0B_AFCCTRL       0x0b
#define SX1231_REG_0C_RESERVED      0x0c
#define SX1231_REG_0D_LISTEN1       0x0d
#define SX1231_REG_0E_LISTEN2       0x0e
#define SX1231_REG_0F_LISTEN3       0x0f
#define SX1231_REG_10_VERSION       0x10
#define SX1231_REG_11_PALEVEL       0x11
#define SX1231_REG_12_PARAMP        0x12
#define SX1231_REG_13_OCP           0x13
#define SX1231_REG_14_RESERVED      0x14
#define SX1231_REG_15_RESERVED      0x15
#define SX1231_REG_16_RESERVED      0x16
#define SX1231_REG_17_RESERVED      0x17
#define SX1231_REG_18_LNA           0x18
#define SX1231_REG_19_RXBW          0x19
#define SX1231_REG_1A_AFCBW         0x1a
#define SX1231_REG_1B_OOKPEAK       0x1b
#define SX1231_REG_1C_OOKAVG        0x1c
#define SX1231_REG_1D_OOKFIX        0x1d
#define SX1231_REG_1E_AFCFEI        0x1e
#define SX1231_REG_1F_AFCMSB        0x1f
#define SX1231_REG_20_AFCLSB        0x20
#define SX1231_REG_21_FEIMSB        0x21
#define SX1231_REG_22_FEILSB        0x22
#define SX1231_REG_23_RSSICONFIG    0x23
#define SX1231_REG_24_RSSIVALUE     0x24
#define SX1231_REG_25_DIOMAPPING1   0x25
#define SX1231_REG_26_DIOMAPPING2   0x26
#define SX1231_REG_27_IRQFLAGS1     0x27
#define SX1231_REG_28_IRQFLAGS2     0x28
#define SX1231_REG_29_RSSITHRESH    0x29
#define SX1231_REG_2A_RXTIMEOUT1    0x2a
#define SX1231_REG_2B_RXTIMEOUT2    0x2b
#define SX1231_REG_2C_PREAMBLEMSB   0x2c
#define SX1231_REG_2D_PREAMBLELSB   0x2d
#define SX1231_REG_2E_SYNCCONFIG    0x2e
#define SX1231_REG_2F_SYNCVALUE1    0x2f

#define SX1231_REG_37_PACKETCONFIG1 0x37
#define SX1231_REG_38_PAYLOADLENGTH 0x38
#define SX1231_REG_39_NODEADRS      0x39
#define SX1231_REG_3A_BROADCASTADRS 0x3a
#define SX1231_REG_3B_AUTOMODES     0x3b
#define SX1231_REG_3C_FIFOTHRESH    0x3c
#define SX1231_REG_3D_PACKETCONFIG2 0x3d
#define SX1231_REG_3E_AESKEY1       0x3e

#define SX1231_REG_4E_TEMP1         0x4e
#define SX1231_REG_4F_TEMP2         0x4f
#define SX1231_REG_58_TESTLNA       0x58
#define SX1231_REG_5A_TESTPA1       0x5a
#define SX1231_REG_5C_TESTPA2       0x5c
#define SX1231_REG_6F_TESTDAGC      0x6f
#define SX1231_REG_71_TESTAFC       0x71

#define SX1231_OPMODE_MODE_MASK     0b00011100
#define SX1231_OPMODE_MODE_STBY     0b00000100
#define SX1231_OPMODE_MODE_RX       0b00010000

#define SX1231_IRQFLAGS1_MODEREADY  0b10000000