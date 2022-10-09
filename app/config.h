#ifndef POTM_CONFIG_H_
#define POTM_CONFIG_H_
// rx tx sck cs --> 0/4 3/7 2/6 



#define RFM69_MISO 4u
#define RFM69_MOSI 7u
#define RFM69_SCK 6u
#define RFM69_CS 5u

#define RFM69_RST 12u
#define RFM69_IRQ 10u

#define RFM69_DIO2 11u

#define RF_FREQUENCY_MHZ 433.92

// Oregon is 1024Hz, doubled for chiprate for the Manchester encoding
// Using higher even multiples may help the same code work for other transmitters
#define RFM_OOK_CHIPRATE (1024  * 2)

// Notional clock rate of RFM69 module
#define FXOSC 32000000

#define SPI_BAUD_RATE (1000*1000)

#endif
