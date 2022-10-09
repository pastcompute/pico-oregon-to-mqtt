# Introduction

This project describes another approach to building a relay for 433MHz Oregon weather sensors,
allowing easy integration into your home automation system.

This system uses just a Raspberry Pico or Pico-Wireless, and an RFM69 433MHz module.

With the Pico, data is output on the USB serial port allowing the Pico to be connected to another computer such as a Raspberry Pi to provide the MQTT network client. It could be directly connected to a machine already running OpenHAB or Homebridge, etc.

Traditionally Oregon connections use an Arduino or other microcontroller with 433MHz radio modules, typically with analogue superheterodyne design. Instead this solution uses an RFM69 with an Sx1231 chip, providing better filtering and detection and cleaner output, and RSSI information. Using the Pico we can take advantage of its two cores, dedicating the second core to capturing pulse timing with reduced jitter with the first core responsible for configuring the receiver module and providing serial communication and streaming RSSI information from the receiver.  We could then look at adding some additional intelligence to have the device switch frequencies or other parameters immediately after a successful decode to search for transmissions from multiple varying transmitters.
