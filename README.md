# Introduction

This project describes a modern approach to building a relay for 433MHz Oregon weather sensors,
allowing easy integration into your home automation system.
This system uses just a Raspberry Pico or Pico-Wireless, and an RFM69 433Mzh module.
With the Pico, data is output on the USB serial port allowing the Pico to be connected to another computer such as a Raspberry Pi to provide the MQTT network client. Uing a Pico-Wireless, the device can directly act as an MQTT client.
Traditionally Oregon connections use 433MHz radio modules typically with analogue superheterodyne design. The RFM69 using an Sx1231 chip provides better filtering and detection and cleaner output, and RSSI information. With the Pico we can take advantage of its
two cores, dedicating the second core to capturing pulse timing with reduced jitter with the first core responsible for controlling the
receiver module and serial or wireless communication, and also streaming RSSI information from the receiver.
We can also add some additional intelligence to have the device switch frequencies or other parameters immediately after a successful decode to search for transmissions from multiple varying devices.
