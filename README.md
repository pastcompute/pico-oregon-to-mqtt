# Introduction

This project describes another approach to building a relay for 433MHz Oregon weather sensors,
allowing easy integration into your home automation system.

Briefly, a Raspberry Pi Pico uses an RFM69 to decode and parse OOK and then write this on its serial port (or USB as serial) as CSV.  Any computer (a Raspberry Pi, Nuc or whatever) can then take this and with a small python program convert it to MQTT that can be ingested into a home automation system.

# Architecture

In my case, this is the following architecture:

```
       x 433MHz RF
       x
   xxxx xxxx
      x|x
    xxx|xxx
       |Discone antenna
       |
+------+--------+
|               |
|               |
|  RFM 69       |
|               |
|               |                                   Home Web / App
+-------+-------+                                         ^
        |SPI                                              |
        |                                                 |
+-------v-------+        +---------------+         +------+--------+
|               |        |               |         |      |        |
|               |        |  Raspberry Pi |         |      |        |
|Raspberry Pico | Serial |               |  +------+-->OpenHAB     |
|               +-------->  Raspbian     |  |      |               |
|               |        |  Mosquitto    |  +------+->HomeBridge   |
|               |        |               |  |      |      |        |
|               |        |MQTT Conversion+--+      |      |        |
+---------------+        +---------------+         +------+--------+
                                                          |
                                                          v
                                                     Apple Homekit
```

The MQTT messages I output, conform to the output of the rtl_433 program, which I was using previously for this task.
This meant I did not need to immediately modify the configuration of my OpenHAB and homebridge instances.

# System 

This system uses just a Raspberry Pico or Pico-Wireless, and an RFM69 433MHz module.

With the Pico, data is output on the USB serial port allowing the Pico to be connected to another computer such as a Raspberry Pi to provide the MQTT network client. It could be directly connected to a machine already running OpenHAB or Homebridge, etc.

Traditionally Oregon connections use an Arduino or other microcontroller with 433MHz radio modules, typically with analogue superheterodyne design. Instead this solution uses an RFM69 with an Sx1231 chip, providing better filtering and detection and cleaner output, and RSSI information. Using the Pico we can take advantage of its two cores, dedicating the second core to capturing pulse timing with reduced jitter with the first core responsible for configuring the receiver module and providing serial communication and streaming RSSI information from the receiver.  We could then look at adding some additional intelligence to have the device switch frequencies or other parameters immediately after a successful decode to search for transmissions from multiple varying transmitters.

For an underlying exploration, see https://github.com/pastcompute/pico-rfm69-ook-experiments

For a 3D printed enclosure, see https://github.com/pastcompute/openscad-rpipico-rfm69-container

For how I made the antenna, see https://community.element14.com/challenges-projects/project14/rf/b/blog/posts/building-a-poor-man-s-quarter-wave-433mhz-antenna-antenna-s-construction