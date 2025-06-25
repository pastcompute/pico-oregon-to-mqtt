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

# Sample output

I have two freezer alarms (these were identiifed by RTL433 as Lacrosse)

Serial:
```
Lacrosse,200431522,24.8,182,-22.7,ok,-54.0,6886
Oregon,200435223,24.8,1d20,1,27,19.3,89,ok,-62.5,4001
STATUS,200445893,24.3,-88.4,90494,2483,10555
STATUS,200460893,24.3,-88.4,90494,2482,10756
Lacrosse,200470599,26.2,143,-17.0,ok,-72.5,6887
```

MQTT:
```
rtl_433/unifi2/devices/Oregon-THGR122N/1/111/channel 1
rtl_433/unifi2/devices/Oregon-THGR122N/1/111/temperature_C 19.3
rtl_433/unifi2/devices/Oregon-THGR122N/1/111/humidity 89
rtl_433/unifi2/devices/Oregon-THGR122N/1/111/battery_ok 1
rtl_433/unifi2/devices/Oregon-THGR122N/1/111/rssi -62.0
rtl_433/unifi2/devices/LaCrosse-TX141Bv2/2/182/channel 2
rtl_433/unifi2/devices/LaCrosse-TX141Bv2/2/182/temperature_C -22.5
rtl_433/unifi2/devices/LaCrosse-TX141Bv2/2/182/battery_ok 1
rtl_433/unifi2/devices/LaCrosse-TX141Bv2/2/182/rssi -53.5
```

Note I have seen the identifier change in the past (`111`, `143`, `182` in the above example), perhaps on a battery change.

The Python code still needs improvements:
- pull in the password, user, IP, expected idenitfiers from configuration
- a systemd script (at the moment I start a screen session after a power outage and run it there)

Configuration snippet for Homebridge:
```
        {
            "accessory": "mqttthing",
            "type": "temperatureSensor",
            "name": "BackVerandahT",
            "url": "mqtt://REDACTED",
            "username": "REDACTED",
            "password": "REDACTED",
            "caption": "Back Verandah Temp",
            "topics": {
                "getCurrentTemperature": "rtl_433/unifi2/devices/Oregon-THGR122N/1/111/temperature_C",
                "minTemperature": -20,
                "maxTemperature": -60
            }
        },
        {
            "accessory": "mqttthing",
            "type": "humiditySensor",
            "name": "BackVerandahH",
            "caption": "Back Verandah Humidity",
            "topics": {
                "getCurrentRelativeHumidity": "rtl_433/unifi2/devices/Oregon-THGR122N/1/111/humidity"
            }
        },
        {
            "accessory": "mqttthing",
            "type": "temperatureSensor",
            "name": "DeepFreeze",
            "caption": "Deep Freeze",
            "topics": {
                "getCurrentTemperature": "rtl_433/unifi2/devices/LaCrosse-TX141Bv2/2/182/temperature_C",
                "minTemperature": -23,
                "maxTemperature": -15
            }
        },
        {
            "accessory": "mqttthing",
            "type": "temperatureSensor",
            "name": "Freezer",
            "caption": "Freezer",
            "topics": {
                "getCurrentTemperature": "rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/temperature_C",
                "minTemperature": -21,
                "maxTemperature": -12
            }
        },
```

# System 

This system uses just a Raspberry Pico or Pico-Wireless, and an RFM69 433MHz module.

With the Pico, data is output on the USB serial port allowing the Pico to be connected to another computer such as a Raspberry Pi to provide the MQTT network client. It could be directly connected to a machine already running OpenHAB or Homebridge, etc.

Traditionally Oregon connections use an Arduino or other microcontroller with 433MHz radio modules, typically with analogue superheterodyne design. Instead this solution uses an RFM69 with an Sx1231 chip, providing better filtering and detection and cleaner output, and RSSI information. Using the Pico we can take advantage of its two cores, dedicating the second core to capturing pulse timing with reduced jitter with the first core responsible for configuring the receiver module and providing serial communication and streaming RSSI information from the receiver.  We could then look at adding some additional intelligence to have the device switch frequencies or other parameters immediately after a successful decode to search for transmissions from multiple varying transmitters.

# References

For an underlying exploration, see https://github.com/pastcompute/pico-rfm69-ook-experiments

For a 3D printed enclosure, see https://github.com/pastcompute/openscad-rpipico-rfm69-container

How I made the antenna was inspired by this: https://community.element14.com/challenges-projects/project14/rf/b/blog/posts/building-a-poor-man-s-quarter-wave-433mhz-antenna-antenna-s-construction

For decoding Oregon I modified an existing Arduino library, then extended it with support for the Lacrosse by inspecting the source code of RTL433 (files in [app/decoder/]() )
